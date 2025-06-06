#include "aic.h"
#include "aic_regs.h"
#include "cpu_regs.h"
#include "exception.h"
#include "gxf.h"
#include "iodev.h"
#include "memory.h"
#include "uart.h"
#include "xnuboot.h"

extern u32 _vt_vectors_start[];
extern u64 _vt_m1n1_mmu[];
extern void *g_xnu_entry;
extern u64 _vt_m1n1_call;
extern struct vt_mmu_info _vt_mmuinfo;

u32 *_vt_vectors_start_va;
u64 vt_pt_walk(u64 addr, u64 *ttbr_reg);
u64 *vt_pt_getl3(u64 addr, u64 *ttbr_reg);
void make_page_executable(u64 addr, u64 *ttbr_reg);

// #define DEBUG

#ifdef DEBUG
#include "utils.h"
#define vt_dprintf printf
#else
#define vt_dprintf(...)                                                                               \
    do {                                                                                           \
    } while (0)
#endif

#define phy2virt(addr) ((u64)addr - cur_boot_args.phys_base + cur_boot_args.virt_base);

#define VADDR_L4_OFFSET_BITS 2
#define VADDR_L3_OFFSET_BITS 14
#define VADDR_L2_OFFSET_BITS 25
#define VADDR_L1_OFFSET_BITS 36

#define L1_IS_TABLE(pte) ((pte) && FIELD_GET(PTE_TYPE, pte) == PTE_TABLE)
#define L2_IS_TABLE(pte) ((pte) && FIELD_GET(PTE_TYPE, pte) == PTE_TABLE)

#define PTE_TARGET_MASK GENMASK(49, VADDR_L3_OFFSET_BITS)

#define VADDR_L3_INDEX_BITS 11
#define VADDR_L2_INDEX_BITS 11
#define VADDR_L1_INDEX_BITS 8

#define VADDR_L2_ALIGN_MASK GENMASK(VADDR_L2_OFFSET_BITS - 1, VADDR_L3_OFFSET_BITS)
#define VADDR_L3_ALIGN_MASK GENMASK(VADDR_L3_OFFSET_BITS - 1, VADDR_L4_OFFSET_BITS)
#define PTE_TARGET_MASK     GENMASK(49, VADDR_L3_OFFSET_BITS)
#define PTE_TARGET_MASK_L4  GENMASK(49, VADDR_L4_OFFSET_BITS)

#define PTE_LOWER_ATTRIBUTES GENMASK(13, 2)

#define INSN_MSR_OP0       GENMASK(20, 19)
#define INSN_MSR_OP0_SHIFT 19
#define INSN_MSR_OP1       GENMASK(18, 16)
#define INSN_MSR_OP1_SHIFT 16
#define INSN_MSR_CRn       GENMASK(15, 12)
#define INSN_MSR_CRn_SHIFT 12
#define INSN_MSR_CRm       GENMASK(11, 8)
#define INSN_MSR_CRm_SHIFT 8
#define INSN_MSR_OP2       GENMASK(7, 5)
#define INSN_MSR_OP2_SHIFT 5
#define INSN_MSR_Rt        GENMASK(4, 0)
#define INSN_MSR_Rt_SHIFT  0

#define _SYSREG_MSR(_1, _2, op0, op1, CRn, CRm, op2)                                               \
    (((op0) << INSN_MSR_OP0_SHIFT) | ((op1) << INSN_MSR_OP1_SHIFT) |                               \
     ((CRn) << INSN_MSR_CRn_SHIFT) | ((CRm) << INSN_MSR_CRm_SHIFT) |                               \
     ((op2) << INSN_MSR_OP2_SHIFT))
#define SYSREG_MSR(...) _SYSREG_MSR(__VA_ARGS__)
#define SYSREG_MSK      0x1fffff

#define SYSREG_VBAR_EL1  sys_reg(3, 0, 12, 0, 0)
#define SYSREG_TTBR0_EL1 sys_reg(3, 0, 2, 0, 0)
#define SYSREG_TTBR1_EL1 sys_reg(3, 0, 2, 0, 1)
#define SYSREG_TPIDR_EL1 sys_reg(3, 0, 13, 0, 4)
#define SYSREG_TCR_EL1   sys_reg(3, 0, 2, 0, 2)

#define __msr(reg, val)                                                                            \
    ({                                                                                             \
        u64 __val = (u64)val;                                                                      \
        __asm__ volatile("msr\t" #reg ", %0" : : "r"(__val));                                      \
    })
#define _msr(reg, val) __msr(reg, val)
#define sr_tkn(...)    _concat(_sr_tkn, __VA_ARGS__, )(__VA_ARGS__)

#define SYSREG_PASS(regv, sr)                                                                      \
    case SYSREG_MSR(sr):                                                                           \
        printf("[=] write 0x%lx to %s\n", regv, #sr);                                              \
        _msr(sr_tkn(sr), regv);                                                                    \
        break;

u64 xnu_vbar_el1 = 0;
bool xnu_sync_msr(u64 *regs)
{
    // TBD: check exception level and stuff
    u32 insn;
    // u64 spsr = mrs(SPSR_EL1);
    // u64 esr = mrs(ESR_EL1);
    u64 elr = mrs(ELR_EL1);
    insn = read32(elr);
    //    vt_dprintf("xnu_sync at 0x%lx = %x\n", elr, insn);
    u64 reg_value = regs[insn & INSN_MSR_Rt];
    switch (insn & SYSREG_MSK) {
        SYSREG_PASS(reg_value, SYSREG_TCR_EL1);
        // SYSREG_PASS(reg_value, SYSREG_TPIDR_EL1);
        case SYSREG_MSR(SYSREG_TPIDR_EL1):
            printf("[=] write 0x%lx to %s\n", reg_value, "SYSREG_TPIDR_EL1");
            msr(TPIDR_EL1, reg_value);
            break;
        // need do more stuff than logging when meet SYSREG_VBAR_EL1
        case SYSREG_MSR(SYSREG_VBAR_EL1):
            vt_dprintf("[!] write 0x%lx to %s\n", reg_value, "SYSREG_VBAR_EL1");
            if ((reg_value & 0xff00000000000000) == 0xff00000000000000) {
                // set vbar_el1 to virt addr of our handler
                _vt_vectors_start_va = (u32 *)phy2virt(_vt_vectors_start);
                msr(VBAR_EL1, _vt_vectors_start_va);
            } else
                vt_dprintf("just ignore it!\n");

            // u64 *real_vbar_handler_l3 = vt_pt_getl3(reg_value, (u64*)mrs(TTBR1_EL1));
            // u64 real_l3_property = *real_vbar_handler_l3 & ~GENMASK(47, 12);
            // u64 *my_vbar_handler_l3 = vt_pt_getl3((u64)_vt_vectors_start_va,
            // (u64*)mrs(TTBR1_EL1)); write64((u64)my_vbar_handler_l3, (*my_vbar_handler_l3 &
            // GENMASK(47, 12)) | real_l3_property); vt_pt_walk((u64)_vt_vectors_start_va,
            // (u64*)mrs(TTBR1_EL1));

            // msr(VBAR_EL1, reg_value);
            // PLAN: patch the l3 pte of real vbar_handler; but we can't reach real vbar_handler any
            // more!!!
            // u64 *vbar_handler_l3 = vt_pt_getl3(reg_value, (u64*)mrs(TTBR1_EL1));
            // u64 paddr_vbar_handler = *vbar_handler_l3 & GENMASK(47, 12);
            // vt_dprintf("paddr at 0x%lx; ours at %p\n", paddr_vbar_handler, _vt_vectors_start);
            // write64((u64)vbar_handler_l3, (*vbar_handler_l3 & ~GENMASK(47, 12)) |
            // (u64)_vt_vectors_start); vt_pt_walk(reg_value, (u64*)mrs(TTBR1_EL1)); msr(VBAR_EL1,
            // reg_value);

            // u64 ttbr0 = mrs(TTBR0_EL1);
            // u64 tcr = mrs(TCR_EL1);
            // vt_dprintf("vt_vectors_start at %p; ttbr0 is at 0x%lx\n", _vt_vectors_start, ttbr0);
            // xnu_vbar_el1 = reg_value;
            // for (int i=0; i < 16; i++ ){
            //     if(_vt_vectors_start[i*0x20] == 0x14000000) {
            //         _vt_vectors_start[i*0x20] = 0x17c95600;
            //         vt_dprintf("set redirector at %p\n", &_vt_vectors_start[i*0x20]);
            //     }
            //     if(_vt_vectors_start[i*0x20+1] == 0x14000000) {
            //         _vt_vectors_start[i*0x20+1] = 0x17c955ff;
            //         vt_dprintf("set redirector at %p\n", &_vt_vectors_start[i*0x20+1]);
            //     }
            //     msr(VBAR_EL1, _vt_vectors_start);
            // }

            // udelay(-1);
            vt_dprintf("current vbar_el1 = 0x%lx\n", mrs(VBAR_EL1));
            break;
        case SYSREG_MSR(SYSREG_TTBR1_EL1):
            vt_dprintf("[!] write 0x%lx to %s\n", reg_value, "SYSREG_TTBR1_EL1");
            // vt_dprintf("do mmu walk to make sure our vbar handler is executable!\n");
            // make_page_executable((u64)_vt_vectors_start_va, (u64*)reg_value);
            // u64 _vt_double_panic_va = (u64)phy2virt(&_vt_double_panic);
            // make_page_executable(_vt_double_panic_va, (u64*)reg_value);
            // vt_pt_walk(xnu_vbar_el1, (u64*)reg_value);

            _vt_mmuinfo.xnu.ttbr0_el1 = mrs(TTBR0_EL1);
            _vt_mmuinfo.xnu.ttbr1_el1 = reg_value;
            _vt_mmuinfo.xnu.tcr_el1 = mrs(TCR_EL1);
            _vt_mmuinfo.xnu.mair_el1 = mrs(MAIR_EL1);
            // _vt_mmuinfo.sctlr_el1 = mrs(SCTLR_EL1);
            // vt_dprintf("writing xnu's tcr_el1=0x%lx mair_el1=0x%lx\nsctlr_el1=0x%lx
            // ttbr0_el1=0x%lx\n",
            //     regs[60], regs[59], mrs(SCTLR_EL1), regs[61]);
            // set ttbr1_base to new one
            // emulate write
            if (!regs[63]) // early
                regs[62] = reg_value;
            else { // later
                msr(TTBR1_EL1, reg_value);
                // fix privilege here!
                make_page_executable((u64)_vt_vectors_start_va, (u64 *)reg_value);
                u64 _vt_m1n1_call_va = (u64)phy2virt(&_vt_m1n1_call);
                make_page_executable(_vt_m1n1_call_va, (u64 *)reg_value);
                // udelay(-1);
            }
            // udelay(-1);
            break;
        default:
            return false; // not matched; call original sync handler
    }
    elr += 4;
    msr(ELR_EL1, elr);
    return true;
}

bool xnu_double_panic(u64 *regs)
{
    UNUSED(regs);
    printf("double panic in xnu; or m1n1 panic itself!\n");
    udelay(-1);
    return false;
}

u64 vt_pt_walk(u64 addr, u64 *ttbr_reg)
{
    vt_dprintf("vt_pt_walk(0x%lx)\n", addr);

    addr = addr & MASK(39);
    u64 idx = addr >> VADDR_L1_OFFSET_BITS;
    u64 *l2;

    u64 l1d = ttbr_reg[idx];

    vt_dprintf("  l1d = 0x%lx, at %p\n", l1d, &ttbr_reg[idx]);

    if (!L1_IS_TABLE(l1d)) {
        vt_dprintf("  result: 0x%lx\n", l1d);
        return l1d;
    }
    l2 = (u64 *)(l1d & PTE_TARGET_MASK);

    idx = (addr >> VADDR_L2_OFFSET_BITS) & MASK(VADDR_L2_INDEX_BITS);
    u64 l2d = l2[idx];
    vt_dprintf("  l2d = 0x%lx, at %p\n", l2d, &l2[idx]);

    if (!L2_IS_TABLE(l2d)) {

        l2d &= ~PTE_LOWER_ATTRIBUTES;
        l2d |= addr & (VADDR_L2_ALIGN_MASK | VADDR_L3_ALIGN_MASK);

        vt_dprintf("  result: 0x%lx\n", l2d);
        return l2d;
    }

    idx = (addr >> VADDR_L3_OFFSET_BITS) & MASK(VADDR_L3_INDEX_BITS);
    u64 l3d = ((u64 *)(l2d & PTE_TARGET_MASK))[idx];
    vt_dprintf("  l3d = 0x%lx\n", l3d);
    l3d &= ~PTE_LOWER_ATTRIBUTES;
    l3d |= addr & VADDR_L3_ALIGN_MASK;
    vt_dprintf("  result: 0x%lx\n", l3d);
    return l3d;
}

u64 *vt_pt_getl3(u64 addr, u64 *ttbr_reg)
{
    vt_dprintf("vt_pt_getl3(0x%lx)\n", addr);

    addr = addr & MASK(39);
    u64 idx = addr >> VADDR_L1_OFFSET_BITS;
    u64 *l2;

    u64 l1d = ttbr_reg[idx];

    vt_dprintf("  l1d = 0x%lx, at %p, idx=0x%lx\n", l1d, &ttbr_reg[idx], idx);

    if (!L1_IS_TABLE(l1d)) {
        vt_dprintf("  result: 0x%lx\n", l1d);
        return &ttbr_reg[idx];
    }
    l2 = (u64 *)(l1d & PTE_TARGET_MASK);

    idx = (addr >> VADDR_L2_OFFSET_BITS) & MASK(VADDR_L2_INDEX_BITS);
    u64 l2d = l2[idx];
    vt_dprintf("  l2d = 0x%lx, at %p, idx=0x%lx; l2=%p\n", l2d, &l2[idx], idx, l2);

    if (!L2_IS_TABLE(l2d)) {

        l2d &= ~PTE_LOWER_ATTRIBUTES;
        l2d |= addr & (VADDR_L2_ALIGN_MASK | VADDR_L3_ALIGN_MASK);

        vt_dprintf("  result: 0x%lx\n", l2d);
        return &l2[idx];
    }

    idx = (addr >> VADDR_L3_OFFSET_BITS) & MASK(VADDR_L3_INDEX_BITS);
    u64 l3d = ((u64 *)(l2d & PTE_TARGET_MASK))[idx];
    UNUSED(l3d); // while vt_dprintf is not enabled
    vt_dprintf("  l3d = 0x%lx\n", l3d);
    return &((u64 *)(l2d & PTE_TARGET_MASK))[idx];
}

void make_page_executable(u64 addr, u64 *ttbr_reg)
{
    u64 *l3_addr = vt_pt_getl3((u64)addr, (u64 *)ttbr_reg);
    u64 l3_entry = *l3_addr;
    vt_dprintf("[-] l3 entry = 0x%lx at %p\n", l3_entry, l3_addr);
    write64((u64)l3_addr, l3_entry & ~(BIT(53) | BIT(54)));
    // remove xn/pxn bits
    vt_dprintf("[+] l3 entry = 0x%lx at %p\n", *l3_addr, l3_addr);
}

#define INSN_SUB_RD GENMASK(4,0)
#define INSN_SUB_RN GENMASK(9,5)
#define INSN_SUB_IMM GENMASK(21,10)
#define INSN_SUB_SHIFT GENMASK(23,22)

bool xnu_sync_sub(u64 *regs)
{
    //target is sub sp,sp,#stackSize
    //support only one target, or set more bit to instruction/ check regs[pc]
    u64 elr = mrs(ELR_EL1);
    u32 insn = read32(elr);
    if ((insn & 0x3ff) != 0x3ff) {
        //not sub sp, sp, imm
        printf("not a entry of function?\n");
        udelay(-1);
    }
    if ((insn & INSN_SUB_SHIFT) != 0) {
        printf("check shift\n");
        udelay(-1);
    }
    u32 imm = (insn & INSN_SUB_IMM) >> 10;
    msr(SP_EL0, mrs(SP_EL0) - imm);
    if(regs[1] == 0x20e100000) {
        //AIC's register
        regs[3] = 0;   
    }
    printf("pmap_map_bd(0x%lx, 0x%lx, 0x%lx, 0x%lx)\n",
        regs[0], regs[1], regs[2], regs[3]);
    elr += 4;
    msr(ELR_EL1, elr);
    return true;
}

extern bool emulate_store(struct exc_info *ctx, u32 insn, u64 *val, u64 *width, u64 *vaddr);

bool xnu_sync_da(u64 *regs)
{
    u64 elr = mrs(ELR_EL1);
    // u64 far = mrs(FAR_EL1);
    // u32 insn = read32(elr);
    // u64 width;
    // u64 vaddr = far;
    // u8 val[64];
    // memset32(val, 0, sizeof(val));
    // emulate_store((struct exc_info *)regs, insn, (u64 *)val, &width, &vaddr);
    // printf("emulate_store ret width=0x%lx vaddr=0x%lx\n", width, far);
    // udelay(-1);
    u64 esr = mrs(ESR_EL1);
    u32 ec = FIELD_GET(ESR_ISS_DABORT_DFSC, esr);
    printf("DA %x at 0x%lx\n", ec, elr);
    return false;
}

bool xnu_dispatch(u64 *regs)
{
    u64 esr = mrs(ESR_EL1);
    u32 insn;
    u32 ec = FIELD_GET(ESR_EC, esr);

    switch (ec) {
        case ESR_EC_IABORT:
        case ESR_EC_IABORT_LOWER:
        case ESR_EC_UNKNOWN:
            insn = read32(mrs(ELR_EL1));
            if ((insn & 0xffe00000) == 0xffe00000)
                return xnu_sync_msr(regs);
            if ((insn & 0xff000000) == 0xfe000000)
                return xnu_sync_sub(regs);
            break;
    
        case ESR_EC_DABORT:
        case ESR_EC_DABORT_LOWER:
                return false;// not handle it now
                // return xnu_sync_da(regs);
            break;
        default:
            break;
    }
    printf("current ec = 0x%lx\n", esr & ESR_EC);
    if ((1 & mrs(SPSR_EL1)) == 1)
        return xnu_double_panic(regs);
    return false;
}
