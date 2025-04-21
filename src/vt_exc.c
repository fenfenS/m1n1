#include "exception.h"
#include "aic.h"
#include "aic_regs.h"
#include "cpu_regs.h"
#include "gxf.h"
#include "iodev.h"
#include "memory.h"
#include "uart.h"
#include "utils.h"
#include "xnuboot.h"

extern u32 _vt_vectors_start[];
extern u64 _vt_m1n1_mmu[];
extern void *g_xnu_entry;
extern u64 _vt_double_panic;
extern void *iovbar_entry;
extern struct vt_mmu_info _vt_mmuinfo;

bool vbar_set=false;
u32 *_vt_vectors_start_va;
u64 vt_pt_walk(u64 addr, u64* ttbr_reg);
u64* vt_pt_getl3(u64 addr, u64* ttbr_reg);
void make_page_executable(u64 addr, u64* ttbr_reg);

#define phy2virt(addr) ((u64)addr - cur_boot_args.phys_base + cur_boot_args.virt_base);

#define VADDR_L4_OFFSET_BITS 2
#define VADDR_L3_OFFSET_BITS 14
#define VADDR_L2_OFFSET_BITS 25
#define VADDR_L1_OFFSET_BITS 36

#define L1_IS_TABLE(pte) ((pte) && FIELD_GET(PTE_TYPE, pte) == PTE_TABLE)
#define L2_IS_TABLE(pte)     ((pte) && FIELD_GET(PTE_TYPE, pte) == PTE_TABLE)

#define PTE_TARGET_MASK     GENMASK(49, VADDR_L3_OFFSET_BITS)

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

#define __msr(reg, val)                                                                            \
    ({                                                                                             \
        u64 __val = (u64)val;                                                                      \
        __asm__ volatile("msr\t" #reg ", %0" : : "r"(__val));                                      \
    })
#define _msr(reg, val) __msr(reg, val)
#define sr_tkn(...) _concat(_sr_tkn, __VA_ARGS__, )(__VA_ARGS__)

#define SYSREG_PASS(regv, sr)                                                                      \
    case SYSREG_MSR(sr):                                                                           \
        printf("[=] write 0x%lx to %s\n", regv, #sr);                                            \
        _msr(sr_tkn(sr), regv);                                                                      \
        break;


u64 xnu_vbar_el1 = 0;
bool xnu_sync(u64 *regs)
{
    // TBD: check exception level and stuff
    u32 insn;
    // u64 spsr = mrs(SPSR_EL1);
    // u64 esr = mrs(ESR_EL1);
    u64 elr = mrs(ELR_EL1);
    insn = read32(elr);
//    printf("xnu_sync at 0x%lx = %x\n", elr, insn);
    elr += 4;
    u64 reg_value = regs[insn & INSN_MSR_Rt];
    switch (insn & SYSREG_MSK) {
        // SYSREG_PASS(reg_value, SYSREG_TPIDR_EL1);
        case SYSREG_MSR(SYSREG_TPIDR_EL1):
            printf("[=] write 0x%lx to %s\n", reg_value, "SYSREG_TPIDR_EL1");
            msr(TPIDR_EL1, reg_value);
            break;
        // need do more stuff than logging when meet SYSREG_VBAR_EL1
        case SYSREG_MSR(SYSREG_VBAR_EL1):
            printf("[!] write 0x%lx to %s\n", reg_value, "SYSREG_VBAR_EL1");
            printf("just ignore it!\n");

            // u64 *real_vbar_handler_l3 = vt_pt_getl3(reg_value, (u64*)mrs(TTBR1_EL1));
            // u64 real_l3_property = *real_vbar_handler_l3 & ~GENMASK(47, 12);
            // u64 *my_vbar_handler_l3 = vt_pt_getl3((u64)_vt_vectors_start_va, (u64*)mrs(TTBR1_EL1));
            // write64((u64)my_vbar_handler_l3, (*my_vbar_handler_l3 & GENMASK(47, 12)) | real_l3_property);
            // vt_pt_walk((u64)_vt_vectors_start_va, (u64*)mrs(TTBR1_EL1));

            // msr(VBAR_EL1, reg_value);
            //PLAN: patch the l3 pte of real vbar_handler; but we can't reach real vbar_handler any more!!!
            // u64 *vbar_handler_l3 = vt_pt_getl3(reg_value, (u64*)mrs(TTBR1_EL1));
            // u64 paddr_vbar_handler = *vbar_handler_l3 & GENMASK(47, 12);
            // printf("paddr at 0x%lx; ours at %p\n", paddr_vbar_handler, _vt_vectors_start);
            // write64((u64)vbar_handler_l3, (*vbar_handler_l3 & ~GENMASK(47, 12)) | (u64)_vt_vectors_start);
            // vt_pt_walk(reg_value, (u64*)mrs(TTBR1_EL1));
            // msr(VBAR_EL1, reg_value);



            // u64 ttbr0 = mrs(TTBR0_EL1);
            // u64 tcr = mrs(TCR_EL1);
            // printf("vt_vectors_start at %p; ttbr0 is at 0x%lx\n", _vt_vectors_start, ttbr0);
            // xnu_vbar_el1 = reg_value;
            // for (int i=0; i < 16; i++ ){
            //     if(_vt_vectors_start[i*0x20] == 0x14000000) {
            //         _vt_vectors_start[i*0x20] = 0x17c95600;
            //         printf("set redirector at %p\n", &_vt_vectors_start[i*0x20]);
            //     }
            //     if(_vt_vectors_start[i*0x20+1] == 0x14000000) {
            //         _vt_vectors_start[i*0x20+1] = 0x17c955ff;
            //         printf("set redirector at %p\n", &_vt_vectors_start[i*0x20+1]);
            //     }
            //     msr(VBAR_EL1, _vt_vectors_start);
            // }

            // udelay(-1);
            printf("current vbar_el1 = 0x%lx\n", mrs(VBAR_EL1));
            break;
        case SYSREG_MSR(SYSREG_TTBR1_EL1):
            printf("[!] write 0x%lx to %s\n", reg_value, "SYSREG_TTBR1_EL1");
            printf("do mmu walk to make sure our vbar handler is executable!\n");
            make_page_executable((u64)_vt_vectors_start_va, (u64*)reg_value);
            u64 _vt_double_panic_va = (u64)phy2virt(&_vt_double_panic);
            make_page_executable(_vt_double_panic_va, (u64*)reg_value);
            // vt_pt_walk(xnu_vbar_el1, (u64*)reg_value);

            _vt_mmuinfo.xnu.ttbr0_el1 = mrs(TTBR0_EL1);
            _vt_mmuinfo.xnu.ttbr1_el1 = reg_value;
            _vt_mmuinfo.xnu.tcr_el1   = mrs(TCR_EL1);
            _vt_mmuinfo.xnu.mair_el1  = mrs(MAIR_EL1);
            _vt_mmuinfo.sctlr_el1 = mrs(SCTLR_EL1);
            // printf("writing xnu's tcr_el1=0x%lx mair_el1=0x%lx\nsctlr_el1=0x%lx ttbr0_el1=0x%lx\n",
            //     regs[60], regs[59], mrs(SCTLR_EL1), regs[61]);
            //set ttbr1_base to new one
            //emulate write
            msr(TTBR1_EL1, reg_value);
            // udelay(-1);
            break;
        default:
            return false;// not matched; call original sync handler
    }
    msr(ELR_EL1, elr);
    return true;
}

// void vt_exc_sync(u64 *regs) {
//     u64 sctlr_el1 = mrs(SCTLR_EL1);
//     mmu_pt_L0
// }

bool xnu_init(u64 *regs)
{
    if(vbar_set){
        udelay(-1);
    }
    vbar_set = true;
    UNUSED(regs);
    xnu_vbar_el1 = mrs(SYSREG_VBAR_EL1);

    printf("m1n1 after xnu_init!\n");
    // printf("_vt_vectors_start at %p, virt_base=0x%lx\n", _vt_vectors_start, cur_boot_args.virt_base);
    // printf("offset=0x%lx\n", (u64)_vt_vectors_start - cur_boot_args.phys_base);
    _vt_vectors_start_va = (u32*)phy2virt(_vt_vectors_start);
    printf("_vt_vectors_start_va at %p, current vbar=0x%lx\n", _vt_vectors_start_va, xnu_vbar_el1);
    //redirect vbar
    for (int i=0; i < 16; i++ ){
        if(_vt_vectors_start_va[i*0x20] == 0x14000000) {
            _vt_vectors_start_va[i*0x20] = 0x17c93000;
            printf("set redirector at %p\n", &_vt_vectors_start_va[i*0x20]);
        }
        if(_vt_vectors_start_va[i*0x20+1] == 0x14000000) {
            _vt_vectors_start_va[i*0x20+1] = 0x17c92fff;
            printf("set redirector at %p\n", &_vt_vectors_start_va[i*0x20+1]);
        }
        msr(VBAR_EL1, _vt_vectors_start_va);
    }
    if(read32((u64)g_xnu_entry+0x6cbb20)  == 0xd518c000) {
        write32((u64)g_xnu_entry+0x6cbb20, 0xd518c000|0xffe00000);
        //make it undefined
        write32((u64)g_xnu_entry+0x4010,   0xd503201f);
        //there's a check in kernel, bypass it
    }
    else printf("check msr vbar offset\n");
    
    if(read32((u64)g_xnu_entry+0x6cbb18)  == 0xd5182020) {
        write32((u64)g_xnu_entry+0x6cbb18, 0xd5182020|0xffe00000);
        //make it undefined
        write32((u64)g_xnu_entry+0x3fe8,   0xd503201f);
        //there's a check in kernel, bypass it
        printf("set ttbr1_el1 patched\n");
    }
    else printf("check msr ttbr offset\n");
    if(read32((u64)g_xnu_entry+0x6cbb18)  == 0xd5182020) {
        write32((u64)g_xnu_entry+0x6cbb18, 0xd5182020|0xffe00000);
        //make it undefined
        write32((u64)g_xnu_entry+0x3fe8,   0xd503201f);
        //there's a check in kernel, bypass it
        printf("set tcr_el1 patched\n");
    }
    else printf("check msr tcr offset\n");

    write32((u64)g_xnu_entry+0x15ae14, 0xd503201f);
    write32((u64)g_xnu_entry-0x80, 0xd503201f);
    printf("patched ktrr\n");
    write64(0x202050000, (u64)&iovbar_entry | BIT(1));

//     if(read32((u64)g_xnu_entry+0x4448)  == 0xd518d080) {
//       write32((u64)g_xnu_entry+0x4448, 0xd518d080|0xffe00000);
//         //make it undefined
//         printf("set one of msr TPIDR_EL1 patched\n");
//     }
//     else printf("check msr TPIDR_EL1 offset\n");
    // udelay(-1);
    write32((u64)g_xnu_entry-0x6ab8, 0xf2aca332);
    printf("patched userspace's mapping\n");

    return true;//has took over err
}

bool xnu_double_panic(u64* regs) {
    if (regs[59] != 0) {
        printf("double panic in xnu; or m1n1 panic itself!\n");
        udelay(-1);
    }
    return xnu_sync(regs);
    //from reset, which is also sp1
}

u64 vt_pt_walk(u64 addr, u64* ttbr_reg)
{
    printf("vt_pt_walk(0x%lx)\n", addr);

    addr = addr & MASK(39);
    u64 idx = addr >> VADDR_L1_OFFSET_BITS;
    u64 *l2;

    u64 l1d = ttbr_reg[idx];

    printf("  l1d = 0x%lx, at %p\n", l1d, &ttbr_reg[idx]);

    if (!L1_IS_TABLE(l1d)) {
        printf("  result: 0x%lx\n", l1d);
        return l1d;
    }
    l2 = (u64 *)(l1d & PTE_TARGET_MASK);

    idx = (addr >> VADDR_L2_OFFSET_BITS) & MASK(VADDR_L2_INDEX_BITS);
    u64 l2d = l2[idx];
    printf("  l2d = 0x%lx, at %p\n", l2d, &l2[idx]);

    if (!L2_IS_TABLE(l2d)) {

        l2d &= ~PTE_LOWER_ATTRIBUTES;
        l2d |= addr & (VADDR_L2_ALIGN_MASK | VADDR_L3_ALIGN_MASK);

        printf("  result: 0x%lx\n", l2d);
        return l2d;
    }

    idx = (addr >> VADDR_L3_OFFSET_BITS) & MASK(VADDR_L3_INDEX_BITS);
    u64 l3d = ((u64 *)(l2d & PTE_TARGET_MASK))[idx];
    printf("  l3d = 0x%lx\n", l3d);
    l3d &= ~PTE_LOWER_ATTRIBUTES;
    l3d |= addr & VADDR_L3_ALIGN_MASK;
    printf("  result: 0x%lx\n", l3d);
    return l3d;
}

u64* vt_pt_getl3(u64 addr, u64* ttbr_reg)
{
    printf("vt_pt_getl3(0x%lx)\n", addr);

    addr = addr & MASK(39);
    u64 idx = addr >> VADDR_L1_OFFSET_BITS;
    u64 *l2;

    u64 l1d = ttbr_reg[idx];

    printf("  l1d = 0x%lx, at %p, idx=0x%lx\n", l1d, &ttbr_reg[idx], idx);

    if (!L1_IS_TABLE(l1d)) {
        printf("  result: 0x%lx\n", l1d);
        return &ttbr_reg[idx];
    }
    l2 = (u64 *)(l1d & PTE_TARGET_MASK);

    idx = (addr >> VADDR_L2_OFFSET_BITS) & MASK(VADDR_L2_INDEX_BITS);
    u64 l2d = l2[idx];
    printf("  l2d = 0x%lx, at %p, idx=0x%lx; l2=%p\n", l2d, &l2[idx], idx, l2);

    if (!L2_IS_TABLE(l2d)) {

        l2d &= ~PTE_LOWER_ATTRIBUTES;
        l2d |= addr & (VADDR_L2_ALIGN_MASK | VADDR_L3_ALIGN_MASK);

        printf("  result: 0x%lx\n", l2d);
        return &l2[idx];
    }

    idx = (addr >> VADDR_L3_OFFSET_BITS) & MASK(VADDR_L3_INDEX_BITS);
    u64 l3d = ((u64 *)(l2d & PTE_TARGET_MASK))[idx];
    printf("  l3d = 0x%lx\n", l3d);
    return &((u64 *)(l2d & PTE_TARGET_MASK))[idx];
}

void make_page_executable(u64 addr, u64* ttbr_reg){
    u64* l3_addr = vt_pt_getl3((u64)addr, (u64*)ttbr_reg);
    u64 l3_entry = *l3_addr;
    printf("[=] l3 entry = 0x%lx at %p\n", l3_entry, l3_addr);
    write64((u64)l3_addr, l3_entry & ~( BIT(53) | BIT(54) ));
    //remove xn/pxn bits
    printf("[+] l3 entry = 0x%lx at %p\n", *l3_addr, l3_addr);
}