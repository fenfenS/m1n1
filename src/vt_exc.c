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

#define SYSREG_PASS(regv, sr)                                                                      \
    case SYSREG_MSR(sr):                                                                           \
        printf("write 0x%lx to %s\n", regv, #sr);                                            \
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
    printf("xnu_sync at 0x%lx = %x\n", elr, insn);
    elr += 4;
    u64 reg_value = regs[insn & INSN_MSR_Rt];
    switch (insn & SYSREG_MSK) {
        // SYSREG_PASS(reg_value, SYSREG_VBAR_EL1);
        // need do more stuff than logging when meet SYSREG_VBAR_EL1
        case SYSREG_MSR(SYSREG_VBAR_EL1):
            printf("[!] write 0x%lx to %s\n", reg_value, "SYSREG_VBAR_EL1");
            u64 ttbr0 = mrs(TTBR0_EL1);
            u64 tcr = mrs(TCR_EL1);
            printf("vt_vectors_start at %p; ttbr0 is at 0x%lx\n", _vt_vectors_start, ttbr0);
            xnu_vbar_el1 = reg_value;
            for (int i=0; i < 16; i++ ){
                if(_vt_vectors_start[i*0x20] == 0x14000000) {
                    _vt_vectors_start[i*0x20] = 0x17c95600;
                    printf("set redirector at %p\n", &_vt_vectors_start[i*0x20]);
                }
                if(_vt_vectors_start[i*0x20+1] == 0x14000000) {
                    _vt_vectors_start[i*0x20+1] = 0x17c955ff;
                    printf("set redirector at %p\n", &_vt_vectors_start[i*0x20+1]);
                }
                msr(VBAR_EL1, _vt_vectors_start);
            }
            if (_vt_m1n1_mmu[0] == 0xd503201fd503201f)
                _vt_m1n1_mmu[0] = ttbr0;
            if (_vt_m1n1_mmu[1] == 0xd503201fd503201f)
                _vt_m1n1_mmu[1] = tcr; 
            break;
        case SYSREG_MSR(SYSREG_TTBR1_EL1):
            printf("[!] write 0x%lx to %s\n", reg_value, "SYSREG_TTBR1_EL1");
            printf("do mmu walk to make sure our vbar handler is executable!\n");
            make_page_executable((u64)_vt_vectors_start_va, (u64*)reg_value);
            u64 _vt_double_panic_va = (u64)phy2virt(&_vt_double_panic);
            make_page_executable(_vt_double_panic_va, (u64*)reg_value);
            //emulate write
            msr(TTBR1_EL1, reg_value);
            udelay(-1);
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
    if(read32((u64)g_xnu_entry+0x6cbb18)  == 0xd5182020) {
        write32((u64)g_xnu_entry+0x6cbb18, 0xd5182020|0xffe00000);
        //make it undefined
        write32((u64)g_xnu_entry+0x3fe8,   0xd503201f);
        //there's a check in kernel, bypass it
        printf("set ttbr1_el1 patched\n");
    }
    else printf("check msr ttbr offset\n");
    // udelay(-1);
    // return false;//normal return
    return true;
}

void xnu_double_panic(u64* regs) {
    UNUSED(regs);
    printf("double panic in xnu; or m1n1 panic itself!\n");
    udelay(-1);
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
    printf("vt_pt_walk(0x%lx)\n", addr);

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