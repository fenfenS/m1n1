/* SPDX-License-Identifier: MIT */

#include "../build/build_cfg.h"
#include "../build/build_tag.h"

#include "../config.h"

#include "adt.h"
#include "aic.h"
#include "cpufreq.h"
#include "display.h"
#include "exception.h"
#include "fb.h"
#include "firmware.h"
#include "gxf.h"
#include "heapblock.h"
#include "mcc.h"
#include "memory.h"
#include "nvme.h"
#include "payload.h"
#include "pcie.h"
#include "pmgr.h"
#include "sep.h"
#include "smp.h"
#include "string.h"
#include "uart.h"
#include "uartproxy.h"
#include "usb.h"
#include "utils.h"
#include "wdt.h"
#include "xnuboot.h"

extern void *g_xnu_entry;
extern void *g_bootargs;
// extern void _vt_xnu_init;
struct vt_mmu_info _vt_mmuinfo;

struct vector_args next_stage;

const char version_tag[] = "##m1n1_ver##" BUILD_TAG;
const char *const m1n1_version = version_tag + 12;

u32 board_id = ~0, chip_id = ~0;

void xnu_init(void);
void get_device_info(void)
{
    const char *model = (const char *)adt_getprop(adt, 0, "model", NULL);
    const char *target = (const char *)adt_getprop(adt, 0, "target-type", NULL);

    printf("Device info:\n");

    if (model)
        printf("  Model: %s\n", model);

    if (target)
        printf("  Target: %s\n", target);

    is_mac = !!strstr(model, "Mac");

    int chosen = adt_path_offset(adt, "/chosen");
    if (chosen > 0) {
        if (ADT_GETPROP(adt, chosen, "board-id", &board_id) < 0)
            printf("Failed to find board-id\n");
        if (ADT_GETPROP(adt, chosen, "chip-id", &chip_id) < 0)
            printf("Failed to find chip-id\n");

        printf("  Board-ID: 0x%x\n", board_id);
        printf("  Chip-ID: 0x%x\n", chip_id);
    } else {
        printf("No chosen node!\n");
    }

    printf("\n");
}

void run_actions(void)
{
    bool usb_up = false;

#ifndef BRINGUP
#ifdef EARLY_PROXY_TIMEOUT
    int node = adt_path_offset(adt, "/chosen/asmb");
    u64 lp_sip0 = 0;

    if (node >= 0) {
        ADT_GETPROP(adt, node, "lp-sip0", &lp_sip0);
        printf("Boot policy: sip0 = %ld\n", lp_sip0);
    }

    if (!cur_boot_args.video.display && lp_sip0 == 127) {
        printf("Bringing up USB for early debug...\n");

        usb_init();
        usb_iodev_init();

        usb_up = true;

        printf("Waiting for proxy connection... ");
        for (int i = 0; i < EARLY_PROXY_TIMEOUT * 100; i++) {
            for (int j = 0; j < USB_IODEV_COUNT; j++) {
                iodev_id_t iodev = IODEV_USB0 + j;

                if (!(iodev_get_usage(iodev) & USAGE_UARTPROXY))
                    continue;

                usb_iodev_vuart_setup(iodev);
                iodev_handle_events(iodev);
                if (iodev_can_write(iodev) || iodev_can_write(IODEV_USB_VUART)) {
                    printf(" Connected!\n");
                    uartproxy_run(NULL);
                    return;
                }
            }

            mdelay(10);
            if (i % 100 == 99)
                printf(".");
        }
        printf(" Timed out\n");
    }
#endif
#endif

    printf("Checking for payloads...\n");

    if (payload_run() == 0) {
        printf("Valid payload found\n");
        return;
    }
    fb_set_active(true);

    printf("No valid payload found\n");

#ifndef BRINGUP
    if (!usb_up) {
        usb_init();
        usb_iodev_init();
    }
#endif

    printf("Running proxy...\n");

    uartproxy_run(NULL);
}

void m1n1_main(void)
{
    printf("\n\nm1n1 %s\n", m1n1_version);
    printf("Copyright The Asahi Linux Contributors\n");
    printf("Licensed under the MIT license\n\n");

    printf("Running in EL%lu\n\n", mrs(CurrentEL) >> 2);

    firmware_init();

    heapblock_init();

#ifndef BRINGUP
    if (supports_gxf())
        gxf_init();
    mcc_init();
    mmu_init();
    aic_init();
#endif
    wdt_disable();
#ifndef BRINGUP
    pmgr_init();
#ifdef USE_FB
    display_init();
    // Kick DCP to sleep, so dodgy monitors which cause reconnect cycles don't cause us to lose the
    // framebuffer.
    display_shutdown(DCP_SLEEP_IF_EXTERNAL);
    // On idevice we need to always clear, because otherwise it looks scuffed on white devices
    fb_init(!is_mac);
    fb_display_logo();
#ifdef FB_SILENT_MODE
    fb_set_active(!cur_boot_args.video.display);
#else
    fb_set_active(true);
#endif
#endif

    cpufreq_fixup();
    sep_init();
#endif

    printf("Initialization complete.\n");

    if (g_xnu_entry != 0) {
        printf("xnu_entry from pongoOS at %p\n", g_xnu_entry);
        printf("g_bootargs = %p; cur_boot_args=%p\n", g_bootargs, &cur_boot_args);
        cur_boot_args.top_of_kernel_data += 0x800000; // 8MB
        next_stage.entry = g_xnu_entry;
        next_stage.args[0] = (u64)&cur_boot_args;
    } else
        run_actions();

    if (!next_stage.entry) {
        panic("Nothing to do!\n");
    }

    printf("ttbr0 = 0x%lx tcr = 0x%lx mair = 0x%lx\n", mrs(TTBR0_EL1), mrs(TCR_EL1), mrs(MAIR_EL1));
    // save offset and ttbr0 for switch
    _vt_mmuinfo.virt_phy_off = cur_boot_args.virt_base - cur_boot_args.phys_base;
    _vt_mmuinfo.m1n1.ttbr0_el1 = mrs(TTBR0_EL1);
    _vt_mmuinfo.m1n1.ttbr1_el1 = mrs(TTBR1_EL1);
    _vt_mmuinfo.m1n1.tcr_el1 = mrs(TCR_EL1);
    _vt_mmuinfo.m1n1.mair_el1 = mrs(MAIR_EL1);
    _vt_mmuinfo.sctlr_el1 = mrs(SCTLR_EL1);

    printf("Preparing to run next stage at %p...\n", next_stage.entry);

    nvme_shutdown();
    exception_shutdown();
#ifndef BRINGUP
    usb_iodev_shutdown();
    display_shutdown(DCP_SLEEP_IF_EXTERNAL);
#ifdef USE_FB
    fb_shutdown(next_stage.restore_logo);
#endif
    mmu_shutdown();
#endif

    printf("Vectoring to next stage...\n");
    
    xnu_init();

    next_stage.entry(next_stage.args[0], next_stage.args[1], next_stage.args[2], next_stage.args[3],
                     next_stage.args[4]);

    panic("Next stage returned!\n");
}

extern u32 _vt_vectors_start[];
extern void *iovbar_entry;

void xnu_init(void)
{
    printf("xnu_init before booting!\n");
    printf("_vt_vectors_start at %p\n", _vt_vectors_start);
    // redirect vbar
    for (int i = 0; i < 16; i++) {
        if (_vt_vectors_start[i * 0x20] == 0x14000000) {
            _vt_vectors_start[i * 0x20] = 0x17c93000;
            printf("set redirector at %p\n", &_vt_vectors_start[i * 0x20]);
        }
        if (_vt_vectors_start[i * 0x20 + 1] == 0x14000000) {
            _vt_vectors_start[i * 0x20 + 1] = 0x17c92fff;
            printf("set redirector at %p\n", &_vt_vectors_start[i * 0x20 + 1]);
        }
        if (_vt_vectors_start[i * 0x20 + 2] == 0x14000000) {
            _vt_vectors_start[i * 0x20 + 2] = 0x17c92ffe;
            printf("set redirector at %p\n", &_vt_vectors_start[i * 0x20 + 2]);
        }
        msr(VBAR_EL1, _vt_vectors_start);
    }
    if (read32((u64)g_xnu_entry + 0x6cbb20) == 0xd518c000) {
        write32((u64)g_xnu_entry + 0x6cbb20, 0xd518c000 | 0xffe00000);
        // make it undefined
        write32((u64)g_xnu_entry + 0x4010, 0xd503201f);
        // there's a check in kernel, bypass it
    } else
        printf("check msr vbar offset\n");

    if (read32((u64)g_xnu_entry + 0x6cbb18) == 0xd5182020) {
        write32((u64)g_xnu_entry + 0x6cbb18, 0xd5182020 | 0xffe00000);
        // make it undefined
        write32((u64)g_xnu_entry + 0x3fe8, 0xd503201f);
        // there's a check in kernel, bypass it
        printf("set ttbr1_el1 patched\n");
    } else
        printf("check msr ttbr offset\n");

    // if(read32((u64)g_xnu_entry+0x6cbb28)  == 0xd5182040) {
    //     write32((u64)g_xnu_entry+0x6cbb28, 0xd5182040|0xffe00000);
    //     //make it undefined
    //     write32((u64)g_xnu_entry+0x4038, 0xd503201f);
    //     //there's a check in kernel, bypass it
    //     printf("set tcr_el1 patched\n");
    // }
    // else printf("check msr tcr offset\n");

    write32((u64)g_xnu_entry + 0x15ae14, 0xd503201f);
    write32((u64)g_xnu_entry - 0x80, 0xd503201f);
    printf("patched ktrr\n");
    write64(0x202050000, (u64)&iovbar_entry | BIT(1));

    // if(read32((u64)g_xnu_entry+0x4448)  == 0xd518d080) {
    //   write32((u64)g_xnu_entry+0x4448, 0xd518d080|0xffe00000);
    //     //make it undefined
    //     printf("set one of msr TPIDR_EL1 patched\n");
    // }
    // else printf("check msr TPIDR_EL1 offset\n");

    // udelay(-1);
    write32((u64)g_xnu_entry - 0x6ab8, 0xf2aca332);
    printf("patched userspace's mapping\n");
    msr(VBAR_EL1, _vt_vectors_start);
    printf("------------------------------Patched XNU Booting------------------------------\n");
}
