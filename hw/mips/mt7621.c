/*
 * QEMU MT7621 SoC Machine emulation
 *
 * MT7621 is a MIPS 1004Kc-based SoC from MediaTek, widely used in
 * routers (Xiaomi Router 3G, Newifi D2, etc.).
 *
 * Hardware overview:
 *   CPU:    MIPS 1004Kc (dual VPE, MIPS32 Release 2, 880MHz)
 *   GIC:    0x1FBC0000 (via CPS)
 *   CPC:    0x1FBF0000 (via CPS)
 *   GCR:    0x1FBF8000 (default, via CPS)
 *   SYSCTL: 0x1E000000
 *   UART0:  0x1E000C00 (ns16550a compatible)
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/qdev-clock.h"
#include "hw/mips/cps.h"
#include "hw/mips/bootloader.h"
#include "hw/char/serial-mm.h"
#include "hw/core/loader.h"
#include "elf.h"
#include "system/address-spaces.h"
#include "system/device_tree.h"
#include "system/reset.h"
#include "system/system.h"
#include "system/qtest.h"
#include "system/runstate-action.h"
#include "hw/misc/mt7621_sysctl.h"
#include "hw/misc/mt7621_timer.h"
#include "hw/misc/mt7621_memctrl.h"
#include "hw/gpio/mt7621_gpio.h"
#include "hw/ssi/mt7621_spi.h"
#include "hw/net/mt7621_eth.h"
#include "hw/misc/unimp.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "system/block-backend-io.h"

#include <libfdt.h>

#include "hw/mips/mips_tcg_fixup.h"

/* ------------------------------------------------------------------ */
/* FDT interrupt macros (match <dt-bindings/interrupt-controller/mips-gic.h>) */
#define FDT_GIC_SHARED     0
#define FDT_GIC_LOCAL      1
#define FDT_IRQ_TYPE_NONE          0
#define FDT_IRQ_TYPE_LEVEL_HIGH    4

/* ------------------------------------------------------------------ */
/* Memory map */
#define MT7621_DRAM_BASE        0x00000000ULL
#define MT7621_SYSCTL_BASE      0x1E000000ULL
#define MT7621_TIMER_BASE       0x1E000100ULL
#define MT7621_GPIO_BASE        0x1E000600ULL
#define MT7621_I2C_BASE         0x1E000900ULL
#define MT7621_I2S_BASE         0x1E000A00ULL
#define MT7621_SPI_BASE         0x1E000B00ULL
#define MT7621_UART0_BASE       0x1E000C00ULL
#define MT7621_UART1_BASE       0x1E000D00ULL
#define MT7621_UART2_BASE       0x1E000E00ULL
#define MT7621_GDMA_BASE        0x1E002800ULL
#define MT7621_NFI_BASE         0x1E003000ULL
#define MT7621_MEMCTRL_BASE     0x1E005000ULL
#define MT7621_ETH_BASE         0x1E100000ULL
#define MT7621_SDXC_BASE        0x1E130000ULL
#define MT7621_PCIE_BASE        0x1E140000ULL
#define MT7621_USB_BASE         0x1E1C0000ULL
#define MT7621_FLASH_BASE       0x1FC00000ULL
#define MT7621_DRAM_REMAP_BASE  0x20000000ULL

/*
 * MT7621 DRAM address space layout (from Linux kernel):
 *   LOWMEM:  0x00000000 - 0x1BFFFFFF  (max 448 MB contiguous DRAM)
 *   GAP:     0x1C000000 - 0x1FFFFFFF  (64 MB: peripherals, flash, CPC, GIC)
 *   HIGHMEM: 0x20000000 - 0x23FFFFFF  (64 MB remapped DRAM)
 * Total max DRAM = 448 + 64 = 512 MB
 */
#define MT7621_LOWMEM_MAX_SIZE  0x1C000000ULL
#define MT7621_HIGHMEM_BASE     0x20000000ULL
#define MT7621_HIGHMEM_SIZE     0x04000000ULL   /* 64 MB */

#define MT7621_FLASH_SIZE       (4 * MiB)       /* default/min flash size */
#define MT7621_FLASH_MAX_SIZE   (32 * MiB)      /* max SPI NOR flash size */
#define MT7621_IOMEM_SIZE       0x200000ULL     /* 0x1E000000 region */
#define MT7621_MAX_DRAM_MAP     0x20000000ULL   /* 512 MB: max DRAM address space */

/* Kernel offset in flash (where Breed expects the kernel image) */
#define MT7621_KERNEL_FLASH_OFFSET  0x50000


/* ------------------------------------------------------------------ */
/* GCR register offsets for bootloader */
#define GCR_BASE_ADDR           0x1fbf8000ULL
#define GCR_GIC_BASE_OFS        0x0080
#define GCR_CPC_BASE_OFS        0x0088
#define GCR_GIC_BASE_GICEN_MSK  1

/* ------------------------------------------------------------------ */
/* Machine state */
struct MT7621State {
    /*< private >*/
    MachineState parent_obj;

    /* CPS (includes CPU, GIC, CPC, GCR, ITU) */
    MIPSCPSState cps;

    /* Clocks */
    Clock *cpu_clk;
    Clock *periph_clk;

    /* Memory regions */
    MemoryRegion dram;        /* LOWMEM alias (0x0, ≤448MB, for >448MB RAM) */
    MemoryRegion dram_remap;  /* HIGHMEM alias (0x20000000, 64MB, for >448MB RAM) */
    MemoryRegion flash;       /* SPI flash (RAM-backed, read/write) */
    MemoryRegion iomem;       /* 0x1E000000 peripheral region */
    MemoryRegion dram_alias;  /* DRAM alias: wraps addresses beyond RAM size */

    /* SPI Flash backing */
    hwaddr flash_size;       /* Actual flash size (auto-detected from pflash) */
    uint8_t *flash_data;     /* Flash RAM buffer (shared with SPI controller) */
    BlockBackend *flash_blk; /* BlockBackend for pflash drive persistence */
    Notifier flash_save_notifier; /* Exit notifier to persist flash data */

    /* Flash write control: default read-only, enable via machine property */
    bool flash_writable;

    /* Debug: file to save RAM on exit */
    char *debug_ram_path;

    /* CPS sub-device aliases in system_memory */
    MemoryRegion gcr_alias;
    MemoryRegion gic_alias;
    MemoryRegion cpc_alias;

    /* FDT base address (kseg0 virtual) */
    hwaddr fdt_base;

    /* Devices */
    SerialMM *uart0;
};

typedef struct MT7621State MT7621State;

#define TYPE_MT7621_MACHINE   MACHINE_TYPE_NAME("mt7621")
DECLARE_INSTANCE_CHECKER(MT7621State, MT7621_MACHINE, TYPE_MT7621_MACHINE)

/* ------------------------------------------------------------------ */
/* GIC interrupt routing (GIC shared IRQ → peripheral) */
#define MT7621_GIC_IRQ_ETH     3
#define MT7621_GIC_IRQ_PCIE0   4
#define MT7621_GIC_IRQ_GPIO    12
#define MT7621_GIC_IRQ_GDMA    13
#define MT7621_GIC_IRQ_I2S     16
#define MT7621_GIC_IRQ_SDXC    20
#define MT7621_GIC_IRQ_USB     22
#define MT7621_GIC_IRQ_GSW     23
#define MT7621_GIC_IRQ_PCIE1   24
#define MT7621_GIC_IRQ_PCIE2   25
#define MT7621_GIC_IRQ_UART0   26
#define MT7621_GIC_IRQ_UART1   27
#define MT7621_GIC_IRQ_UART2   28

/* ------------------------------------------------------------------ */
/* FDT generation */
static const void *mt7621_create_fdt(MT7621State *s, int *dt_size)
{
    void *fdt;
    MachineState *ms = MACHINE(s);
    uint32_t gic_ph;
    char *name;
    unsigned int ram_low_sz;

    fdt = create_device_tree(dt_size);
    if (!fdt) {
        error_report("mt7621: failed to create device tree");
        exit(1);
    }

    /* Root node */
    qemu_fdt_setprop_string(fdt, "/", "model", "mediatek,mt7621");
    qemu_fdt_setprop_string(fdt, "/", "compatible", "mediatek,mt7621");
    qemu_fdt_setprop_cell(fdt, "/", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/", "#size-cells", 1);

    /* /cpus */
    qemu_fdt_add_subnode(fdt, "/cpus");
    qemu_fdt_setprop_cell(fdt, "/cpus", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#size-cells", 0);
    for (int cpu = 0; cpu < ms->smp.cpus; cpu++) {
        name = g_strdup_printf("/cpus/cpu@%d", cpu);
        qemu_fdt_add_subnode(fdt, name);
        qemu_fdt_setprop_string(fdt, name, "compatible", "mips,mips1004Kc");
        qemu_fdt_setprop_string(fdt, name, "device_type", "cpu");
        qemu_fdt_setprop_cell(fdt, name, "reg", cpu);
        g_free(name);
    }

    /* /soc (simple-bus for all MMIO peripherals) */
    qemu_fdt_add_subnode(fdt, "/soc");
    qemu_fdt_setprop_string(fdt, "/soc", "compatible", "simple-bus");
    qemu_fdt_setprop(fdt, "/soc", "ranges", NULL, 0);
    qemu_fdt_setprop_cell(fdt, "/soc", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/soc", "#size-cells", 1);

    /*
     * GIC: compatible "mti,gic" at 0x1fbc0000.
     *
     * This is the MIPS Global Interrupt Controller. The MMIO region is
     * 0x2000 bytes. #interrupt-cells = 3 (type, number, flags).
     * mti,reserved-cpu-vectors = 7 (GIC_NUM_LOCAL_INTRS).
     */
    gic_ph = qemu_fdt_alloc_phandle(fdt);
    qemu_fdt_add_subnode(fdt, "/soc/interrupt-controller@1fbc0000");
    qemu_fdt_setprop_string(fdt, "/soc/interrupt-controller@1fbc0000",
                            "compatible", "mti,gic");
    qemu_fdt_setprop_cells(fdt, "/soc/interrupt-controller@1fbc0000",
                           "reg", 0x1fbc0000, 0x2000);
    qemu_fdt_setprop(fdt, "/soc/interrupt-controller@1fbc0000",
                     "interrupt-controller", NULL, 0);
    qemu_fdt_setprop_cell(fdt, "/soc/interrupt-controller@1fbc0000",
                          "#interrupt-cells", 3);
    qemu_fdt_setprop_cell(fdt, "/soc/interrupt-controller@1fbc0000",
                          "phandle", gic_ph);
    qemu_fdt_setprop_cell(fdt, "/soc/interrupt-controller@1fbc0000",
                          "mti,reserved-cpu-vectors", 7);

    /* GIC Timer child node */
    qemu_fdt_add_subnode(fdt, "/soc/interrupt-controller@1fbc0000/timer");
    qemu_fdt_setprop_string(fdt, "/soc/interrupt-controller@1fbc0000/timer",
                            "compatible", "mti,gic-timer");
    qemu_fdt_setprop_cells(fdt, "/soc/interrupt-controller@1fbc0000/timer",
                           "interrupts", FDT_GIC_LOCAL, 1, FDT_IRQ_TYPE_NONE);

    /* CDMM (Common Device Memory Map) */
    qemu_fdt_add_subnode(fdt, "/soc/cdmm@1fbd0000");
    qemu_fdt_setprop_string(fdt, "/soc/cdmm@1fbd0000",
                            "compatible", "mti,mips-cdmm");
    qemu_fdt_setprop_cells(fdt, "/soc/cdmm@1fbd0000",
                           "reg", 0x1fbd0000, 0x8000);

    /* CPC (Cluster Power Controller) */
    qemu_fdt_add_subnode(fdt, "/soc/cpc@1fbf0000");
    qemu_fdt_setprop_string(fdt, "/soc/cpc@1fbf0000",
                            "compatible", "mti,mips-cpc");
    qemu_fdt_setprop_cells(fdt, "/soc/cpc@1fbf0000",
                           "reg", 0x1fbf0000, 0x6000);

    /* SYSCTL (system controller) */
    {
        static const char * const sysc_compat[2] = {
            "mediatek,mt7621-sysc", "syscon"
        };
        qemu_fdt_add_subnode(fdt, "/soc/syscon@1e000000");
        qemu_fdt_setprop_string_array(fdt, "/soc/syscon@1e000000",
                                      "compatible",
                                      (char **)&sysc_compat,
                                      ARRAY_SIZE(sysc_compat));
        qemu_fdt_setprop_cells(fdt, "/soc/syscon@1e000000",
                               "reg", 0x1e000000, 0x100);
    }

    /* GPIO (96 pins, interrupt-parent = GIC 12, also interrupt-controller) */
    {
        uint32_t gpio_ph = qemu_fdt_alloc_phandle(fdt);
        qemu_fdt_add_subnode(fdt, "/soc/gpio@1e000600");
        qemu_fdt_setprop_string(fdt, "/soc/gpio@1e000600",
                                "compatible", "mediatek,mt7621-gpio");
        qemu_fdt_setprop_cells(fdt, "/soc/gpio@1e000600",
                               "reg", 0x1e000600, 0x100);
        qemu_fdt_setprop(fdt, "/soc/gpio@1e000600",
                         "interrupt-controller", NULL, 0);
        qemu_fdt_setprop_cell(fdt, "/soc/gpio@1e000600",
                              "#interrupt-cells", 2);
        qemu_fdt_setprop_cell(fdt, "/soc/gpio@1e000600",
                              "phandle", gpio_ph);
        qemu_fdt_setprop_cell(fdt, "/soc/gpio@1e000600",
                              "interrupt-parent", gic_ph);
        qemu_fdt_setprop_cells(fdt, "/soc/gpio@1e000600",
                               "interrupts", FDT_GIC_SHARED,
                               MT7621_GIC_IRQ_GPIO, FDT_IRQ_TYPE_LEVEL_HIGH);
    }

    /* UART0 (console) */
    qemu_fdt_add_subnode(fdt, "/soc/serial@1e000c00");
    qemu_fdt_setprop_string(fdt, "/soc/serial@1e000c00",
                            "compatible", "ns16550a");
    qemu_fdt_setprop_cells(fdt, "/soc/serial@1e000c00",
                           "reg", 0x1e000c00, 0x100);
    qemu_fdt_setprop_cell(fdt, "/soc/serial@1e000c00", "reg-shift", 2);
    qemu_fdt_setprop_cell(fdt, "/soc/serial@1e000c00", "reg-io-width", 4);
    qemu_fdt_setprop_cell(fdt, "/soc/serial@1e000c00",
                          "interrupt-parent", gic_ph);
    qemu_fdt_setprop_cells(fdt, "/soc/serial@1e000c00",
                           "interrupts", FDT_GIC_SHARED,
                           MT7621_GIC_IRQ_UART0, FDT_IRQ_TYPE_LEVEL_HIGH);

    /* UART1 */
    qemu_fdt_add_subnode(fdt, "/soc/serial@1e000d00");
    qemu_fdt_setprop_string(fdt, "/soc/serial@1e000d00",
                            "compatible", "ns16550a");
    qemu_fdt_setprop_cells(fdt, "/soc/serial@1e000d00",
                           "reg", 0x1e000d00, 0x100);
    qemu_fdt_setprop_cell(fdt, "/soc/serial@1e000d00", "reg-shift", 2);
    qemu_fdt_setprop_cell(fdt, "/soc/serial@1e000d00", "reg-io-width", 4);
    qemu_fdt_setprop_cell(fdt, "/soc/serial@1e000d00",
                          "interrupt-parent", gic_ph);
    qemu_fdt_setprop_cells(fdt, "/soc/serial@1e000d00",
                           "interrupts", FDT_GIC_SHARED,
                           MT7621_GIC_IRQ_UART1, FDT_IRQ_TYPE_LEVEL_HIGH);

    /* UART2 */
    qemu_fdt_add_subnode(fdt, "/soc/serial@1e000e00");
    qemu_fdt_setprop_string(fdt, "/soc/serial@1e000e00",
                            "compatible", "ns16550a");
    qemu_fdt_setprop_cells(fdt, "/soc/serial@1e000e00",
                           "reg", 0x1e000e00, 0x100);
    qemu_fdt_setprop_cell(fdt, "/soc/serial@1e000e00", "reg-shift", 2);
    qemu_fdt_setprop_cell(fdt, "/soc/serial@1e000e00", "reg-io-width", 4);
    qemu_fdt_setprop_cell(fdt, "/soc/serial@1e000e00",
                          "interrupt-parent", gic_ph);
    qemu_fdt_setprop_cells(fdt, "/soc/serial@1e000e00",
                           "interrupts", FDT_GIC_SHARED,
                           MT7621_GIC_IRQ_UART2, FDT_IRQ_TYPE_LEVEL_HIGH);

    /* Timer */
    qemu_fdt_add_subnode(fdt, "/soc/timer@1e000100");
    qemu_fdt_setprop_string(fdt, "/soc/timer@1e000100",
                            "compatible", "mediatek,mt7621-timer");
    qemu_fdt_setprop_cells(fdt, "/soc/timer@1e000100",
                           "reg", 0x1e000100, 0x100);
    qemu_fdt_setprop_cell(fdt, "/soc/timer@1e000100",
                          "interrupt-parent", gic_ph);
    /* Timer/wdt share GIC local interrupts via syscon, so no shared IRQ here;
     * the Linux driver reads the GIC timer and CP0 Compare for tick. */

    /* SPI */
    qemu_fdt_add_subnode(fdt, "/soc/spi@1e000b00");
    qemu_fdt_setprop_string(fdt, "/soc/spi@1e000b00",
                            "compatible", "mediatek,mt7621-spi");
    qemu_fdt_setprop_cells(fdt, "/soc/spi@1e000b00",
                           "reg", 0x1e000b00, 0x100);

    /* /chosen */
    qemu_fdt_add_subnode(fdt, "/chosen");
    qemu_fdt_setprop_string(fdt, "/chosen", "stdout-path",
                            "/soc/serial@1e000c00:115200");

    /* /memory@0 — reg will be filled in by the FDT filter at boot */
    qemu_fdt_add_subnode(fdt, "/memory@0");
    qemu_fdt_setprop_string(fdt, "/memory@0", "device_type", "memory");
    ram_low_sz = MIN(ms->ram_size, 256 * MiB);
    qemu_fdt_setprop_cells(fdt, "/memory@0", "reg",
                           0x00000000, ram_low_sz);

    return fdt;
}

/* ------------------------------------------------------------------ */
/* FDT filter: called before boot to patch in runtime values */
static void *mt7621_fdt_filter(void *opaque, const void *fdt_orig,
                               const void *match_data, hwaddr *load_addr)
{
    MT7621State *s = MT7621_MACHINE(opaque);
    MachineState *ms = MACHINE(opaque);
    void *fdt;
    int fdt_size;
    const char *cmdline;
    unsigned int ram_low_sz;

    fdt_size = fdt_totalsize(fdt_orig) * 2;
    fdt = g_malloc0(fdt_size);
    if (fdt_open_into(fdt_orig, fdt, fdt_size) < 0) {
        error_report("mt7621: failed to expand FDT");
        g_free(fdt);
        exit(1);
    }

    /* Patch /chosen/bootargs */
    cmdline = ms->kernel_cmdline;
    if (cmdline && cmdline[0]) {
        qemu_fdt_setprop_string(fdt, "/chosen", "bootargs", cmdline);
    } else {
        qemu_fdt_setprop_string(fdt, "/chosen", "bootargs",
                                "earlyprintk console=ttyS0,115200");
    }

    /* Patch /memory@0 reg with actual RAM size */
    ram_low_sz = MIN(ms->ram_size, 256 * MiB);
    qemu_fdt_setprop_cells(fdt, "/memory@0", "reg",
                           0x00000000, ram_low_sz);

    /* Record the kseg0 virtual address where FDT will be placed */
    *load_addr = cpu_mips_phys_to_kseg0(NULL, s->fdt_base);

    return g_steal_pointer(&fdt);
}

/* ------------------------------------------------------------------ */
/* Firmware / bootloader generation */
static void mt7621_gen_firmware(void *p, hwaddr kernel_entry, hwaddr fdt_addr)
{
    /*
     * Boot sequence:
     * 1. Configure GIC base address via GCR registers (kseg1 access)
     * 2. Configure CPC base address
     * 3. Jump to kernel via UHI boot protocol (a0=-2, a1=fdt_addr)
     */

    /* Step 1: Move GIC to 0x1FBC0000 and enable */
    bl_gen_write_ulong(&p,
        cpu_mips_phys_to_kseg1(NULL, GCR_BASE_ADDR + GCR_GIC_BASE_OFS),
        0x1fbc0000 | GCR_GIC_BASE_GICEN_MSK);

    /* Step 2: Move CPC to 0x1FBF0000 and enable */
    bl_gen_write_ulong(&p,
        cpu_mips_phys_to_kseg1(NULL, GCR_BASE_ADDR + GCR_CPC_BASE_OFS),
        0x1fbf0000 | GCR_GIC_BASE_GICEN_MSK);

    /*
     * Step 3: UHI boot protocol
     *   a0 = -2       → UHI FDT marker
     *   a1 = fdt_addr → kseg0 virtual address of FDT blob
     *   a2 = a3 = 0   → unused
     */
    bl_gen_jump_kernel(&p,
        true, 0,              /* sp  = 0 (kernel sets up own stack) */
        true, (int32_t)-2,    /* a0  = -2 (UHI FDT marker) */
        true, fdt_addr,       /* a1  = FDT kseg0 virtual address */
        true, 0, true, 0,     /* a2 = a3 = 0 */
        kernel_entry);
}

/* ------------------------------------------------------------------ */
/* SPI Flash helpers */

/*
 * Flash size detection: round up to standard SPI NOR flash sizes.
 * MT7621 supports 4/8/16/32 MB SPI NOR chips.
 */
static hwaddr mt7621_align_flash_size(int64_t file_size)
{
    if (file_size <= 4 * MiB) {
        return 4 * MiB;
    } else if (file_size <= 8 * MiB) {
        return 8 * MiB;
    } else if (file_size <= 16 * MiB) {
        return 16 * MiB;
    } else if (file_size <= 32 * MiB) {
        return 32 * MiB;
    }
    error_report("mt7621: pflash file too large (%lld bytes, max %dMB)",
                 (long long)file_size, (int)(MT7621_FLASH_MAX_SIZE / MiB));
    exit(1);
}

/*
 * Flash persistence: write RAM-backed flash buffer back to the
 * BlockBackend file on VM exit.
 */
static void mt7621_flash_save(Notifier *notifier, void *data)
{
    MT7621State *s = container_of(notifier, MT7621State, flash_save_notifier);

    /*
     * Flash persistence is now handled in real-time via blk_pwrite() in
     * the SPI handler (spi_flash_persist).  This exit notifier is kept as
     * a safety net but typically fails with ENOMEDIUM because the block
     * subsystem is already shut down by the time exit notifiers run.
     * Silently ignore failures — all writes were already persisted.
     */
    if (s->flash_blk && s->flash_data) {
        blk_pwrite(s->flash_blk, 0, s->flash_size, s->flash_data, 0);
    }
}

/* ------------------------------------------------------------------ */
/* Machine initialization */
static void mt7621_machine_init(MachineState *machine)
{
    MT7621State *s = MT7621_MACHINE(machine);
    const void *fdt;
    int dt_size;
    hwaddr dtb_paddr, dtb_vaddr, kernel_entry;
    bool has_pflash = false;

    /* ---- Clocks ---- */
    s->cpu_clk = clock_new(OBJECT(machine), "CPUCLK");
    clock_set_hz(s->cpu_clk, 880000000);   /* 880 MHz */

    s->periph_clk = clock_new(OBJECT(machine), "PERIPHCLK");
    clock_set_hz(s->periph_clk, 50000000); /* 50 MHz */

    /* ---- CPS (Coherent Processing System: CPU + GIC + CPC + GCR) ---- */
    object_initialize_child(OBJECT(machine), "cps", &s->cps, TYPE_MIPS_CPS);
    object_property_set_str(OBJECT(&s->cps), "cpu-type",
                            machine->cpu_type, &error_fatal);
    object_property_set_uint(OBJECT(&s->cps), "num-vp",
                             machine->smp.cpus, &error_fatal);
    qdev_connect_clock_in(DEVICE(&s->cps), "clk-in", s->cpu_clk);
    sysbus_realize(SYS_BUS_DEVICE(&s->cps), &error_fatal);

    /*
     * Map CPS container over the entire address space with low priority.
     */
    /*
     * Map CPS container over entire address space with low priority (1).
     * Peripheral devices are mapped at higher priority (0) and take
     * precedence. This is the standard MIPS CPS pattern (see boston.c).
     */
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->cps), 0, 0, 1);

    /*
     * Fix GCR position within CPS container: CP0_CMGCRBase is 0
     * at realize time, so GCR ends up at container offset 0.
     * Relocate it to the correct physical address 0x1fbf8000.
     */
    memory_region_del_subregion(&s->cps.container, &s->cps.gcr.iomem);
    memory_region_set_address(&s->cps.gcr.iomem, 0);
    memory_region_add_subregion(&s->cps.container, 0x1fbf8000,
                                &s->cps.gcr.iomem);
    s->cps.gcr.gcr_base = 0x1fbf8000;

    /* ---- Memory ---- */
    /*
     * MT7621 DRAM address space has a 64 MB gap at 0x1C000000–0x1FFFFFFF
     * (peripherals, SPI flash, CPC, GIC).  For RAM > 448 MB (LOWMEM max),
     * split into LOWMEM (0x0, ≤448 MB) and HIGHMEM (0x20000000, 64 MB)
     * to match real hardware.  Flash at 0x1FC00000 sits in the gap and is
     * directly visible — no overlap with DRAM.
     *
     * For RAM ≤ 448 MB, map the whole machine->ram at 0x0 and add a
     * wrap-around alias for Breed's DRAM-size auto-detection.
     */
    if (machine->ram_size > MT7621_LOWMEM_MAX_SIZE) {
        /* HIGHMEM: remaining RAM at 0x20000000 (max 64 MB) */
        hwaddr highmem_size = MIN(machine->ram_size - MT7621_LOWMEM_MAX_SIZE,
                                  MT7621_HIGHMEM_SIZE);
        memory_region_init_alias(&s->dram_remap, NULL, "mt7621.highmem",
                                 machine->ram, MT7621_LOWMEM_MAX_SIZE,
                                 highmem_size);
        memory_region_add_subregion(get_system_memory(), MT7621_HIGHMEM_BASE,
                                    &s->dram_remap);

        /* LOWMEM: first 448 MB of RAM at physical 0x0 */
        memory_region_init_alias(&s->dram, NULL, "mt7621.lowmem",
                                 machine->ram, 0, MT7621_LOWMEM_MAX_SIZE);
        memory_region_add_subregion(get_system_memory(), MT7621_DRAM_BASE,
                                    &s->dram);
    } else {
        /* All RAM fits in LOWMEM — map directly at physical 0x0 */
        memory_region_add_subregion(get_system_memory(), MT7621_DRAM_BASE,
                                    machine->ram);

        /*
         * DRAM alias: mirror RAM at the next address boundary so Breed's
         * wrap-around DRAM-size auto-detection works correctly.
         * Priority -1: lower than peripherals (flash, SYSCTL, etc.).
         */
        if (machine->ram_size < MT7621_MAX_DRAM_MAP) {
            hwaddr alias_size = MIN(machine->ram_size,
                                    MT7621_MAX_DRAM_MAP - machine->ram_size);
            memory_region_init_alias(&s->dram_alias, NULL, "mt7621.dram-alias",
                                     machine->ram, 0, alias_size);
            memory_region_add_subregion_overlap(get_system_memory(),
                                                machine->ram_size,
                                                &s->dram_alias, -1);
        }
    }

    /*
     * SPI Flash (RAM-backed, read/write).  The flash data buffer is
     * shared with the SPI controller so that Page-Program / Sector-Erase
     * commands modify the same backing store the CPU sees via the
     * memory-mapped window at 0xBFC00000.
     *
     * Boot priority (AND logic between -bios and pflash):
     *   pflash + -bios : pflash loaded, -bios overlaid at offset 0
     *   pflash only     : complete flash dump, boot from offset 0
     *   -bios only      : bios at offset 0, rest 0xFF
     *   neither         : generated minimal bootloader
     */
    {
        DriveInfo *pflash_dinfo = drive_get(IF_PFLASH, 0, 0);

        if (pflash_dinfo) {
            int64_t file_size;
            s->flash_blk = blk_by_legacy_dinfo(pflash_dinfo);
            /*
             * Request write permission only when flash-writable=true (default
             * is read-only).  The BlockBackend will be claimed (attached to
             * the SPI controller device) later to avoid the orphan-drive
             * validation error.
             */
            if (s->flash_writable) {
                blk_set_perm(s->flash_blk,
                             BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE,
                             BLK_PERM_ALL, &error_abort);
            } else {
                blk_set_perm(s->flash_blk,
                             BLK_PERM_CONSISTENT_READ,
                             BLK_PERM_ALL, &error_abort);
            }
            file_size = blk_getlength(s->flash_blk);
            if (file_size < 0) {
                error_report("mt7621: cannot determine pflash file size");
                exit(1);
            }
            s->flash_size = mt7621_align_flash_size(file_size);
            has_pflash = true;
        } else {
            s->flash_size = MT7621_FLASH_SIZE;
        }

        /* Allocate flash RAM buffer (0xFF = erased SPI NOR flash) */
        s->flash_data = g_malloc(s->flash_size);
        memset(s->flash_data, 0xFF, s->flash_size);

        /* Load pflash file content into buffer */
        if (has_pflash) {
            int64_t file_size = blk_getlength(s->flash_blk);
            if (file_size > 0) {
                blk_pread(s->flash_blk, 0, file_size, s->flash_data, 0);
            }
            printf("MT7621: loaded pflash (%lld bytes, flash %llu bytes, %s)\n",
                   (long long)file_size,
                   (unsigned long long)s->flash_size,
                   s->flash_writable ? "writable" : "read-only");
        }

        /*
         * Create RAM-backed MemoryRegion at the reset vector (0x1FC00000).
         * Flash sits in the GAP region (0x1C000000-0x1FFFFFFF) which is
         * never covered by DRAM, so it is always directly visible.
         * Priority 0 ensures flash wins over the wrap-around alias (-1).
         */
        memory_region_init_ram_ptr(&s->flash, NULL, "mt7621.flash",
                                   s->flash_size, s->flash_data);
        memory_region_add_subregion_overlap(get_system_memory(),
                                            MT7621_FLASH_BASE,
                                            &s->flash, 0);

        /*
         * Register exit persistence notifier for pflash (only in writable mode;
         * real-time persistence via SPI handler handles the actual writes).
         */
        if (has_pflash && s->flash_writable) {
            s->flash_save_notifier.notify = mt7621_flash_save;
            qemu_add_exit_notifier(&s->flash_save_notifier);
        }
    }

    /*
     * Overlay -bios at flash offset 0 if provided.
     * When pflash + -bios are both given (AND logic), the bios replaces
     * the bootloader portion of the flash dump while kernel and rootfs
     * data from pflash remains at higher offsets.
     */
    if (machine->firmware) {
        ssize_t bios_size;
        bios_size = load_image_size(machine->firmware, s->flash_data,
                                    s->flash_size);
        if (bios_size < 0) {
            error_report("mt7621: failed to load firmware '%s'",
                         machine->firmware);
            exit(1);
        }
        printf("MT7621: loaded firmware '%s' (%zd bytes at 0x%llx)\n",
               machine->firmware, bios_size,
               (unsigned long long)MT7621_FLASH_BASE);
    } else if (!has_pflash) {
        /*
         * No firmware and no pflash: write a minimal bootloader that
         * just configures GIC/CPC addresses, then spins.
         */
        mt7621_gen_firmware(s->flash_data,
                            cpu_mips_phys_to_kseg0(NULL, MT7621_FLASH_BASE),
                            cpu_mips_phys_to_kseg0(NULL, MT7621_FLASH_BASE));
    }

    /*
     * XIP shadow copy: On MT7621, SPI flash is also accessible at
     * physical address 0 (XIP boot mode).  Copy flash content into the
     * beginning of DRAM so "boot mem 0x50000" reads the kernel image
     * from flash via physical 0, while writes go to DRAM.
     *
     * Flash at 0x1FC00000 is in the GAP region (not DRAM), so the reset
     * vector is served by the flash MemoryRegion directly — no shadow
     * copy needed there.
     */
    {
        /*
         * DIAGNOSIS: XIP shadow copy disabled.
         * The shadow copy fills DRAM[0..flash_size] with flash content,
         * including erased sectors (0xFF). If Breed's heap overlaps with
         * this region, dlmalloc reads 0xFFFFFFFF as valid chunk headers,
         * causing heap corruption and allocation failures.
         */
        /* uint8_t *dram_ptr = memory_region_get_ram_ptr(machine->ram); */
        /* hwaddr xip_size = MIN(s->flash_size, machine->ram_size); */
        /* memcpy(dram_ptr, s->flash_data, xip_size); */
    }

    /*
     * Breed bootloader dlmalloc workaround (version-independent).
     *
     * Breed's dlmalloc has a bug in its non-contiguous MORECORE path:
     * when sbrk returns memory at a different address than the old top
     * chunk and old_top_size < MINSIZE (16), the allocator corrupts the
     * new top chunk's size field to 1:
     *
     *   li   $v1, 1           # 0x24030001  (addiu $v1, $zero, 1)
     *   <branch instruction>
     *   sw   $v1, 4($v0)      # 0xAC430004  ← corrupts top->size
     *
     * The opcode 0xAC430004 appears 19 times in Breed, but the specific
     * 3-instruction sequence (li $v1,1 … sw $v1,4($v0)) is unique.
     * Pattern mode (vaddr==2) reads the instruction 8 bytes before the
     * target to verify context, disambiguating the bug from legitimate
     * uses of the same opcode (e.g. struct copy in sub_8FFCA164).
     *
     * This makes the fix portable across ALL Breed versions regardless
     * of where the decompressed code is relocated in memory.
     *
     * Only active when firmware is loaded, zero impact on other machines.
     */
    if (machine->firmware || has_pflash) {
        mips_tcg_fixup_pattern(
            0xAC430004,   /* target: sw $v1, 4($v0) — buggy instruction */
            0x00000000,   /* replacement: NOP */
            0x24030001,   /* context: li $v1, 1 — 2 instructions before */
            -8            /* 8 bytes before target (2 × 4-byte MIPS insns) */
        );
    }
    /*
     * Bios-only mode: pre-configure CPS GCR so Breed sees GIC/CPC as
     * connected. The actual return is after all peripherals are created.
     */
    if ((machine->firmware || has_pflash) && !machine->kernel_filename) {
        info_report("MT7621: flash-boot mode - pre-configuring GIC/CPC");
        {
            MIPSGCRState *gcr = &s->cps.gcr;
            gcr->gic_base = 0x1fbc0000 | GCR_GIC_BASE_GICEN_MSK;
            memory_region_set_address(gcr->gic_mr,
                gcr->gic_base & 0xFFFFFFFE0000ULL);
            memory_region_set_enabled(gcr->gic_mr,
                gcr->gic_base & GCR_GIC_BASE_GICEN_MSK);
            gcr->cpc_base = 0x1fbf0000 | GCR_GIC_BASE_GICEN_MSK;
            memory_region_set_address(gcr->cpc_mr,
                gcr->cpc_base & 0xFFFFFFFF8000ULL);
            memory_region_set_enabled(gcr->cpc_mr,
                gcr->cpc_base & GCR_GIC_BASE_GICEN_MSK);
        }

    }

    /* ---- SYSCTL (system controller: chip ID, clocks, reset) ---- */
    {
        DeviceState *sysctl_dev = qdev_new(TYPE_MT7621_SYSCTL);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sysctl_dev), &error_fatal);
        /* Map with overlap priority 0 (highest) to ensure it wins over
         * the CPS container at priority 1 */
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(sysctl_dev), 0,
                                MT7621_SYSCTL_BASE, 0);
    }

    /* ---- Ethernet Frame Engine stub ---- */
    {
        DeviceState *eth_dev = qdev_new(TYPE_MT7621_ETH);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(eth_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(eth_dev), 0, MT7621_ETH_BASE);
    }

    /* ---- SPI Controller (shares flash_data buffer) ---- */
    {
        DeviceState *spi_dev = qdev_new(TYPE_MT7621_SPI);
        MT7621SPIState *spi = MT7621_SPI(spi_dev);
        spi->flash_data = s->flash_data;
        spi->flash_size = s->flash_size;
        spi->flash_blk = s->flash_writable ? s->flash_blk : NULL;
        sysbus_realize_and_unref(SYS_BUS_DEVICE(spi_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(spi_dev), 0, MT7621_SPI_BASE);

        /*
         * Claim the pflash BlockBackend by attaching it to the SPI device.
         * This prevents the blockdev validation from flagging the drive as
         * an orphan ("machine type does not support if=pflash").
         */
        if (s->flash_blk) {
            blk_attach_dev(s->flash_blk, spi_dev);
        }
    }

    /* ---- Memory Controller stub ---- */
    {
        DeviceState *memctrl_dev = qdev_new(TYPE_MT7621_MEMCTRL);
        object_property_set_uint(OBJECT(memctrl_dev), "dram-size",
                                 machine->ram_size, &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(memctrl_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(memctrl_dev), 0, MT7621_MEMCTRL_BASE);
    }

    /* ---- GPIO (96 pins, 3 banks, IRQ aggregated to GIC 12) ---- */
    {
        DeviceState *gpio_dev = qdev_new(TYPE_MT7621_GPIO);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(gpio_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(gpio_dev), 0, MT7621_GPIO_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(gpio_dev), 0,
                          get_cps_irq(&s->cps, MT7621_GIC_IRQ_GPIO));
    }

    /* ---- GDMA (General DMA, 16 channels) stub ---- */
    create_unimplemented_device("mt7621.gdma", MT7621_GDMA_BASE, 0x800);

    /* ---- NFI (NAND Flash Interface) stub ---- */
    create_unimplemented_device("mt7621.nfi", MT7621_NFI_BASE, 0x1000);

    /* ---- Timer (2x GPT + watchdog, ptimer-based) ---- */
    {
        DeviceState *timer_dev = qdev_new(TYPE_MT7621_TIMER);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(timer_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(timer_dev), 0, MT7621_TIMER_BASE);
    }

    /* ---- UART0 (ns16550a compatible, reg-shift=2) ---- */
    s->uart0 = serial_mm_init(get_system_memory(),
                              MT7621_UART0_BASE, 2,
                              get_cps_irq(&s->cps, MT7621_GIC_IRQ_UART0),
                              115200,
                              serial_hd(0), DEVICE_NATIVE_ENDIAN);

    /* flash-boot mode: all peripherals created, skip FDT+kernel */
    if ((machine->firmware || has_pflash) && !machine->kernel_filename) {
        return;  /* CPU will boot from flash at 0xBFC00000 */
    }

    /* ---- FDT generation ---- */
    fdt = mt7621_create_fdt(s, &dt_size);
    if (!fdt) {
        error_report("mt7621: FDT creation failed");
        exit(1);
    }
    machine->fdt = (void *)fdt;

    /* ---- Kernel loading ---- */
    if (machine->kernel_filename) {
        uint64_t kernel_entry_raw, kernel_high;

        kernel_entry_raw = 0;
        kernel_high = 0;
        if (!load_elf(machine->kernel_filename, NULL,
                      cpu_mips_kseg0_to_phys, NULL,
                      &kernel_entry_raw, NULL,
                      &kernel_high, NULL,
                      ELFDATA2LSB, EM_MIPS, 1, 0)) {
            error_report("mt7621: could not load kernel '%s'",
                         machine->kernel_filename);
            exit(1);
        }
        kernel_entry = kernel_entry_raw;

        /*
         * Place FDT blob in RAM after kernel.
         * Align to 64 KiB boundary, then convert to kseg0 virtual address.
         */
        dtb_paddr = QEMU_ALIGN_UP(kernel_high, 64 * KiB);
        dtb_vaddr = cpu_mips_phys_to_kseg0(NULL, dtb_paddr);
        s->fdt_base = dtb_vaddr;
    } else {
        /*
         * No kernel provided — this is a firmware-only launch.
         * Place FDT at 128 MiB in kseg0 for a firmware to find.
         */
        dtb_paddr = 128 * MiB;
        dtb_vaddr = cpu_mips_phys_to_kseg0(NULL, dtb_paddr);
        s->fdt_base = dtb_vaddr;
        kernel_entry = dtb_vaddr;   /* firmware starts from here */
    }

    /* Run FDT through the filter to inject runtime values */
    {
        hwaddr load_addr;
        void *filtered_fdt = mt7621_fdt_filter(s, fdt, NULL, &load_addr);
        dtb_vaddr = load_addr;
        dt_size = fdt_totalsize(filtered_fdt);
        machine->fdt = filtered_fdt;

        /* Place the filtered FDT as a ROM blob in guest RAM */
        rom_add_blob_fixed("dtb", filtered_fdt, dt_size, dtb_paddr);
    }

    /* Register FDT random seed update on reset */
    qemu_register_reset_nosnapshotload(qemu_fdt_randomize_seeds,
                        rom_ptr_for_as(&address_space_memory,
                                       dtb_paddr, dt_size));

    /* ---- Bootloader in flash ---- */
    mt7621_gen_firmware(s->flash_data, kernel_entry, dtb_vaddr);
}

/* ------------------------------------------------------------------ */
/* Machine class */
static bool mt7621_machine_get_flash_writable(Object *obj, Error **errp)
{
    MT7621State *s = MT7621_MACHINE(obj);
    return s->flash_writable;
}

static void mt7621_machine_set_flash_writable(Object *obj, bool value,
                                              Error **errp)
{
    MT7621State *s = MT7621_MACHINE(obj);
    s->flash_writable = value;
}

static void mt7621_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "MediaTek MT7621 SoC (MIPS 1004Kc)";
    mc->init = mt7621_machine_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("1004Kc");
    mc->min_cpus = 1;
    mc->max_cpus = 2;       /* MT7621 has 2 VPEs */
    mc->default_ram_id = "mt7621.dram";
    mc->default_ram_size = 256 * MiB;
    mc->no_floppy = 1;
    mc->no_parallel = 1;

    object_class_property_add_bool(oc, "flash-writable",
                                   mt7621_machine_get_flash_writable,
                                   mt7621_machine_set_flash_writable);
    object_class_property_set_description(oc, "flash-writable",
        "Enable flash write persistence to backing file "
        "(default: off = read-only, writes affect RAM only)");
}

static const TypeInfo mt7621_machine_type = {
    .name          = TYPE_MT7621_MACHINE,
    .parent        = TYPE_MACHINE,
    .instance_size = sizeof(MT7621State),
    .class_init    = mt7621_machine_class_init,
};

static void mt7621_machine_register_types(void)
{
    type_register_static(&mt7621_machine_type);
}

type_init(mt7621_machine_register_types)
