/*
 * MT7621 SYSCTL (System Controller) emulation
 *
 * Register map from MT7621 Programming Guide:
 *   0x00  CHIPID0_3    "MT76" ASCII (R)
 *   0x04  CHIPID4_7    "21  " ASCII (R)  -> 0x20203132 == MT7621_CHIP_NAME1
 *   0x0C  CHIP_REV_ID  Chip Revision Identification (R)
 *   0x10  SYSCFG       System Configuration (RW)
 *   0x14  SYSCFG1      System Configuration 1 (RW)
 *   0x2C  CLKCFG0      Clock Configuration 0 (RW)
 *   0x30  CLKCFG1      Clock Configuration 1 (RW)
 *   0x34  RSTCTL       Reset Control (RW)
 *   0x38  INTCTRL      Interrupt Control (RW)
 *   0x3C  INTSTATUS    Interrupt Status (RW)
 *   0x44  CUR_CLK_STS  Current clock status (R)
 *   0x58  TESTCTL      Test Control (RW)
 *   0x60  GPIO_MODE    GPIO purpose selection (RW)
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/misc/mt7621_sysctl.h"
#include "hw/core/qdev-properties.h"
#include "system/reset.h"
#include "system/runstate.h"

/* Register offsets */
#define CHIPID0_3       0x00
#define CHIPID4_7       0x04
#define CHIP_REV_ID     0x0C
#define SYSCFG          0x10
#define SYSCFG1         0x14
#define CLKCFG0         0x2C
#define CLKCFG1         0x30
#define RSTCTL          0x34
#define INTCTRL         0x38
#define INTSTATUS       0x3C
#define CUR_CLK_STS     0x44
#define TESTCTL         0x58
#define GPIO_MODE       0x60
#define REG_COUNT       0x100

#define RSTCTL_SW_RST   BIT(0)

static uint64_t mt7621_sysctl_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    MT7621SysctlState *s = MT7621_SYSCTL(opaque);
    uint32_t idx = offset >> 2;
    uint32_t val;

    if (idx >= (REG_COUNT >> 2)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_sysctl: bad read 0x%" HWADDR_PRIx "\n", offset);
        return 0;
    }

    val = s->regs[idx];

    /* Special read-side handling */
    switch (offset) {
    case CUR_CLK_STS:
        /* Must be non-zero: Breed uses FDIV from bits[12:8] as divisor */
        val = val ? val : 0x00000101;
        break;
    default:
        break;
    }

    return val;
}

static void mt7621_sysctl_write(void *opaque, hwaddr offset,
                                uint64_t val, unsigned size)
{
    MT7621SysctlState *s = MT7621_SYSCTL(opaque);
    uint32_t idx = offset >> 2;

    if (idx >= (REG_COUNT >> 2)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_sysctl: bad write 0x%" HWADDR_PRIx
                      " val 0x%" PRIx64 "\n", offset, val);
        return;
    }

    switch (offset) {
    case CHIPID0_3:
    case CHIPID4_7:
    case CHIP_REV_ID:
    case CUR_CLK_STS:
        /* Read-only */
        break;

    case RSTCTL:
        s->regs[idx] = val;
        if (val & RSTCTL_SW_RST) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        break;

    default:
        /*
         * All other registers (SYSCFG, SYSCFG1, CLKCFG0, CLKCFG1,
         * INTCTRL, INTSTATUS, TESTCTL, GPIO_MODE, etc.) are
         * simple read/write — just store the value.
         */
        s->regs[idx] = val;
        break;
    }
}

static const MemoryRegionOps mt7621_sysctl_ops = {
    .read  = mt7621_sysctl_read,
    .write = mt7621_sysctl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void mt7621_sysctl_reset_hold(Object *obj, ResetType type)
{
    MT7621SysctlState *s = MT7621_SYSCTL(obj);
    memset(s->regs, 0, sizeof(s->regs));

    s->regs[CHIPID0_3 >> 2] = 0x3637544D;    /* "MT76" == MT7621_CHIP_NAME0 */
    s->regs[CHIPID4_7 >> 2] = 0x20203132;    /* "21  " == MT7621_CHIP_NAME1 */
    /*
     * CHIP_REV_ID (0x0C):
     *   bit 17 CPU_ID  : 1 = dual-core (MT7621A). U-Boot SPL reads this to
     *                     boot the secondary core (bootup_secondary_core).
     *   bit 16 PKG_ID  : 1 = A-package.
     *   bits 11:8 VER  : silicon version (1).
     *   bits 3:0  ECO  : eco revision (3 for MT7621A).
     */
    s->regs[CHIP_REV_ID >> 2] = 0x00030103;  /* MT7621A dual-core, rev 1, eco 3 */
    /*
     * SYSCFG (0x10): System configuration register.
     *   Bits 3:0  = CHIP_MODE (boot device), derived from flash size:
     *     2 = SPI-NOR 3-Byte Addr (flash <= 16 MiB),
     *     3 = SPI-NOR 4-Byte Addr (flash >  16 MiB, e.g. 32 MiB).
     *     U-Boot reads this in print_cpuinfo() to print the boot device.
     *   Bit 4    = DRAM_TYPE: 0 = DDR3 (matches -m RAM).
     *   Bits 8:6 = XTAL_MODE_SEL: 1 = 20 MHz crystal.
     */
    s->regs[SYSCFG >> 2] = (1U << 6) |
        ((s->flash_size > 16 * MiB) ? 3 : 2);  /* XTAL=20MHz, DDR3, CHIP_MODE */
    /*
     * CLKCFG0 (0x2C): Clock configuration register.
     *   Bits 31:30 = CPU_CLK_SEL: 0 = 880 MHz, 1 = 440 MHz
     *   Bits 23:0  = peripheral clock enables
     * Must be 0 for 880 MHz CPU mode so Breed calculates correct clocks.
     */
    s->regs[CLKCFG0 >> 2] = 0x00100001;
    /* Enable clocks: ETH[23], FE[6], GDMA[14], UART[19,20,21], SPI[18], GPIO[13] */
    s->regs[CLKCFG1 >> 2] = BIT(23) | BIT(6) | BIT(14) | BIT(19) | BIT(20) | BIT(21) | BIT(18) | BIT(13);
    /*
     * CUR_CLK_STS (0x44): clock configuration status.
     *   Bits 4:0  = FFRAC (CPU PLL fraction) = 1
     *   Bits 12:8 = CPU_FDIV (feedback divider) = 1
     * Breed: cpu_clk = base * (FBDIV+1) / prediv / FDIV * FFRAC
     * With FDIV=1: CPU = 20MHz * 44 / 1 * 1 = 880 MHz
     * Bits 12:8 MUST be non-zero (Breed traps on div-by-zero).
     */
    s->regs[CUR_CLK_STS >> 2] = 0x00000101;
}

static const Property mt7621_sysctl_properties[] = {
    DEFINE_PROP_UINT32("flash-size", MT7621SysctlState, flash_size, 0),
};

static void mt7621_sysctl_init(Object *obj)
{
    MT7621SysctlState *s = MT7621_SYSCTL(obj);
    memory_region_init_io(&s->iomem, obj, &mt7621_sysctl_ops, s,
                          TYPE_MT7621_SYSCTL, REG_COUNT);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void mt7621_sysctl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    dc->desc = "MT7621 System Controller";
    rc->phases.hold = mt7621_sysctl_reset_hold;
    device_class_set_props(dc, mt7621_sysctl_properties);
}

static const TypeInfo mt7621_sysctl_info = {
    .name          = TYPE_MT7621_SYSCTL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT7621SysctlState),
    .instance_init = mt7621_sysctl_init,
    .class_init    = mt7621_sysctl_class_init,
};

static void mt7621_sysctl_register_types(void)
{
    type_register_static(&mt7621_sysctl_info);
}
type_init(mt7621_sysctl_register_types)
