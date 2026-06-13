/*
 * MT7621 Memory Controller emulation (minimal stub)
 *
 * Provides only the DRAM_SIZE register that Linux reads to
 * determine installed RAM. All other registers are unimplemented.
 *
 * Register map:
 *   0x00  DRAM_SIZE   (R)  Detected DRAM size in bytes
 *   0x04  DRAM_CFG    (R)  DRAM configuration
 *   0x08  DDR2_CFG    (R)  DDR2/DDR3 timing parameters
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/mt7621_memctrl.h"
#include "hw/core/qdev-properties.h"

static uint64_t mt7621_memctrl_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    MT7621MemCtrlState *s = MT7621_MEMCTRL(opaque);
    uint32_t ret;

    switch (offset) {
    case 0x00:  /* DRAM_SIZE */
        ret = s->dram_size;
        break;
    case 0x04:  /* DRAM_CFG — DDR3, 16-bit bus, 256MB */
        ret = 0x004008C0;
        break;
    /*
     * SMT_CTRL / DDR_PHY registers (0x610–0x61C).
     * Breed reads 0x618 to extract FBDIV/PREDIV for DDR clock calculation.
     *   Bits 10:4  = FBDIV (feedback divider)
     *   Bits 13:12 = PREDIV (pre-divider selector)
     * Breed formula: DDR = XTAL * FBDIV * 4 / (1 << PREDIV)
     * Value 0x00000140 → FBDIV=20, PREDIV=0  → 20*20*4 = 1600 MHz
     */
    case 0x610:
        ret = 0x00000001;
        break;
    case 0x618:
        ret = 0x00000140;
        break;
    case 0x61C:
        ret = 0x00000001;
        break;
    case 0x648: /* MEMC_REG_CPU_PLL — CPU PLL configuration */
        /*
         * CPU = (FBDIV+1) * XTAL >> prediv_tbl[PREDIV] / FDIV * FFRAC
         * XTAL = 20MHz, FBDIV=43, PREDIV=0 → 44*20 = 880MHz
         * PREDIV[13:12], FBDIV[10:4]
         */
        ret = 0x000002B0;  /* FBDIV=43, PREDIV=0: (43+1)*20=880MHz */
        break;
    default:
        ret = 0x00000001;
        break;
    }
    return ret;
}

static void mt7621_memctrl_write(void *opaque, hwaddr offset,
                                 uint64_t val, unsigned size)
{
    qemu_log_mask(LOG_UNIMP,
                  "mt7621_memctrl: unimplemented write 0x%" HWADDR_PRIx
                  " = 0x%" PRIx64 "\n", offset, val);
}

static const MemoryRegionOps mt7621_memctrl_ops = {
    .read  = mt7621_memctrl_read,
    .write = mt7621_memctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mt7621_memctrl_reset_hold(Object *obj, ResetType type)
{
    /* DRAM_SIZE is set by machine via property, not reset */
}

static void mt7621_memctrl_init(Object *obj)
{
    MT7621MemCtrlState *s = MT7621_MEMCTRL(obj);

    memory_region_init_io(&s->iomem, obj, &mt7621_memctrl_ops, s,
                          TYPE_MT7621_MEMCTRL, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const Property mt7621_memctrl_properties[] = {
    DEFINE_PROP_UINT32("dram-size", MT7621MemCtrlState, dram_size,
                       256 * 1024 * 1024),
};

static void mt7621_memctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "MT7621 Memory Controller";
    device_class_set_props(dc, mt7621_memctrl_properties);
    rc->phases.hold = mt7621_memctrl_reset_hold;
}

static const TypeInfo mt7621_memctrl_info = {
    .name          = TYPE_MT7621_MEMCTRL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT7621MemCtrlState),
    .instance_init = mt7621_memctrl_init,
    .class_init    = mt7621_memctrl_class_init,
};

static void mt7621_memctrl_register_types(void)
{
    type_register_static(&mt7621_memctrl_info);
}
type_init(mt7621_memctrl_register_types)
