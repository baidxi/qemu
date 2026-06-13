/*
 * MT7621 GPIO Controller emulation
 *
 * 96 GPIO pins in 3 banks of 32.
 *
 * Register layout (based on Linux driver gpio-mt7621.c):
 *   BANK_STRIDE = 0x04 (banks at consecutive 4-byte offsets)
 *   Register types at 0x10 intervals:
 *
 *   Offset            Register  Description
 *   0x00 + bank*4     CTRL      Pin direction (1=output, 0=input)
 *   0x10 + bank*4     POL       Interrupt polarity
 *   0x20 + bank*4     DATA      Pin data value
 *   0x30 + bank*4     DSET      Data set (write-1 to set bit in DATA)
 *   0x40 + bank*4     DCLR      Data clear (write-1 to clear bit in DATA)
 *   0x50 + bank*4     REDGE     Rising edge detect enable
 *   0x60 + bank*4     FEDGE     Falling edge detect enable
 *   0x70 + bank*4     HLVL      High level detect enable
 *   0x80 + bank*4     LLVL      Low level detect enable
 *   0x90 + bank*4     STAT      Interrupt status (W1C)
 *   0xA0 + bank*4     EDGE      Edge detect
 *
 * Bank 0: pins GPIO0-31   (adds 0x00 to each register offset)
 * Bank 1: pins GPIO32-63  (adds 0x04 to each register offset)
 * Bank 2: pins GPIO64-95  (adds 0x08 to each register offset)
 *
 * Example: Bank 1 DATA = 0x20 + 0x04 = 0x24
 *
 * The GPIO acts as a cascaded interrupt-controller, aggregating
 * all pin interrupts onto GIC shared interrupt 12.
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/gpio/mt7621_gpio.h"

/* Register type offsets (base of each 16-byte row) */
#define GPIO_REG_CTRL      0x00
#define GPIO_REG_POL       0x10
#define GPIO_REG_DATA      0x20
#define GPIO_REG_DSET      0x30
#define GPIO_REG_DCLR      0x40
#define GPIO_REG_REDGE     0x50
#define GPIO_REG_FEDGE     0x60
#define GPIO_REG_HLVL      0x70
#define GPIO_REG_LLVL      0x80
#define GPIO_REG_STAT      0x90
#define GPIO_REG_EDGE      0xA0

#define GPIO_BANK_STRIDE   0x04
#define NUM_BANKS          3
#define PINS_PER_BANK      32

/*
 * Decode an absolute offset into (register_type, bank):
 *   register_type = (offset / 0x10) * 0x10   (row base: 0x00, 0x10, ..., 0xA0)
 *   bank          = (offset % 0x10) / 0x04    (column: 0, 1, 2)
 */
static inline int gpio_reg_type(hwaddr offset)
{
    return (offset / 0x10) * 0x10;
}

static inline int gpio_bank(hwaddr offset)
{
    return (offset % 0x10) / GPIO_BANK_STRIDE;
}

static uint64_t mt7621_gpio_read(void *opaque, hwaddr offset,
                                 unsigned size)
{
    MT7621GPIOState *s = MT7621_GPIO(opaque);
    int reg = gpio_reg_type(offset);
    int bank = gpio_bank(offset);

    if (bank >= NUM_BANKS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_gpio: bad bank %d (offset 0x%" HWADDR_PRIx ")\n",
                      bank, offset);
        return 0;
    }

    switch (reg) {
    case GPIO_REG_CTRL:  return s->bank[bank].ctrl;
    case GPIO_REG_POL:   return s->bank[bank].pol;
    case GPIO_REG_DATA:  return s->bank[bank].data;
    case GPIO_REG_DSET:  return 0;  /* write-only */
    case GPIO_REG_DCLR:  return 0;  /* write-only */
    case GPIO_REG_REDGE: return s->bank[bank].redge;
    case GPIO_REG_FEDGE: return s->bank[bank].fedge;
    case GPIO_REG_HLVL:  return s->bank[bank].hlvl;
    case GPIO_REG_LLVL:  return s->bank[bank].llvl;
    case GPIO_REG_STAT:  return s->bank[bank].stat;
    case GPIO_REG_EDGE:  return s->bank[bank].edge;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_gpio: bad read offset 0x%" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void mt7621_gpio_write(void *opaque, hwaddr offset,
                              uint64_t val, unsigned size)
{
    MT7621GPIOState *s = MT7621_GPIO(opaque);
    int reg = gpio_reg_type(offset);
    int bank = gpio_bank(offset);
    MT7621GPIOBank *b;

    if (bank >= NUM_BANKS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_gpio: bad bank %d (offset 0x%" HWADDR_PRIx ")\n",
                      bank, offset);
        return;
    }
    b = &s->bank[bank];

    switch (reg) {
    case GPIO_REG_CTRL:
        b->ctrl = val;
        break;
    case GPIO_REG_POL:
        b->pol = val;
        break;
    case GPIO_REG_DATA:
        b->data = val;
        break;
    case GPIO_REG_DSET:
        /* Write-1 to set corresponding bits in DATA */
        b->data |= val;
        break;
    case GPIO_REG_DCLR:
        /* Write-1 to clear corresponding bits in DATA */
        b->data &= ~val;
        break;
    case GPIO_REG_REDGE:
        b->redge = val;
        break;
    case GPIO_REG_FEDGE:
        b->fedge = val;
        break;
    case GPIO_REG_HLVL:
        b->hlvl = val;
        break;
    case GPIO_REG_LLVL:
        b->llvl = val;
        break;
    case GPIO_REG_STAT:
        /* Write-1-to-clear interrupt status */
        b->stat &= ~val;
        break;
    case GPIO_REG_EDGE:
        b->edge = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_gpio: bad write offset 0x%" HWADDR_PRIx "\n",
                      offset);
        break;
    }
}

static const MemoryRegionOps mt7621_gpio_ops = {
    .read  = mt7621_gpio_read,
    .write = mt7621_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mt7621_gpio_reset_hold(Object *obj, ResetType type)
{
    MT7621GPIOState *s = MT7621_GPIO(obj);

    for (int i = 0; i < NUM_BANKS; i++) {
        memset(&s->bank[i], 0, sizeof(MT7621GPIOBank));
    }
}

static void mt7621_gpio_init(Object *obj)
{
    MT7621GPIOState *s = MT7621_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &mt7621_gpio_ops, s,
                          TYPE_MT7621_GPIO, MT7621_GPIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    /* GPIO interrupt output (aggregated, connects to GIC 12) */
    sysbus_init_irq(sbd, &s->irq);
}

static void mt7621_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "MT7621 GPIO Controller";
    rc->phases.hold = mt7621_gpio_reset_hold;
}

static const TypeInfo mt7621_gpio_info = {
    .name          = TYPE_MT7621_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT7621GPIOState),
    .instance_init = mt7621_gpio_init,
    .class_init    = mt7621_gpio_class_init,
};

static void mt7621_gpio_register_types(void)
{
    type_register_static(&mt7621_gpio_info);
}

type_init(mt7621_gpio_register_types)
