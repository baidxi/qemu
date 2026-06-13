/*
 * MT7621 / RT2880 UART — non-standard register map
 *
 * Register offsets (word-aligned, regshift=2):
 *   0x00  RX (R)    0x04  TX (W)
 *   0x08  IER (R/W) 0x0C  IIR (R)
 *   0x10  FCR (W)   0x14  LCR (R/W)
 *   0x18  MCR (R/W) 0x1C  LSR (R)
 *   0x20  MSR (R)   0x28  Divisor Latch (R/W)
 */
#include "qemu/osdep.h"
#include "chardev/char.h"
#include "hw/char/mt7621_uart.h"

static uint64_t mt7621_uart_read(void *opaque, hwaddr offset, unsigned size)
{
    MT7621UartState *s = MT7621_UART(opaque);

    if (offset == 0x28) return s->dll;
    switch (offset) {
    case 0x00: return s->rx;
    case 0x08: return s->ier;
    case 0x0C: return 0x01;
    case 0x14: return s->lcr;
    case 0x18: return s->mcr;
    case 0x1C: return (s->rx ? 0x01 : 0x00) | 0x60;
    case 0x20: return 0x00;
    default:   return 0;
    }
}

static void mt7621_uart_write(void *opaque, hwaddr offset,
                              uint64_t val, unsigned size)
{
    MT7621UartState *s = MT7621_UART(opaque);
    uint8_t ch;

    if (offset == 0x28) { s->dll = val & 0xFF; return; }
    switch (offset) {
    case 0x04: ch = val & 0xFF; qemu_chr_write_all(s->chr, &ch, 1); break;
    case 0x08: s->ier = val; break;
    case 0x10: break;
    case 0x14: s->lcr = val; break;
    case 0x18: s->mcr = val; break;
    }
}

static const MemoryRegionOps mt7621_uart_ops = {
    .read  = mt7621_uart_read,
    .write = mt7621_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void mt7621_uart_init(Object *obj)
{
    MT7621UartState *s = MT7621_UART(obj);
    memory_region_init_io(&s->iomem, obj, &mt7621_uart_ops, s,
                          TYPE_MT7621_UART, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void mt7621_uart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->desc = "MT7621/RT2880 UART";
}

static const TypeInfo mt7621_uart_info = {
    .name          = TYPE_MT7621_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT7621UartState),
    .instance_init = mt7621_uart_init,
    .class_init    = mt7621_uart_class_init,
};
static void mt7621_uart_register_types(void) { type_register_static(&mt7621_uart_info); }
type_init(mt7621_uart_register_types)
