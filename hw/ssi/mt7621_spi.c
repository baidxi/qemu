/*
 * MT7621 SPI Controller — register-accurate from Linux spi-mt7621.c
 *
 * Register map:
 *   0x00  SPI_TRANS   (RW) bit16=BUSY, bit8=START, [7:0]=TX/RX count
 *   0x04  SPI_OPCODE  (W)  TX data buffer (4 bytes, weird byte order)
 *   0x08  SPI_DATA0   (R)  RX data bytes 0-3
 *   0x0C  SPI_DATA1   (R)  RX data bytes 4-7
 *   0x10  SPI_DATA2   (R)  RX data bytes 8-11
 *   0x14  SPI_DATA3   (R)  RX data bytes 12-15
 *   0x28  SPI_MASTER  (RW) clock divider[27:16], slave_sel[31:29], more_buf[2]
 *   0x2C  SPI_MOREBUF (RW) RX count[15:12], TX count[31:24]
 *   0x38  SPI_POLAR   (RW) CS polarity
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/ssi/mt7621_spi.h"

static uint64_t mt7621_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    MT7621SPIState *s = MT7621_SPI(opaque);
    uint32_t val;

    switch (offset) {
    case 0x00:  /* SPI_TRANS */
        return (s->busy ? (1 << 16) : 0) | s->ctrl;
    case 0x08:  /* SPI_DATA0: RX bytes 0-3 */
        val = s->rx_fifo[0] | (s->rx_fifo[1] << 8) |
              (s->rx_fifo[2] << 16) | (s->rx_fifo[3] << 24);
        return val;
    case 0x0C:  /* SPI_DATA1 */
    case 0x10:  /* SPI_DATA2 */
    case 0x14:  /* SPI_DATA3 */
    case 0x18:  /* SPI_DATA4 */
        return 0;  /* no more data */
    case 0x28:  /* SPI_MASTER */
        return s->master_cfg;
    case 0x2C:  /* SPI_MOREBUF */
        return 0;
    case 0x38:  /* SPI_POLAR (CS) */
        return s->cs_polar;
    default:
        return 0;
    }
}

static void mt7621_spi_write(void *opaque, hwaddr offset,
                             uint64_t val, unsigned size)
{
    MT7621SPIState *s = MT7621_SPI(opaque);

    switch (offset) {
    case 0x00:  /* SPI_TRANS: bit8=START, bit16=BUSY */
        s->ctrl = val;
        if (val & (1 << 8)) {  /* START */
            s->busy = true;
            s->busy = false;   /* instant completion */
        }
        break;
    case 0x04:  /* SPI_OPCODE: TX data (byte-swapped) */
        {
            /*
             * Breed sends 0x9F (JEDEC ID) as opcode.
             * The byte order is swab32'd by the Linux driver
             * for the first word, so a raw write of 0x9F becomes
             * 0x9F000000. We need to extract the byte.
             *
             * Breed writes byte 0x9F to SPI_OPCODE (word 0).
             * The actual flash command is in bits [7:0].
             */
            uint8_t cmd = val & 0xFF;
            if (!cmd && (val & 0xFF00))
                cmd = (val >> 8) & 0xFF;
            if (!cmd && (val & 0xFF0000))
                cmd = (val >> 16) & 0xFF;

            if (cmd == 0x9F) {  /* JEDEC ID */
                s->rx_fifo[0] = 0xEF;  /* Winbond */
                s->rx_fifo[1] = 0x40;  /* 128Mbit capacity */
                s->rx_fifo[2] = 0x18;  /* W25Q128 */
                s->rx_fifo[3] = 0x00;
            } else if (cmd == 0x05) {  /* Read Status */
                s->rx_fifo[0] = 0x00;  /* not busy, not write enabled */
            } else if (cmd == 0x90) {  /* Read Manufacturer ID */
                s->rx_fifo[0] = 0xEF;
                s->rx_fifo[1] = 0x00;
            } else if (cmd == 0xAB) {  /* Release from Deep Power Down */
                s->rx_fifo[0] = 0x00;
            } else if (cmd == 0x9E) {  /* Read Electronic Signature */
                s->rx_fifo[0] = 0x16;  /* 1.8V */
            }
            /* For unknown commands, return 0x00 */
        }
        break;
    case 0x28:  /* SPI_MASTER */
        s->master_cfg = val;
        break;
    case 0x2C:  /* SPI_MOREBUF */
        break;
    case 0x38:  /* SPI_POLAR (CS) */
        s->cs_polar = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps mt7621_spi_ops = {
    .read  = mt7621_spi_read,
    .write = mt7621_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void mt7621_spi_init(Object *obj)
{
    MT7621SPIState *s = MT7621_SPI(obj);
    memory_region_init_io(&s->iomem, obj, &mt7621_spi_ops, s,
                          TYPE_MT7621_SPI, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void mt7621_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->desc = "MT7621 SPI Controller";
}

static const TypeInfo mt7621_spi_info = {
    .name          = TYPE_MT7621_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT7621SPIState),
    .instance_init = mt7621_spi_init,
    .class_init    = mt7621_spi_class_init,
};

static void mt7621_spi_register_types(void)
{
    type_register_static(&mt7621_spi_info);
}
type_init(mt7621_spi_register_types)
