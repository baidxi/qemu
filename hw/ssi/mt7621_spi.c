/*
 * MT7621 SPI Controller — register-accurate from Linux spi-mt7621.c
 *
 * Supports SPI NOR flash commands via the MT7621 SPI register interface.
 * The flash data buffer is shared with the machine model (mt7621.c)
 * for memory-mapped access and persistence.
 *
 * Register map:
 *   0x00  SPI_TRANS   (RW) bit16=BUSY, bit8=START, [7:0]=byte count
 *   0x04  SPI_OPCODE  (W)  TX data word 0 (command + address)
 *   0x08  SPI_DATA0   (RW) TX/RX data word 1
 *   0x0C  SPI_DATA1   (RW) TX/RX data word 2
 *   0x10  SPI_DATA2   (RW) TX/RX data word 3
 *   0x14  SPI_DATA3   (RW) TX/RX data word 4
 *   0x28  SPI_MASTER  (RW) clock divider[27:16], slave_sel[31:29]
 *   0x2C  SPI_MOREBUF (RW) TX count[31:24], RX count[15:8]
 *   0x38  SPI_POLAR   (RW) CS polarity
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/ssi/mt7621_spi.h"
#include "system/block-backend-io.h"

/* ---- SPI NOR flash commands ---- */
#define SPI_CMD_WRITE_ENABLE     0x06
#define SPI_CMD_WRITE_DISABLE    0x04
#define SPI_CMD_READ_DATA        0x03
#define SPI_CMD_FAST_READ        0x0B
#define SPI_CMD_READ_STATUS      0x05
#define SPI_CMD_READ_STATUS2     0x35
#define SPI_CMD_WRITE_STATUS     0x01
#define SPI_CMD_PAGE_PROGRAM     0x02
#define SPI_CMD_SECTOR_ERASE     0x20  /* 4 KB */
#define SPI_CMD_BLOCK_ERASE_32K  0x52
#define SPI_CMD_BLOCK_ERASE_64K  0xD8
#define SPI_CMD_CHIP_ERASE       0xC7
#define SPI_CMD_JEDEC_ID         0x9F
#define SPI_CMD_READ_MFG_ID      0x90
#define SPI_CMD_RELEASE_PD       0xAB
#define SPI_CMD_READ_ES          0x9E

/* ---- Helpers ---- */

/* Get the command byte from the OPCODE register value */
static uint8_t spi_get_cmd(uint32_t opcode)
{
    uint8_t cmd = opcode & 0xFF;
    if (cmd == 0) {
        cmd = (opcode >> 24) & 0xFF;  /* try bswap32 order */
    }
    return cmd;
}

/* Extract 24-bit flash address from OPCODE register */
static uint32_t spi_extract_addr(uint32_t opcode)
{
    if (opcode & 0xFF) {
        /* Command in byte 0 (little-endian packing) */
        return ((opcode >> 8) & 0xFF) << 16 |
               ((opcode >> 16) & 0xFF) << 8 |
               ((opcode >> 24) & 0xFF);
    } else {
        /* Command in byte 3 (big-endian / bswap32 packing) */
        return ((opcode >> 16) & 0xFF) << 16 |
               ((opcode >> 8) & 0xFF) << 8 |
               (opcode & 0xFF);
    }
}

/*
 * Persist a flash region to the backing BlockBackend immediately.
 * Called after every SPI NOR write command (Page Program, Erase) so
 * that flash data survives VM exit without relying on an exit notifier
 * (which fires after the block subsystem has already shut down).
 */
static void spi_flash_persist(MT7621SPIState *s, hwaddr offset, hwaddr len)
{
    if (s->flash_blk) {
        int ret = blk_pwrite(s->flash_blk, offset, len,
                             s->flash_data + offset, 0);
        if (ret < 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mt7621_spi: failed to persist flash at 0x%llx "
                          "(%d)\n", (unsigned long long)offset, ret);
        }
    }
}

/* ---- Read handler ---- */
static uint64_t mt7621_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    MT7621SPIState *s = MT7621_SPI(opaque);

    switch (offset) {
    case 0x00:  /* SPI_TRANS */
        return (s->busy ? (1 << 16) : 0) | s->ctrl;
    case 0x08:  /* SPI_DATA0: RX bytes 0-3 */
        return s->rx_fifo[0]  | (s->rx_fifo[1]  << 8) |
               (s->rx_fifo[2] << 16) | (s->rx_fifo[3]  << 24);
    case 0x0C:  /* SPI_DATA1: RX bytes 4-7 */
        return s->rx_fifo[4]  | (s->rx_fifo[5]  << 8) |
               (s->rx_fifo[6] << 16) | (s->rx_fifo[7]  << 24);
    case 0x10:  /* SPI_DATA2: RX bytes 8-11 */
        return s->rx_fifo[8]  | (s->rx_fifo[9]  << 8) |
               (s->rx_fifo[10] << 16) | (s->rx_fifo[11] << 24);
    case 0x14:  /* SPI_DATA3: RX bytes 12-15 */
        return s->rx_fifo[12] | (s->rx_fifo[13] << 8) |
               (s->rx_fifo[14] << 16) | (s->rx_fifo[15] << 24);
    case 0x28:  /* SPI_MASTER */
        return s->master_cfg;
    case 0x2C:  /* SPI_MOREBUF */
        return s->more_buf;
    case 0x38:  /* SPI_POLAR */
        return s->cs_polar;
    default:
        return 0;
    }
}

/*
 * Process SPI NOR flash command.
 * @allow_write: if false, skip commands that modify flash (Page Program,
 *               Erase) because data registers may not be valid yet.
 *               Set true when processing on TRANS START.
 */
static void spi_process_cmd(MT7621SPIState *s, bool allow_write)
{
    uint8_t cmd = spi_get_cmd(s->opcode);
    uint32_t addr;

    /* Clear RX FIFO for each command */
    memset(s->rx_fifo, 0, sizeof(s->rx_fifo));

    switch (cmd) {
    /* ---- Identification commands ---- */
    case SPI_CMD_JEDEC_ID:
        /* Winbond W25Q128: EF 40 18 (16 MB) */
        s->rx_fifo[0] = 0xEF;
        s->rx_fifo[1] = 0x40;
        s->rx_fifo[2] = 0x18;
        break;

    case SPI_CMD_READ_MFG_ID:
        s->rx_fifo[0] = 0xEF;
        s->rx_fifo[1] = 0x18;
        break;

    case SPI_CMD_RELEASE_PD:
        s->rx_fifo[0] = 0xEF;
        s->rx_fifo[1] = 0x40;
        s->rx_fifo[2] = 0x18;
        break;

    case SPI_CMD_READ_ES:
        s->rx_fifo[0] = 0x16;  /* 1.8V */
        break;

    /* ---- Status register commands ---- */
    case SPI_CMD_READ_STATUS:
        /* Bit 0 = BUSY (always 0), Bit 1 = WEL */
        s->rx_fifo[0] = s->write_enable ? 0x02 : 0x00;
        break;

    case SPI_CMD_READ_STATUS2:
        s->rx_fifo[0] = 0x00;
        break;

    /* ---- Write control commands ---- */
    case SPI_CMD_WRITE_ENABLE:
        s->write_enable = true;
        break;

    case SPI_CMD_WRITE_DISABLE:
        s->write_enable = false;
        break;

    /* ---- Read Data ---- */
    case SPI_CMD_READ_DATA:
    case SPI_CMD_FAST_READ:
        if (s->flash_data) {
            int rx_count;
            addr = spi_extract_addr(s->opcode);
            /* RX count from MOREBUF [15:8] */
            rx_count = (s->more_buf >> 8) & 0xFF;
            if (rx_count == 0 || rx_count > 16) {
                rx_count = 16;
            }
            for (int i = 0; i < rx_count; i++) {
                if ((uint64_t)addr + i < s->flash_size) {
                    s->rx_fifo[i] = s->flash_data[addr + i];
                }
            }
        }
        break;

    /* ---- Page Program ---- */
    case SPI_CMD_PAGE_PROGRAM:
        if (allow_write && s->write_enable && s->flash_data) {
            int tx_count;
            uint8_t *tx_buf = (uint8_t *)s->data_regs;
            addr = spi_extract_addr(s->opcode);
            tx_count = (s->more_buf >> 24) & 0xFF;
            if (tx_count == 0) {
                tx_count = 4;
            }
            if (tx_count > 16) {
                tx_count = 16;
            }
            /* NOR flash: program = AND with existing data */
            for (int i = 0; i < tx_count; i++) {
                uint32_t off = addr + i;
                if (off < s->flash_size) {
                    s->flash_data[off] &= tx_buf[i];
                }
            }
            spi_flash_persist(s, addr, tx_count);
            s->write_enable = false;
        }
        break;

    /* ---- Erase commands ---- */
    case SPI_CMD_SECTOR_ERASE:  /* 4 KB */
        if (allow_write && s->write_enable && s->flash_data) {
            addr = spi_extract_addr(s->opcode) & ~0xFFFu;
            for (uint32_t i = 0; i < 0x1000; i++) {
                if ((uint64_t)addr + i < s->flash_size) {
                    s->flash_data[addr + i] = 0xFF;
                }
            }
            spi_flash_persist(s, addr, 0x1000);
            s->write_enable = false;
        }
        break;

    case SPI_CMD_BLOCK_ERASE_32K:
        if (allow_write && s->write_enable && s->flash_data) {
            addr = spi_extract_addr(s->opcode) & ~0x7FFFu;
            for (uint32_t i = 0; i < 0x8000; i++) {
                if ((uint64_t)addr + i < s->flash_size) {
                    s->flash_data[addr + i] = 0xFF;
                }
            }
            spi_flash_persist(s, addr, 0x8000);
            s->write_enable = false;
        }
        break;

    case SPI_CMD_BLOCK_ERASE_64K:
        if (allow_write && s->write_enable && s->flash_data) {
            addr = spi_extract_addr(s->opcode) & ~0xFFFFu;
            for (uint32_t i =  0; i < 0x10000; i++) {
                if ((uint64_t)addr + i < s->flash_size) {
                    s->flash_data[addr + i] = 0xFF;
                }
            }
            spi_flash_persist(s, addr, 0x10000);
            s->write_enable = false;
        }
        break;

    case SPI_CMD_CHIP_ERASE:
        if (allow_write && s->write_enable && s->flash_data) {
            memset(s->flash_data, 0xFF, s->flash_size);
            spi_flash_persist(s, 0, s->flash_size);
            s->write_enable = false;
        }
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "mt7621_spi: unhandled command 0x%02x\n", cmd);
        break;
    }
}

/* ---- Write handler ---- */
static void mt7621_spi_write(void *opaque, hwaddr offset,
                             uint64_t val, unsigned size)
{
    MT7621SPIState *s = MT7621_SPI(opaque);

    switch (offset) {
    case 0x00:  /* SPI_TRANS: bit8=START, bit16=BUSY */
        s->ctrl = val;
        if (val & (1 << 8)) {  /* START */
            s->busy = true;
            /*
             * Process the complete transfer.  At this point, OPCODE,
             * DATA registers, and MOREBUF have all been written.
             */
            spi_process_cmd(s, true);
            s->busy = false;  /* instant completion */
        }
        break;

    case 0x04:  /* SPI_OPCODE: first TX word (command + address) */
        s->opcode = val;
        /*
         * Process read commands immediately for backward compatibility:
         * Breed writes the command byte and reads the response without
         * explicitly triggering START.  Write commands (Page Program,
         * Erase) are deferred to the TRANS START handler.
         */
        spi_process_cmd(s, false);
        break;

    case 0x08:  /* SPI_DATA0 */
        s->data_regs[0] = val;
        break;
    case 0x0C:  /* SPI_DATA1 */
        s->data_regs[1] = val;
        break;
    case 0x10:  /* SPI_DATA2 */
        s->data_regs[2] = val;
        break;
    case 0x14:  /* SPI_DATA3 */
        s->data_regs[3] = val;
        break;

    case 0x28:  /* SPI_MASTER */
        s->master_cfg = val;
        break;

    case 0x2C:  /* SPI_MOREBUF: TX count [31:24], RX count [15:8] */
        s->more_buf = val;
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
