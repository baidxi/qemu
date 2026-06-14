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
 *   0x18  SPI_DATA4   (RW) TX/RX data word 5
 *   0x1C  SPI_DATA5   (RW) TX/RX data word 6
 *   0x20  SPI_DATA6   (RW) TX/RX data word 7
 *   0x24  SPI_DATA7   (RW) TX/RX data word 8
 *   0x28  SPI_MASTER  (RW) clock divider[27:16], slave_sel[31:29]
 *   0x2C  SPI_MOREBUF (RW) TX count[31:24]/[7:0], RX count[19:12] (in bits)
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
#define SPI_CMD_EN4B             0xB7  /* Enter 4-byte address mode */
#define SPI_CMD_EX4B             0xE9  /* Exit 4-byte address mode */

/* ---- Helpers ---- */

/*
 * Get the command byte from a single OPCODE write.  Small commands (JEDEC ID,
 * Read Status, etc.) are written with the command in byte 0; some MSB-first
 * packings put it in the top byte.  Prefer byte 0 and fall back to the top
 * byte only when byte 0 is zero.
 */
static uint8_t spi_get_cmd(uint32_t opcode)
{
    uint8_t cmd = opcode & 0xFF;
    if (cmd == 0) {
        cmd = (opcode >> 24) & 0xFF;
    }
    return cmd;
}

/*
 * MOREBUF bit-field accessors.  CMD_CNT[29:24], MISO_CNT[20:12] and
 * MOSI_CNT[8:0] all store *bit* counts; divide by 8 for byte counts.  These
 * overlap with the legacy layout for the values the drivers actually emit, so
 * a single decoder handles both the old (vanilla) and new (OpenWrt) drivers.
 */
static int spi_morebuf_cmd_bytes(uint32_t mb)
{
    return ((mb >> 24) & 0x3f) / 8;
}

static int spi_morebuf_rx_bytes(uint32_t mb)
{
    return ((mb >> 12) & 0x1ff) / 8;
}

/*
 * Decode the flash address for the current transfer.
 *
 * The Linux spi-mt7621 half-duplex driver in 4-byte-address mode splits the
 * flash address across two registers: OPCODE holds address >> 8 and the low
 * byte of DATA0 holds address & 0xff.  The command byte is delivered by the
 * first OPCODE write (captured in first_opcode) and is NOT present in the final
 * OPCODE value, e.g.
 *   addr 0x00050000 -> OPCODE 0x00000500, DATA0 0x00
 *   addr 0x00359b78 -> OPCODE 0x0000359b, DATA0 0x78
 */
static uint32_t spi_decode_addr(MT7621SPIState *s)
{
    return (s->opcode << 8) | (s->data_regs[0] & 0xFF);
}

/*
 * Winbond W25Q SPI-NOR JEDEC ID: manufacturer 0xEF, type 0x40, capacity.
 * The capacity byte is log2(flash size in bytes), e.g.
 *   4 MB -> 0x16 (W25Q32), 8 MB -> 0x17 (W25Q64),
 *   16 MB -> 0x18 (W25Q128), 32 MB -> 0x19 (W25Q256).
 *
 * This MUST match the actual pflash size (s->flash_size, which the machine
 * model aligns to a power-of-2 MiB).  Hard-coding one capacity (e.g. always
 * W25Q128/16 MB) makes the kernel's spi-nor driver size the device wrong, so
 * MTD partitions that extend to the real flash end (e.g. "firmware" on a
 * 32 MB dump) get truncated and the rootfs can no longer be found.
 */
static uint8_t spi_w25q_capacity(hwaddr flash_size)
{
    uint8_t cap = 0;

    g_assert(flash_size >= 1);
    while (flash_size > 1) {
        flash_size >>= 1;
        cap++;
    }
    return cap;
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
    case 0x18:  /* SPI_DATA4: RX bytes 16-19 */
        return s->rx_fifo[16] | (s->rx_fifo[17] << 8) |
               (s->rx_fifo[18] << 16) | (s->rx_fifo[19] << 24);
    case 0x1C:  /* SPI_DATA5: RX bytes 20-23 */
        return s->rx_fifo[20] | (s->rx_fifo[21] << 8) |
               (s->rx_fifo[22] << 16) | (s->rx_fifo[23] << 24);
    case 0x20:  /* SPI_DATA6: RX bytes 24-27 */
        return s->rx_fifo[24] | (s->rx_fifo[25] << 8) |
               (s->rx_fifo[26] << 16) | (s->rx_fifo[27] << 24);
    case 0x24:  /* SPI_DATA7: RX bytes 28-31 */
        return s->rx_fifo[28] | (s->rx_fifo[29] << 8) |
               (s->rx_fifo[30] << 16) | (s->rx_fifo[31] << 24);
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
    int cmd_bytes = spi_morebuf_cmd_bytes(s->more_buf);
    uint8_t cmd;
    uint32_t addr;

    /* Clear RX FIFO for each command */
    memset(s->rx_fifo, 0, sizeof(s->rx_fifo));

    /*
     * Decode the command byte.  A transfer whose MOREBUF command-phase count
     * is non-zero starts a brand-new SPI transaction; otherwise it is an
     * auto-increment continuation that reuses the command from the previous
     * transfer (cur_cmd).
     *
     * Two OPCODE-write styles reach this point:
     *  - Linux spi-mt7621 READ writes OPCODE twice: the command byte first
     *    (captured in first_opcode), then the little-endian address which
     *    overwrites OPCODE.  opcode_write_count >= 2 in that case.
     *  - U-Boot / single-write style packs command + address MSB-first into
     *    one OPCODE word, so the command is the top byte.
     */
    if (cmd_bytes > 0) {
        if (s->opcode_write_count >= 2) {
            /* Linux 2-write read: command byte survives in first_opcode. */
            s->cur_cmd = s->first_opcode & 0xFF;
        } else {
            /* Single OPCODE write (JEDEC, status, erase/program): command
             * is byte 0 of the OPCODE word. */
            s->cur_cmd = spi_get_cmd(s->opcode);
        }
    }
    cmd = s->cur_cmd;

    switch (cmd) {
    /* ---- Identification commands ---- */
    case SPI_CMD_JEDEC_ID:
        /* Winbond W25Q: EF 40 <capacity>, capacity tracks flash_size */
        s->rx_fifo[0] = 0xEF;
        s->rx_fifo[1] = 0x40;
        s->rx_fifo[2] = spi_w25q_capacity(s->flash_size);
        break;

    case SPI_CMD_READ_MFG_ID:
        s->rx_fifo[0] = 0xEF;
        s->rx_fifo[1] = spi_w25q_capacity(s->flash_size);
        break;

    case SPI_CMD_RELEASE_PD:
        s->rx_fifo[0] = 0xEF;
        s->rx_fifo[1] = 0x40;
        s->rx_fifo[2] = spi_w25q_capacity(s->flash_size);
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
            int rx_bytes = spi_morebuf_rx_bytes(s->more_buf);
            if (rx_bytes <= 0 || rx_bytes > 32) {
                rx_bytes = 32;
            }
            if (cmd_bytes > 0) {
                /* Fresh command phase: decode the start address. */
                s->read_addr = spi_decode_addr(s);
            }
            /* cmd_bytes == 0: auto-increment continuation of the prior read,
             * which keeps the read_addr pointer advanced from last time. */
            for (int i = 0; i < rx_bytes; i++) {
                if ((uint64_t)s->read_addr + i < s->flash_size) {
                    s->rx_fifo[i] = s->flash_data[s->read_addr + i];
                }
            }
            /*
             * Advance the auto-increment pointer only on the TRANS START path
             * so a continuation transfer resumes right after the previous one.
             */
            if (allow_write) {
                s->read_addr += rx_bytes;
            }
        }
        break;

    /* ---- Page Program ---- */
    case SPI_CMD_PAGE_PROGRAM:
        if (allow_write && s->write_enable && s->flash_data) {
            int tx_count;
            uint8_t *tx_buf = (uint8_t *)s->data_regs;
            addr = spi_decode_addr(s);
            tx_count = (s->more_buf >> 24) & 0xFF;
            if (tx_count == 0) {
                tx_count = 4;
            }
            if (tx_count > 32) {
                tx_count = 32;
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
            addr = spi_decode_addr(s) & ~0xFFFu;
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
            addr = spi_decode_addr(s) & ~0x7FFFu;
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
            addr = spi_decode_addr(s) & ~0xFFFFu;
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

    case SPI_CMD_EN4B:
        /* Enter 4-byte address mode.  No data phase; just track the mode. */
        s->addr_4byte = true;
        break;

    case SPI_CMD_EX4B:
        /* Exit 4-byte address mode.  No data phase; just track the mode. */
        s->addr_4byte = false;
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
             * Process the complete transfer.  At this point OPCODE (possibly
             * written twice: command then address), DATA registers and MOREBUF
             * have all been written, so this is the authoritative decode point.
             */
            spi_process_cmd(s, true);
            /*
             * Reset the OPCODE-write counter so a follow-up transfer in the
             * same chip-select window is treated as an auto-increment
             * continuation unless the driver writes OPCODE again.
             */
            s->opcode_write_count = 0;
            s->busy = false;  /* instant completion */
        }
        break;

    case 0x04:  /* SPI_OPCODE: TX word 0 (command and/or address) */
        s->opcode = val;
        if (s->opcode_write_count == 0) {
            /* First OPCODE write of this chip-select window: remember it so
             * the command byte survives a second (address) OPCODE write. */
            s->first_opcode = val;
        }
        s->opcode_write_count++;
        /*
         * All command processing is deferred to TRANS START so that both
         * OPCODE writes (command, then address) of the Linux half-duplex read
         * are visible when the command and address are decoded.
         */
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
    case 0x18:  /* SPI_DATA4 */
        s->data_regs[4] = val;
        break;
    case 0x1C:  /* SPI_DATA5 */
        s->data_regs[5] = val;
        break;
    case 0x20:  /* SPI_DATA6 */
        s->data_regs[6] = val;
        break;
    case 0x24:  /* SPI_DATA7 */
        s->data_regs[7] = val;
        break;

    case 0x28:  /* SPI_MASTER */
        s->master_cfg = val;
        break;

    case 0x2C:  /* SPI_MOREBUF: TX count [31:24], RX count [15:8] */
        s->more_buf = val;
        break;

    case 0x38:  /* SPI_POLAR (CS) */
        s->cs_polar = val;
        /* Chip-select boundary: start counting OPCODE writes afresh. */
        s->opcode_write_count = 0;
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
