/*
 * MT7621 SPI Controller device header
 */
#ifndef HW_SSI_MT7621_SPI_H
#define HW_SSI_MT7621_SPI_H

#include "hw/core/sysbus.h"
#include "hw/ssi/ssi.h"
#include "system/block-backend.h"

#define TYPE_MT7621_SPI   "mt7621-spi"

struct MT7621SPIState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    /* Register state */
    bool busy;
    uint32_t ctrl;
    uint32_t opcode;        /* SPI_OPCODE (0x04): first TX word */
    uint32_t data_regs[8];  /* SPI_DATA0-7 (0x08-0x24): TX/RX words */
    uint32_t more_buf;      /* SPI_MOREBUF (0x2C): TX/RX byte counts */
    uint32_t master_cfg;    /* SPI_MASTER (0x28) */
    uint32_t cs_polar;      /* SPI_POLAR (0x38) */

    /*
     * RX FIFO (up to 32 bytes per half-duplex transfer).  The MT7621
     * half-duplex driver reads up to 32 bytes per transaction from DATA0-7
     * (see linux-6.6.77/drivers/spi/spi-mt7621.c).
     *
     * The Linux spi-mt7621 driver issues a flash read as SEVERAL register
     * writes before TRANS START, e.g. a READ_DATA (0x03) of a 4-byte address:
     *
     *     OPCODE  <- 0x00000003      (command byte, in byte 0)
     *     OPCODE  <- 0x00050000 bswap'd   (4-byte address, overwrites cmd!)
     *     DATA0.. <- write data (if any)
     *     MOREBUF <- bit counts  (CMD_CNT=32, MISO_CNT, MOSI_CNT)
     *     TRANS   <- START
     *
     * Because the second OPCODE write OVERWRITES the command byte, the command
     * must be captured from the FIRST OPCODE write after chip-select assert
     * (first_opcode / opcode_write_count).  A follow-up transfer within the
     * same chip-select window that has no command phase (MOREBUF CMD_CNT == 0)
     * is an auto-increment continuation: it keeps reading from read_addr.
     */
    uint8_t rx_fifo[32];
    hwaddr read_addr;        /* SPI-NOR auto-increment read pointer */
    uint8_t cur_cmd;         /* command byte of the transfer being executed */
    uint32_t first_opcode;   /* value of the first OPCODE write after CS assert */
    int opcode_write_count;  /* #OPCODE writes since CS assert / TRANS START */

    /* SPI NOR flash backing (shared with machine) */
    uint8_t *flash_data;    /* Pointer to flash RAM buffer */
    hwaddr flash_size;      /* Size of flash buffer */
    BlockBackend *flash_blk; /* BlockBackend for real-time persistence (NULL if no pflash) */
    bool write_enable;      /* SPI NOR WEL (Write Enable Latch) */
    bool addr_4byte;        /* SPI NOR 4-byte address mode (EN4B/EX4B) */
};

OBJECT_DECLARE_SIMPLE_TYPE(MT7621SPIState, MT7621_SPI)

#endif
