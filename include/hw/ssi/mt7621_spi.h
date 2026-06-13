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
    uint32_t data_regs[4];  /* SPI_DATA0-3 (0x08-0x14): TX/RX words */
    uint32_t more_buf;      /* SPI_MOREBUF (0x2C): TX/RX byte counts */
    uint32_t master_cfg;    /* SPI_MASTER (0x28) */
    uint32_t cs_polar;      /* SPI_POLAR (0x38) */

    /* RX FIFO (up to 16 bytes per transfer) */
    uint8_t rx_fifo[16];
    int rx_len;

    /* SPI NOR flash backing (shared with machine) */
    uint8_t *flash_data;    /* Pointer to flash RAM buffer */
    hwaddr flash_size;      /* Size of flash buffer */
    BlockBackend *flash_blk; /* BlockBackend for real-time persistence (NULL if no pflash) */
    bool write_enable;      /* SPI NOR WEL (Write Enable Latch) */
};

OBJECT_DECLARE_SIMPLE_TYPE(MT7621SPIState, MT7621_SPI)

#endif
