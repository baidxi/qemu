/*
 * MT7621 SPI Controller device header
 */
#ifndef HW_SSI_MT7621_SPI_H
#define HW_SSI_MT7621_SPI_H

#include "hw/core/sysbus.h"
#include "hw/ssi/ssi.h"

#define TYPE_MT7621_SPI   "mt7621-spi"

struct MT7621SPIState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    bool busy;
    uint32_t ctrl;
    uint32_t master_cfg;
    uint32_t cs_polar;
    /* JEDEC ID response */
    uint8_t rx_fifo[4];
};

OBJECT_DECLARE_SIMPLE_TYPE(MT7621SPIState, MT7621_SPI)

#endif
