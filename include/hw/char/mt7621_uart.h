/*
 * MT7621 / RT2880 UART device header
 */
#ifndef HW_CHAR_MT7621_UART_H
#define HW_CHAR_MT7621_UART_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"

#define TYPE_MT7621_UART   "mt7621-uart"

struct MT7621UartState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    Chardev *chr;
    uint8_t rx;
    uint8_t ier, lcr, mcr;
    uint8_t dll;
};

OBJECT_DECLARE_SIMPLE_TYPE(MT7621UartState, MT7621_UART)

#endif
