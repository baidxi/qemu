/*
 * MT7621 GPIO Controller device header
 *
 * Register layout (based on Linux driver gpio-mt7621.c):
 *   BANK_STRIDE = 0x04 (banks are at consecutive 4-byte offsets)
 *   Register types at 0x10 intervals:
 *     0x00 + bank*4  CTRL   (RW) Pin direction (1=output)
 *     0x10 + bank*4  POL    (RW) Interrupt polarity
 *     0x20 + bank*4  DATA   (RW) Pin data value
 *     0x30 + bank*4  DSET   (WO) Data set (write-1 to set bit)
 *     0x40 + bank*4  DCLR   (WO) Data clear (write-1 to clear bit)
 *     0x50 + bank*4  REDGE  (RW) Rising edge detect enable
 *     0x60 + bank*4  FEDGE  (RW) Falling edge detect enable
 *     0x70 + bank*4  HLVL   (RW) High level detect enable
 *     0x80 + bank*4  LLVL   (RW) Low level detect enable
 *     0x90 + bank*4  STAT   (RW) Interrupt status (W1C)
 *     0xA0 + bank*4  EDGE   (RW) Edge detect
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#ifndef HW_GPIO_MT7621_GPIO_H
#define HW_GPIO_MT7621_GPIO_H

#include "hw/core/sysbus.h"

#define TYPE_MT7621_GPIO   "mt7621-gpio"

#define MT7621_GPIO_BANKS  3
#define MT7621_GPIO_PINS   96
#define MT7621_GPIO_SIZE   0x100

typedef struct MT7621GPIOBank {
    uint32_t ctrl;      /* direction: 1=output (GPIO_REG_CTRL @ 0x00) */
    uint32_t pol;       /* interrupt polarity (GPIO_REG_POL @ 0x10) */
    uint32_t data;      /* pin data (GPIO_REG_DATA @ 0x20) */
    uint32_t redge;     /* rising edge detect (GPIO_REG_REDGE @ 0x50) */
    uint32_t fedge;     /* falling edge detect (GPIO_REG_FEDGE @ 0x60) */
    uint32_t hlvl;      /* high level detect (GPIO_REG_HLVL @ 0x70) */
    uint32_t llvl;      /* low level detect (GPIO_REG_LLVL @ 0x80) */
    uint32_t stat;      /* interrupt status W1C (GPIO_REG_STAT @ 0x90) */
    uint32_t edge;      /* edge detect (GPIO_REG_EDGE @ 0xA0) */
} MT7621GPIOBank;

struct MT7621GPIOState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;                       /* Aggregated → GIC 12 */
    MT7621GPIOBank bank[MT7621_GPIO_BANKS];
};

OBJECT_DECLARE_SIMPLE_TYPE(MT7621GPIOState, MT7621_GPIO)

#endif
