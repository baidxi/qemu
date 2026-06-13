/*
 * MT7621 Memory Controller device header
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#ifndef HW_MISC_MT7621_MEMCTRL_H
#define HW_MISC_MT7621_MEMCTRL_H

#include "hw/core/sysbus.h"

#define TYPE_MT7621_MEMCTRL   "mt7621-memctrl"

struct MT7621MemCtrlState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t dram_size;     /* set via property from machine->ram_size */
};

OBJECT_DECLARE_SIMPLE_TYPE(MT7621MemCtrlState, MT7621_MEMCTRL)

#endif
