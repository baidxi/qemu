/*
 * MT7621 System Controller (SYSCTL) device header
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#ifndef HW_MISC_MT7621_SYSCTL_H
#define HW_MISC_MT7621_SYSCTL_H

#include "hw/core/sysbus.h"

#define TYPE_MT7621_SYSCTL   "mt7621-sysctl"

struct MT7621SysctlState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[0x100 / 4];   /* 256 bytes (REG_COUNT) */
};

OBJECT_DECLARE_SIMPLE_TYPE(MT7621SysctlState, MT7621_SYSCTL)

#endif
