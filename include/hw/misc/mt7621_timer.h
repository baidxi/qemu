/*
 * MT7621 Timer / Watchdog device header
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#ifndef HW_MISC_MT7621_TIMER_H
#define HW_MISC_MT7621_TIMER_H

#include "hw/core/sysbus.h"
#include "hw/core/ptimer.h"

#define TYPE_MT7621_TIMER   "mt7621-timer"

typedef struct MT7621TimerChannel {
    ptimer_state *ptimer;
    qemu_irq irq;
    uint32_t load;
    uint32_t ctrl;
    uint32_t clk_freq;
    bool int_pending;
} MT7621TimerChannel;

struct MT7621TimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MT7621TimerChannel channel[2];
};

OBJECT_DECLARE_SIMPLE_TYPE(MT7621TimerState, MT7621_TIMER)

#endif
