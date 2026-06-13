/*
 * MT7621 Ethernet Frame Engine device header
 *
 * Register space: 0x40000 bytes (FE + GMACs + Switch)
 * All registers are stored in a flat array for read-back fidelity.
 */
#ifndef HW_NET_MT7621_ETH_H
#define HW_NET_MT7621_ETH_H

#include "hw/core/sysbus.h"

#define TYPE_MT7621_ETH   "mt7621-eth"

#define MT7621_ETH_SIZE   0x40000  /* 256 KB covers FE + GMACs + switch */

struct MT7621EthState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[MT7621_ETH_SIZE / 4];  /* register backing store */
};

OBJECT_DECLARE_SIMPLE_TYPE(MT7621EthState, MT7621_ETH)

#endif
