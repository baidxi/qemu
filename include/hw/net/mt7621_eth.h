/*
 * MT7621 Ethernet Frame Engine device header
 *
 * Register space: 0x40000 bytes (FE + GMACs + Switch)
 * All registers are stored in a flat array for read-back fidelity.
 *
 * This device emulates the Ralink/MTK PDMA (Packet DMA) ring engine
 * and connects to the QEMU net subsystem for host communication.
 */
#ifndef HW_NET_MT7621_ETH_H
#define HW_NET_MT7621_ETH_H

#include "hw/core/sysbus.h"
#include "net/net.h"
#include "qemu/timer.h"

#define TYPE_MT7621_ETH   "mt7621-eth"

#define MT7621_ETH_SIZE   0x40000  /* 256 KB covers FE + GMACs + switch */

struct MT7621EthState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;                         /* interrupt output to GIC IRQ 3 */

    /* Register backing store (preserved for read-back fidelity) */
    uint32_t regs[MT7621_ETH_SIZE / 4];

    /* QEMU net subsystem */
    NICState *nic;
    NICConf conf;

    /*
     * Poll-mode RX buffer.  Incoming packets are buffered here and only
     * written to the RX descriptor ring when the guest reads an ETH
     * register (i.e. actively polls).  This prevents the ring from being
     * filled before the guest's polling loop starts, which would cause
     * DRX_IDX to wrap and appear empty.
     */
    uint8_t rx_pending_buf[1600];
    size_t  rx_pending_len;

    /*
     * Set to true when the CPU writes RX_CRX_IDX0 AFTER at least one
     * RX packet has been received (i.e. the CPU has genuinely consumed
     * descriptors, not just set the initial ring-empty value).  Used
     * to validate the (next_drx == crx) ring-full heuristic.
     */
    bool crx_committed;

    /*
     * Timestamp (ns, virtual clock) of the last RX IRQ falling edge.
     * Used to enforce a minimum gap between RX interrupt assertions,
     * preventing interrupt storms that can starve Breed's handler.
     */
    int64_t rx_irq_down_ns;
    uint32_t rx_stall_count;
    struct QEMUTimer *rx_dma_timer;      /* DMA transfer delay (~20µs) */
    struct QEMUTimer *rx_keepalive_timer; /* 50ms TX keepalive */
};

OBJECT_DECLARE_SIMPLE_TYPE(MT7621EthState, MT7621_ETH)

#endif
