/*
 * MT7621 Ethernet Frame Engine — PDMA ring emulation + QEMU net NIC
 *
 * Implements the Ralink/MTK Packet DMA (PDMA) ring engine used by
 * U-Boot, Breed and the legacy Ralink SDK ("rt2880-eth").  The device
 * connects to the QEMU net subsystem so that guest firmware can
 * exchange Ethernet frames with the host (slirp / tap / socket).
 *
 * PDMA ring 0 register layout (offsets from FE base 0x1E110000):
 *
 *   RX ring 0:  0x0900 RX_BASE_PTR0, 0x0904 RX_MAX_CNT0,
 *               0x0908 RX_CRX_IDX0 (CPU), 0x090C RX_DRX_IDX0 (DMA)
 *   TX ring 0:  0x0800 TX_BASE_PTR0, 0x0804 TX_MAX_CNT0,
 *               0x0808 TX_CTX_IDX0 (CPU, trigger), 0x080C TX_DTX_IDX0 (DMA)
 *   PDMA global: 0x0A04 GLO_CFG, 0x0A08 RST_IDX, 0x0A20 INT_STATUS,
 *                0x0A28 INT_MASK
 *
 * PDMA descriptors are 16 bytes (4 x 32-bit words), little-endian.
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/net/mt7621_eth.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "net/net.h"
#include "net/eth.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#define ETH_DEBUG 0

#if ETH_DEBUG
#define eth_dbg(fmt, ...) \
    fprintf(stderr, "mt7621_eth: " fmt, ## __VA_ARGS__)
#else
#define eth_dbg(fmt, ...) do { } while (0)
#endif

/* =====================================================================
 *  GSW (Gigabit Switch) offsets
 * ===================================================================== */
#define GSW_BASE_OFF      0x10000
#define GSW_CHIP_ID       0x1001C

/* =====================================================================
 *  PDMA register offsets (corrected per Linux mtk_reg_map)
 * ===================================================================== */

/* RX ring 0 */
#define PDMA_RX_BASE_PTR0   0x0900
#define PDMA_RX_MAX_CNT0    0x0904
#define PDMA_RX_CRX_IDX0    0x0908   /* CPU RX index */
#define PDMA_RX_DRX_IDX0    0x090C   /* DMA RX index (written by us) */

/* TX ring 0 */
#define PDMA_TX_BASE_PTR0   0x0800
#define PDMA_TX_MAX_CNT0    0x0804
#define PDMA_TX_CTX_IDX0    0x0808   /* CPU TX index (write = TX trigger) */
#define PDMA_TX_DTX_IDX0    0x080C   /* DMA TX index (written by us) */

/* PDMA global */
#define PDMA_GLO_CFG        0x0A04
#define PDMA_RST_IDX        0x0A08
#define PDMA_DELAY_INT      0x0A0C
#define PDMA_INT_STATUS     0x0A20
#define PDMA_INT_MASK       0x0A28

/* FE global interrupt (matches Linux MTK_INT_STATUS2=0x08 / MTK_FE_INT_ENABLE=0x0C) */
#define FE_INT_STATUS       0x0008
#define FE_INT_ENABLE       0x000C

/* FE core (matches Linux MTK_RST_GL=0x04) */
#define FE_RST_GL           0x0004

/* MDIO access is only through GSW at 0x10004 on MT7621 */
#define GSW_MDIO_ACCESS     (GSW_BASE_OFF + 0x04)   /* 0x10004 */

/* =====================================================================
 *  GLO_CFG bits
 * ===================================================================== */
#define GLO_CFG_TX_DMA_EN   0x00000001
#define GLO_CFG_RX_DMA_EN   0x00000004

/* =====================================================================
 *  Interrupt status bits
 * ===================================================================== */
#define INT_TX_DONE0        0x00000001
#define INT_RX_DONE0        0x00010000
#define INT_ALL_TX          (INT_TX_DONE0)
#define INT_ALL_RX          (INT_RX_DONE0)

/* =====================================================================
 *  RST_IDX bits
 * ===================================================================== */
#define RST_IDX_TX0         0x00000001
#define RST_IDX_RX0         0x00010000

/* =====================================================================
 *  PDMA descriptor bit fields  (word 1 of the 16-byte descriptor)
 *
 *  TX descriptor word 1 (txd2):
 *    [31]    DDONE   -- 1=CPU filled (ready for DMA), cleared by DMA after send
 *    [30]    LS0     -- last segment of buffer 0
 *    [29:16] PLEN0   -- buffer-0 data length (14 bits)
 *    [14]    LS1     -- last segment of buffer 1
 *    [13:0]  PLEN1   -- buffer-1 data length (14 bits)
 *
 *  RX descriptor word 1 (rxd2):
 *    [31]    DONE    -- 1=DMA received a packet (CPU can read)
 *    [30]    LSO     -- last segment
 *    [29:16] PLEN0   -- received packet length (14 bits)
 * ===================================================================== */
#define DESC_DDONE          0x80000000u
#define DESC_LS0            0x40000000u
#define DESC_GET_PLEN0(w)   (((w) >> 16) & 0x3FFF)
#define DESC_SET_PLEN0(len) (((uint32_t)(len) & 0x3FFF) << 16)
#define DESC_PLEN0_MASK     DESC_SET_PLEN0(0x3FFF)  /* bits [29:16] */

#define DESC_SIZE           16   /* bytes per PDMA descriptor */
#define MAX_PKT_SIZE        1600

/* =====================================================================
 *  MDIO / PHY helpers
 * ===================================================================== */
#define MDIO_START_BIT      (1u << 31)
#define MDIO_CMD_RD         (1u << 18)

static uint16_t mt7621_mdio_phy_read(uint32_t phy_addr, uint32_t reg_addr)
{
    switch (reg_addr) {
    case 0:   return 0x1140;  /* ANEG_EN | DUPLEX | SPEED100 */
    case 1:   return 0x796D;  /* LSTATUS | ANEGCOMP | 100FD | 1000FD */
    case 2:   return 0x0000;
    case 3:   return 0x0000;
    case 4:   return 0x01E1;  /* 100FD | 100HD | 10FD | 10HD */
    case 5:   return 0x45E1;
    case 9:   return 0x0300;  /* 1000FD advertised */
    default:  return 0x0000;
    }
}

/* =====================================================================
 *  PDMA descriptor DMA helpers
 * ===================================================================== */

/* Read one 16-byte descriptor (4 words) from guest physical memory. */
static bool pdma_desc_read(hwaddr addr, uint32_t w[4])
{
    uint8_t buf[DESC_SIZE];
    if (dma_memory_read(&address_space_memory, addr, buf, DESC_SIZE,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    for (int i = 0; i < 4; i++) {
        w[i] = ldl_le_p(buf + i * 4);
    }
    return true;
}

/* Write one 32-bit word of a descriptor back to guest memory. */
static bool pdma_desc_write_word(hwaddr desc_addr, int word_idx, uint32_t val)
{
    uint8_t buf[4];
    stl_le_p(buf, val);
    return dma_memory_write(&address_space_memory,
                            desc_addr + word_idx * 4, buf, 4,
                            MEMTXATTRS_UNSPECIFIED) == MEMTX_OK;
}

static inline uint32_t idx_next(uint32_t idx, uint32_t cnt)
{
    return cnt ? ((idx + 1) % cnt) : 0;
}

/*
 * Convert MIPS virtual (KSEG0/KSEG1) addresses to physical for DMA access.
 *
 * KSEG0 (0x80000000-0x9FFFFFFF) and KSEG1 (0xA0000000-0xBFFFFFFF) are
 * permanently mapped to physical 0x00000000-0x1FFFFFFF via bits [28:0].
 * Guest firmware stores KSEG0/KSEG1 addresses in DMA descriptor pointers;
 * the SoC DMA engine needs the physical equivalent.  Addresses already
 * in the physical range (< 0x80000000) pass through unchanged.
 */
static inline hwaddr mips_dma_addr(uint32_t addr)
{
    if (addr >= 0x80000000u && addr < 0xC0000000u) {
        return (hwaddr)(addr & 0x1FFFFFFFu);
    }
    return (hwaddr)addr;
}

/* =====================================================================
 *  Interrupt management
 * ===================================================================== */
static void mt7621_eth_update_irq(MT7621EthState *s)
{
    uint32_t pdma = s->regs[PDMA_INT_STATUS >> 2] &
                    s->regs[PDMA_INT_MASK >> 2];
    uint32_t fe   = s->regs[FE_INT_STATUS >> 2] &
                    s->regs[FE_INT_ENABLE >> 2];
    int level = (pdma | fe) ? 1 : 0;

    qemu_set_irq(s->irq, level);
}

static void mt7621_eth_try_flush(MT7621EthState *s);

static void mt7621_eth_dma_timer_cb(void *opaque)
{
    MT7621EthState *s = MT7621_ETH(opaque);
    mt7621_eth_try_flush(s);
}

static void mt7621_eth_set_int(MT7621EthState *s, uint32_t bits)
{
    s->regs[PDMA_INT_STATUS >> 2] |= bits;
    s->regs[FE_INT_STATUS >> 2]   |= bits;
    mt7621_eth_update_irq(s);
}

/* =====================================================================
 *  TX path: process descriptors when CPU advances TX_CTX_IDX0
 * ===================================================================== */
static void mt7621_eth_tx_kick(MT7621EthState *s)
{
    uint32_t tx_base = s->regs[PDMA_TX_BASE_PTR0 >> 2];
    hwaddr tx_base_phys = mips_dma_addr(tx_base);
    uint32_t tx_cnt  = s->regs[PDMA_TX_MAX_CNT0 >> 2];
    uint32_t ctx     = s->regs[PDMA_TX_CTX_IDX0 >> 2];
    uint32_t dtx     = s->regs[PDMA_TX_DTX_IDX0 >> 2];
    bool sent = false;

    eth_dbg("TX kick: base=0x%x cnt=%u ctx=%u dtx=%u glo=0x%x\n",
            tx_base, tx_cnt, ctx, dtx, s->regs[PDMA_GLO_CFG >> 2]);

    if (!tx_base || !tx_cnt) {
        eth_dbg("TX: no ring configured\n");
        return;
    }

    eth_dbg("TX kick: tx_base=0x%x -> phys=0x%llx cnt=%u ctx=%u dtx=%u\n",
            tx_base, (unsigned long long)tx_base_phys, tx_cnt, ctx, dtx);

    if (!(s->regs[PDMA_GLO_CFG >> 2] & GLO_CFG_TX_DMA_EN)) {
        eth_dbg("TX: TX_DMA not enabled (glo_cfg=0x%x)\n",
                s->regs[PDMA_GLO_CFG >> 2]);
        return;
    }

    while (dtx != ctx) {
        hwaddr desc_addr = tx_base_phys + (hwaddr)dtx * DESC_SIZE;
        uint32_t desc[4];
        uint8_t buf[MAX_PKT_SIZE];
        uint32_t plen;
        int len;

        if (!pdma_desc_read(desc_addr, desc)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mt7621_eth: TX desc read failed @0x%" HWADDR_PRIx
                          "\n", desc_addr);
            break;
        }

        plen = DESC_GET_PLEN0(desc[1]);
        len = plen > MAX_PKT_SIZE ? MAX_PKT_SIZE : plen;

        eth_dbg("TX desc[%u]: w0=0x%x w1=0x%x (ddone=%u ls0=%u plen=%u)\n",
                dtx, desc[0], desc[1],
                !!(desc[1] & DESC_DDONE), !!(desc[1] & DESC_LS0), plen);

        if (len > 0 && desc[0] != 0) {
            hwaddr buf_phys = mips_dma_addr(desc[0]);
            if (dma_memory_read(&address_space_memory, buf_phys, buf, len,
                                MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
                eth_dbg("TX: sending %d bytes from 0x%x (phys 0x%llx)\n",
                        len, desc[0], (unsigned long long)buf_phys);
                qemu_send_packet(qemu_get_queue(s->nic), buf, len);
                sent = true;
            } else {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "mt7621_eth: TX data read failed @0x%x len=%u\n",
                              desc[0], len);
            }
        }

        /*
         * Write-back: the PDMA has consumed this descriptor.  With
         * TX_WB_DDONE enabled (which the firmware sets during init), the
         * hardware writes the DDONE bit back to 1 so the driver can poll
         * for TX completion.  The driver WAITS for DDONE==1 before
         * reusing a descriptor: on the first lap through the TX ring it
         * finds DDONE=1 (left by descriptor init), but on the second lap
         * only our write-back can supply it.  Setting DDONE here is
         * therefore essential — clearing it (as a previous, incorrect
         * attempt did) makes the firmware spin forever waiting for TX
         * completion, freezing ping after one TX-ring-full of packets.
         * PLEN0 and the other bits are left untouched (the driver
         * rewrites them when reusing the descriptor).
         */
        pdma_desc_write_word(desc_addr, 1, desc[1] | DESC_DDONE);

        dtx = idx_next(dtx, tx_cnt);
    }

    s->regs[PDMA_TX_DTX_IDX0 >> 2] = dtx;

    if (sent) {
        mt7621_eth_set_int(s, INT_TX_DONE0);
    }
}

/* =====================================================================
 *  RX path: packet arrives from net backend -> fill RX descriptor
 * ===================================================================== */
static bool mt7621_eth_rx_ring_ready(MT7621EthState *s)
{
    if (!(s->regs[PDMA_GLO_CFG >> 2] & GLO_CFG_RX_DMA_EN)) {
        return false;
    }
    if (!s->regs[PDMA_RX_BASE_PTR0 >> 2] || !s->regs[PDMA_RX_MAX_CNT0 >> 2]) {
        return false;
    }
    return true;
}

/* Forward */
static bool mt7621_eth_do_rx(MT7621EthState *s, const uint8_t *buf, size_t size);
static void mt7621_eth_dump_rx_state(MT7621EthState *s, const char *tag);

/*
 * Flush any packets the net backend buffered while can_receive() was
 * returning false.  Also try to deliver the internally buffered packet.
 */
static void mt7621_eth_try_flush(MT7621EthState *s)
{
    if (!s->nic || !mt7621_eth_rx_ring_ready(s)) return;
    if (s->rx_pending_len) {
        if (mt7621_eth_do_rx(s, s->rx_pending_buf, s->rx_pending_len)) {
            s->rx_pending_len = 0;
            mt7621_eth_set_int(s, INT_RX_DONE0);

            /*
             * Backlog watchdog: if the firmware stops consuming RX (e.g.
             * hung inside TX send), pings pile up and DRX runs ahead of
             * CRX.  Dump the full TX+RX ring state once so we can see the
             * hang.  (Steady state is DRX==CRX+1; a startup burst may reach
             * CRX+2, so trigger at >=3.)
             */
            uint32_t drx = s->regs[PDMA_RX_DRX_IDX0 >> 2];
            uint32_t crx = s->regs[PDMA_RX_CRX_IDX0 >> 2];
            uint32_t cnt = s->regs[PDMA_RX_MAX_CNT0 >> 2];
            static bool dumped;
            if (cnt && !dumped) {
                uint32_t ahead = (drx + cnt - crx) % cnt;
                if (ahead >= 3) {
                    dumped = true;
                    mt7621_eth_dump_rx_state(s, "BACKLOG-WATCHDOG");
                }
            }
        }
    }
    qemu_flush_queued_packets(qemu_get_queue(s->nic));
}

/*
 * Core RX: write one packet into the DMA ring.  Returns true on success.
 * Does NOT set the interrupt — caller (receive path) does that.
 */
static bool mt7621_eth_do_rx(MT7621EthState *s, const uint8_t *buf, size_t size)
{
    uint32_t rx_base, rx_cnt, didx;
    hwaddr rx_base_phys, desc_addr, buf_phys;
    uint32_t desc[4];

    rx_base = s->regs[PDMA_RX_BASE_PTR0 >> 2];
    rx_base_phys = mips_dma_addr(rx_base);
    rx_cnt  = s->regs[PDMA_RX_MAX_CNT0 >> 2];
    didx    = s->regs[PDMA_RX_DRX_IDX0 >> 2];

    desc_addr = rx_base_phys + (hwaddr)didx * DESC_SIZE;
    if (!pdma_desc_read(desc_addr, desc) || (desc[1] & DESC_DDONE)) {
        return false;  /* ring full or read error */
    }

    buf_phys = mips_dma_addr(desc[0]);
    if (dma_memory_write(&address_space_memory, buf_phys, buf, size,
                         MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }

    desc[1] = DESC_DDONE | DESC_LS0 | DESC_SET_PLEN0(size);
    pdma_desc_write_word(desc_addr, 1, desc[1]);

    s->regs[PDMA_RX_DRX_IDX0 >> 2] = idx_next(didx, rx_cnt);

    return true;
}

/*
 * Receive callback: try DMA ring first, fall back to internal buffer.
 */
static ssize_t mt7621_eth_receive(NetClientState *nc, const uint8_t *buf,
                                    size_t size)
{
    MT7621EthState *s = qemu_get_nic_opaque(nc);

    if (!mt7621_eth_rx_ring_ready(s)) {
        return 0;
    }
    if (size > MAX_PKT_SIZE) {
        size = MAX_PKT_SIZE;
    }

    if (!s->rx_pending_len) {
        memcpy(s->rx_pending_buf, buf, size);
        s->rx_pending_len = size;
        timer_mod_ns(s->rx_dma_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
    }
    return size;
}

static bool mt7621_eth_can_receive(NetClientState *nc)
{
    MT7621EthState *s = qemu_get_nic_opaque(nc);

    if (!mt7621_eth_rx_ring_ready(s)) {
        return false;
    }

    /*
     * Return whether the single-packet RX buffer is free.  We must NOT
     * call try_flush() here: try_flush() invokes qemu_flush_queued_packets()
     * which calls can_receive() again, creating a recursive delivery chain
     * that fills multiple RX descriptors in one shot and sets RX_DONE
     * repeatedly — an interrupt storm that overwhelms firmware which
     * processes only one packet per interrupt invocation.  Instead, rely
     * on the 20 µs DMA timer and the CRX / RX_BASE_PTR0 write handlers
     * to pace delivery at one packet per timer fire.
     */
    return s->rx_pending_len == 0;
}

static void mt7621_eth_set_link(NetClientState *nc)
{
    /* Link status tracking -- PHY emulation handles status reads. */
}

/*
 * Diagnostic: dump the RX descriptor ring state.  Triggered when the
 * firmware appears stuck polling RX_DONE, to reveal any DRX/CRX or
 * descriptor DDONE desynchronisation.
 */
static void mt7621_eth_dump_rx_state(MT7621EthState *s, const char *tag)
{
    uint32_t base = s->regs[PDMA_RX_BASE_PTR0 >> 2];
    uint32_t cnt  = s->regs[PDMA_RX_MAX_CNT0 >> 2];
    uint32_t crx  = s->regs[PDMA_RX_CRX_IDX0 >> 2];
    hwaddr base_phys = mips_dma_addr(base);
    uint32_t i;

    if (cnt == 0 || cnt > 4096) {
        return;
    }
    for (i = 0; i < cnt && i < 16; i++) {
        uint32_t k = (crx + i) % cnt;
        uint32_t d[4];
        pdma_desc_read(base_phys + (hwaddr)k * DESC_SIZE, d);
    }
    /* TX ring snapshot */
    {
        uint32_t tctx = s->regs[PDMA_TX_CTX_IDX0 >> 2];
        uint32_t tdtx = s->regs[PDMA_TX_DTX_IDX0 >> 2];
        uint32_t tcnt = s->regs[PDMA_TX_MAX_CNT0 >> 2];
        hwaddr tbp = mips_dma_addr(s->regs[PDMA_TX_BASE_PTR0 >> 2]);

        if (tcnt > 0 && tcnt <= 4096) {
            uint32_t j, n = 0;
            for (j = tdtx; n < 16; n++) {
                uint32_t d[4];
                pdma_desc_read(tbp + (hwaddr)j * DESC_SIZE, d);
                j = idx_next(j, tcnt);
                if (j == tctx) break;
            }
        }
    }
}

/* =====================================================================
 *  Register read / write
 * ===================================================================== */
static uint64_t mt7621_eth_read(void *opaque, hwaddr offset, unsigned size)
{
    MT7621EthState *s = MT7621_ETH(opaque);
    uint32_t idx = offset >> 2;
    uint32_t val;

    if (idx >= ARRAY_SIZE(s->regs)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_eth: read beyond register space at 0x%"
                      HWADDR_PRIx "\n", offset);
        return 0;
    }

    val = s->regs[idx];

    /*
     * Death-loop detector: if the firmware keeps polling RX_DONE without
     * servicing it (counter is reset on every CRX write and every
     * INT_STATUS clear), dump the RX ring state once to reveal the cause.
     */
    if (offset == PDMA_INT_STATUS && (val & (INT_RX_DONE0 | INT_TX_DONE0))) {
        s->rx_stall_count++;
        if (s->rx_stall_count == 200) {
            mt7621_eth_dump_rx_state(s, "DEATH-LOOP");
        }
    }
    return val;
}

static void mt7621_eth_write(void *opaque, hwaddr offset,
                             uint64_t val, unsigned size)
{
    MT7621EthState *s = MT7621_ETH(opaque);
    uint32_t idx = offset >> 2;
    uint32_t old_val;

    if (idx >= ARRAY_SIZE(s->regs)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_eth: write beyond register space at 0x%"
                      HWADDR_PRIx " = 0x%" PRIx64 "\n", offset, val);
        return;
    }

    old_val = s->regs[idx];
    s->regs[idx] = (uint32_t)val;

    switch (offset) {
    /*
     * MDIO transaction: bit 31 START auto-clears.  For reads, fill
     * in emulated PHY data.
     */
    case GSW_MDIO_ACCESS:
        if (val & MDIO_START_BIT) {
            if (val & MDIO_CMD_RD) {
                uint32_t phy = (val >> 25) & 0x1F;
                uint32_t reg = (val >> 20) & 0x1F;
                uint16_t data = mt7621_mdio_phy_read(phy, reg);
                s->regs[idx] = (val & ~MDIO_START_BIT) | data;
            } else {
                s->regs[idx] = val & ~MDIO_START_BIT;
            }
        }
        break;

    /*
     * PDMA RST_IDX: reset DMA ring indices only (NOT CPU indices).
     * The CPU indices (CTX_IDX, CRX_IDX) are managed by software
     * and are not affected by hardware reset.
     */
    case PDMA_RST_IDX:
        if (val & RST_IDX_TX0) {
            s->regs[PDMA_TX_DTX_IDX0 >> 2] = 0;
        }
        if (val & RST_IDX_RX0) {
            s->regs[PDMA_RX_DRX_IDX0 >> 2] = 0;
            s->crx_committed = false;  /* ring reset: CRX not yet meaningful */
        }
        s->regs[idx] = 0;   /* write-only */
        mt7621_eth_try_flush(s);
        break;

    /*
     * TX ring setup registers -- trace for debugging.
     */
    case PDMA_TX_BASE_PTR0:
    case PDMA_TX_MAX_CNT0:
        break;

    /*
     * RX ring setup registers.  When the ring becomes ready (or after
     * the CPU recycles descriptors via CRX_IDX), flush any packets the
     * net backend buffered while can_receive() was returning false.
     */
    case PDMA_RX_BASE_PTR0:
    case PDMA_RX_MAX_CNT0:
        mt7621_eth_try_flush(s);
        break;

    case PDMA_RX_CRX_IDX0:
        {
            uint32_t new_crx = (uint32_t)val;
            uint32_t old_crx = old_val;
            uint32_t drx_val = s->regs[PDMA_RX_DRX_IDX0 >> 2];
            uint32_t cnt_val = s->regs[PDMA_RX_MAX_CNT0 >> 2];
            hwaddr base_phys = mips_dma_addr(s->regs[PDMA_RX_BASE_PTR0 >> 2]);

            /*
             * Clear DONE bits for descriptors the CPU just returned
             * to DMA (from old_crx+1 to new_crx, inclusive, wrapping).
             * This is required because some firmware (e.g. Breed) only
             * writes CRX without explicitly clearing DONE in each
             * descriptor.  Without this, when DRX wraps around after
             * 64 packets, it hits descriptors that still have DONE=1
             * and the ring stalls permanently.
             */
            if (cnt_val > 0 && new_crx != old_crx) {
                uint32_t i = old_crx;
                do {
                    uint32_t desc[4];
                    hwaddr desc_addr;
                    i = idx_next(i, cnt_val);
                    desc_addr = base_phys + (hwaddr)i * DESC_SIZE;
                    if (pdma_desc_read(desc_addr, desc) &&
                        (desc[1] & DESC_DDONE)) {
                        desc[1] &= ~DESC_DDONE;
                        pdma_desc_write_word(desc_addr, 1, desc[1]);
                    }
                } while (i != new_crx);
            }

            /*
             * Mark CRX as committed if we've already received at least
             * one packet.  Initial CRX writes (e.g. Breed writes 63
             * before any data arrives) have drx==0 and do NOT set this
             * flag, so the (next_drx == crx) heuristic stays suppressed
             * until the CPU has genuinely consumed some descriptors.
             */
            if (drx_val != 0) {
                s->crx_committed = true;
            }
            s->rx_stall_count = 0; /* CRX written: CPU is alive */
        }
        break;

    /*
     * GLO_CFG: TX/RX DMA enable.  When RX DMA is first enabled, flush
     * any packets buffered during boot (before the ring was up).
     */
    case PDMA_GLO_CFG:
        mt7621_eth_try_flush(s);
        break;

    /*
     * TX trigger: CPU advances the TX CPU index, signalling that new
     * descriptors have been filled and are ready for DMA processing.
     */
    case PDMA_TX_CTX_IDX0:
        mt7621_eth_tx_kick(s);
        break;

    /*
     * Interrupt status registers: write-1-to-clear.
     */
    case PDMA_INT_STATUS:
        s->rx_stall_count = 0;  /* interrupt being serviced: not stuck */
        s->regs[PDMA_INT_STATUS >> 2] = old_val & ~(uint32_t)val;
        mt7621_eth_update_irq(s);
        break;

    case FE_INT_STATUS:
        s->regs[FE_INT_STATUS >> 2] = old_val & ~(uint32_t)val;
        mt7621_eth_update_irq(s);
        break;

    /*
     * Interrupt mask / enable registers.
     */
    case PDMA_INT_MASK:
        mt7621_eth_update_irq(s);
        break;
    case FE_INT_ENABLE:
        mt7621_eth_update_irq(s);
        break;

    /*
     * FE global reset.
     */
    case FE_RST_GL:
        if (val & 0x01) {
            s->regs[idx] = 0;
        }
        break;

    default:
        break;
    }
}

static const MemoryRegionOps mt7621_eth_ops = {
    .read  = mt7621_eth_read,
    .write = mt7621_eth_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* =====================================================================
 *  NetClientInfo
 * ===================================================================== */
static NetClientInfo net_mt7621_eth_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = mt7621_eth_can_receive,
    .receive = mt7621_eth_receive,
    .link_status_changed = mt7621_eth_set_link,
};

/* =====================================================================
 *  Device lifecycle
 * ===================================================================== */
static void mt7621_eth_reset_hold(Object *obj, ResetType type)
{
    MT7621EthState *s = MT7621_ETH(obj);
    memset(s->regs, 0, sizeof(s->regs));
    s->rx_pending_len = 0;
    s->crx_committed = false;   /* CRX not yet committed after real RX */
    s->rx_irq_down_ns = 0;
    s->rx_stall_count = 0;

    /* GSW chip ID: MT7530 = 0x00007530. */
    s->regs[GSW_CHIP_ID >> 2] = 0x00007530;

    /*
     * Interrupt mask / enable registers reset to 0 (all interrupts
     * disabled).  Software must explicitly enable them.  This matches
     * real MT7621 hardware and prevents spurious interrupts during boot.
     */
    s->regs[PDMA_INT_MASK >> 2] = 0;
    s->regs[FE_INT_ENABLE >> 2] = 0;
}

static void mt7621_eth_init(Object *obj)
{
    MT7621EthState *s = MT7621_ETH(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    sysbus_init_irq(sbd, &s->irq);
    memory_region_init_io(&s->iomem, obj, &mt7621_eth_ops, s,
                          TYPE_MT7621_ETH, MT7621_ETH_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    s->rx_dma_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   mt7621_eth_dma_timer_cb, s);
}

static void mt7621_eth_realize(DeviceState *dev, Error **errp)
{
    MT7621EthState *s = MT7621_ETH(dev);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_mt7621_eth_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic),
                             s->conf.macaddr.a);
}

static const Property mt7621_eth_properties[] = {
    DEFINE_NIC_PROPERTIES(MT7621EthState, conf),
};

static void mt7621_eth_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = mt7621_eth_reset_hold;
    dc->realize = mt7621_eth_realize;
    device_class_set_props(dc, mt7621_eth_properties);
    dc->desc = "MT7621 Ethernet Frame Engine (PDMA)";
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo mt7621_eth_info = {
    .name          = TYPE_MT7621_ETH,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT7621EthState),
    .instance_init = mt7621_eth_init,
    .class_init    = mt7621_eth_class_init,
};

static void mt7621_eth_register_types(void)
{
    type_register_static(&mt7621_eth_info);
}

type_init(mt7621_eth_register_types)
