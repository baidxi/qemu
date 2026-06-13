/*
 * MT7621 Ethernet Frame Engine (GMAC) — register-store stub
 *
 * All writes are stored into a backing register array; reads return
 * the stored value so that firmware (e.g. Breed) can verify its own
 * configuration.  No actual packet DMA is performed.
 *
 * Register blocks (offsets from FE base 0x1E110000):
 *
 *   FE core:   0x0000-0x00FF  (interrupt status/mask, global config)
 *   PDMA:      0x0800-0x0BFF  (RX/TX ring pointers, global cfg, IRQ)
 *   QDMA:      0x1800-0x1BFF  (queue DMA, free queue)
 *   GDMA1:     0x2400-0x27FF  (Gigabit MAC 1 counters)
 *   GDMA2:     0x2800-0x2BFF  (Gigabit MAC 2 counters)
 *   GSW:       0x10000-0x1FFFF (Gigabit Switch registers)
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/net/mt7621_eth.h"
#include "qemu/log.h"

/*
 * GSW (Gigabit Switch) register offsets within the ETH region.
 * The GSW is embedded in the MT7621 at ETH base + 0x10000.
 */
#define GSW_BASE_OFF      0x10000
#define GSW_CHIP_ID       0x1001C   /* GSW chip identification */
#define GSW_SYS_CTRL      0x10000   /* GSW system control */

/*
 * PDMA offsets
 */
#define PDMA_RX_BASE_PTR0   0x0900
#define PDMA_RX_MAX_CNT0    0x0904
#define PDMA_RX_CRX_IDX0    0x0908
#define PDMA_RX_DRX_IDX0    0x090C
#define PDMA_TX_BASE_PTR0   0x0A00
#define PDMA_TX_MAX_CNT0    0x0A04
#define PDMA_TX_CTX_IDX0    0x0A08
#define PDMA_RST_IDX        0x0A0C
#define PDMA_GLO_CFG        0x0A04
#define PDMA_IRQ_STATUS     0x0A20
#define PDMA_IRQ_MASK       0x0A28

/* FE core */
#define FE_RST_GL           0x000C

/*
 * MDIO Access register (offset 0x04 in Ralink SDK FE register map).
 * Bit layout:
 *   [31]      START      — write 1 to begin transaction; auto-clears
 *   [30]      PHY_ID     — 0: GE1 PHY, 1: GE2 PHY
 *   [29:25]   PHY_ADDR   — PHY address (5 bits)
 *   [24:20]   REG_ADDR   — PHY register (5 bits)
 *   [19:18]   CMD        — 01: read, 10: write (some SDKs use single bits)
 *   [17:16]   MDC config
 *   [15:0]    DATA       — read or write data
 */
#define FE_MDIO_ACCESS      0x0004
#define GSW_MDIO_ACCESS     (GSW_BASE_OFF + 0x04)  /* 0x10004 */
#define MDIO_START_BIT      (1u << 31)
#define MDIO_CMD_RD         (1u << 18)  /* read command */

/*
 * Minimal PHY register emulation for Breed's ETH init.
 * Returns plausible values so Breed doesn't hang waiting for PHY data.
 */
static uint16_t mt7621_mdio_phy_read(uint32_t phy_addr, uint32_t reg_addr)
{
    switch (reg_addr) {
    case 0:   /* MII_BMCR – Basic Mode Control */
        return 0x1140;  /* ANEG_EN | DUPLEX | SPEED100 */
    case 1:   /* MII_BMSR – Basic Mode Status */
        return 0x796D;  /* LSTATUS | ANEGCOMP | 100FD | 1000FD */
    case 2:   /* MII_PHYSID1 */
        return 0x0000;
    case 3:   /* MII_PHYSID2 */
        return 0x0000;
    case 4:   /* MII_ADVERTISE */
        return 0x01E1;  /* 100FD | 100HD | 10FD | 10HD */
    case 5:   /* MII_LPA */
        return 0x45E1;
    case 9:   /* MII_CTRL1000 */
        return 0x0300;  /* 1000FD advertised */
    default:
        return 0x0000;
    }
}

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

    switch (offset) {
    /*
     * DMA index registers: return the CPU-writeable counterpart so that
     * firmware sees its own ring position when no real DMA is running.
     */
    case PDMA_RX_DRX_IDX0:   /* shadow → RX_CRX_IDX0 */
        val = s->regs[PDMA_RX_CRX_IDX0 >> 2];
        break;
    default:
        break;
    }

    return val;
}

static void mt7621_eth_write(void *opaque, hwaddr offset,
                             uint64_t val, unsigned size)
{
    MT7621EthState *s = MT7621_ETH(opaque);
    uint32_t idx = offset >> 2;

    if (idx >= ARRAY_SIZE(s->regs)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_eth: write beyond register space at 0x%"
                      HWADDR_PRIx " = 0x%" PRIx64 "\n", offset, val);
        return;
    }

    /* Store the value first (write-through) */
    s->regs[idx] = (uint32_t)val;

    /* Special side-effect handling */
    switch (offset) {
    case FE_MDIO_ACCESS:
    case GSW_MDIO_ACCESS:
        /*
         * MDIO transaction: bit 31 (START) auto-clears when the
         * transaction completes.  Since we don't have real PHYs,
         * complete it instantly.  For read commands, populate the
         * lower 16 bits with emulated PHY register data.
         */
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

    case PDMA_RST_IDX:
        if (val & 0x01) {  /* Reset RX indices */
            s->regs[PDMA_RX_CRX_IDX0 >> 2] = 0;
        }
        if (val & 0x02) {  /* Reset TX indices */
            s->regs[PDMA_TX_CTX_IDX0 >> 2] = 0;
        }
        /* RST_IDX is write-only, clear it after taking effect */
        s->regs[idx] = 0;
        break;

    case FE_RST_GL:
        /* FE global reset — clear all DMA engine registers */
        if (val & 0x01) {
            s->regs[idx] = 0;
        }
        break;

    default:
        /* No special handling needed */
        break;
    }
}

static const MemoryRegionOps mt7621_eth_ops = {
    .read  = mt7621_eth_read,
    .write = mt7621_eth_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void mt7621_eth_reset_hold(Object *obj, ResetType type)
{
    MT7621EthState *s = MT7621_ETH(obj);
    memset(s->regs, 0, sizeof(s->regs));

    /*
     * GSW (Gigabit Switch) chip identification.
     * The GSW is an embedded MT7530-class switch at offset 0x10000.
     * Chip ID at offset 0x1001C: must return a valid ID for Breed to
     * detect the switch and initialize the ethernet driver.
     * MT7530 chip ID = 0x00007530.
     */
    s->regs[0x1001C >> 2] = 0x00007530;
}

static void mt7621_eth_init(Object *obj)
{
    MT7621EthState *s = MT7621_ETH(obj);
    /* 256KB covers FE + GMACs + switch */
    memory_region_init_io(&s->iomem, obj, &mt7621_eth_ops, s,
                          TYPE_MT7621_ETH, MT7621_ETH_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void mt7621_eth_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    dc->desc = "MT7621 Ethernet Frame Engine";
    rc->phases.hold = mt7621_eth_reset_hold;
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
