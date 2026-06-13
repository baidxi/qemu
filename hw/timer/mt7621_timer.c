/*
 * MT7621 Timer / Watchdog emulation
 *
 * Provides 2 general-purpose 32-bit countdown timers using QEMU's
 * ptimer framework. The watchdog is modeled but non-functional for
 * early boot (Linux uses it for system reset).
 *
 * Register map (per timer, n=1,2):
 *   0x00 + (n-1)*0x10: TIMER_LOAD   (RW) Reload value
 *   0x04 + (n-1)*0x10: TIMER_VALUE  (R)  Current count
 *   0x08 + (n-1)*0x10: TIMER_CTRL   (RW) Control register
 *     CTRL[0]   = timer enable
 *     CTRL[1]   = auto-reload (periodic mode)
 *     CTRL[5]   = interrupt enable
 *     CTRL[7]   = prescale /32
 *
 * Copyright (c) 2026
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/misc/mt7621_timer.h"
#include "hw/core/ptimer.h"
#include "hw/core/irq.h"

#define TIMER_LOAD     0x00
#define TIMER_VALUE    0x04
#define TIMER_CTRL     0x08

#define TIMER_STRIDE   0x10    /* spacing between timer register groups */

#define TIMER_CTRL_ENABLE     BIT(0)
#define TIMER_CTRL_AUTOLOAD   BIT(1)
#define TIMER_CTRL_INT_EN     BIT(5)
#define TIMER_CTRL_PRESCALE   BIT(7)

static void mt7621_timer_tick(void *opaque)
{
    MT7621TimerChannel *ch = opaque;

    ch->int_pending = true;
    if (ch->ctrl & TIMER_CTRL_INT_EN) {
        qemu_irq_raise(ch->irq);
    }
}

static void mt7621_timer_update(MT7621TimerChannel *ch)
{
    ptimer_transaction_begin(ch->ptimer);

    if (ch->ctrl & TIMER_CTRL_ENABLE) {
        uint32_t period_ns;

        /* Calculate period in ns: 1 / (clk_freq / prescale) */
        period_ns = 1000000000ULL / (ch->clk_freq);
        if (ch->ctrl & TIMER_CTRL_PRESCALE) {
            period_ns *= 32;
        }

        ptimer_set_period(ch->ptimer, period_ns);
        ptimer_set_limit(ch->ptimer, ch->load ? ch->load : 0xFFFFFFFF, 1);
        ptimer_run(ch->ptimer,
                   (ch->ctrl & TIMER_CTRL_AUTOLOAD) ? 0 : 1);
    } else {
        ptimer_stop(ch->ptimer);
    }

    ptimer_transaction_commit(ch->ptimer);
}

static uint64_t mt7621_timer_read(void *opaque, hwaddr offset,
                                  unsigned size)
{
    MT7621TimerState *s = MT7621_TIMER(opaque);
    int ch_idx = offset / TIMER_STRIDE;
    int reg = offset % TIMER_STRIDE;

    if (ch_idx >= 2) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_timer: bad channel %d\n", ch_idx);
        return 0;
    }

    MT7621TimerChannel *ch = &s->channel[ch_idx];

    switch (reg) {
    case TIMER_LOAD:
        return ch->load;
    case TIMER_VALUE:
        return ptimer_get_count(ch->ptimer);
    case TIMER_CTRL:
        return ch->ctrl;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_timer: bad read offset 0x%" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void mt7621_timer_write(void *opaque, hwaddr offset,
                               uint64_t val, unsigned size)
{
    MT7621TimerState *s = MT7621_TIMER(opaque);
    int ch_idx = offset / TIMER_STRIDE;
    int reg = offset % TIMER_STRIDE;

    if (ch_idx >= 2) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_timer: bad channel %d\n", ch_idx);
        return;
    }

    MT7621TimerChannel *ch = &s->channel[ch_idx];

    switch (reg) {
    case TIMER_LOAD:
        ch->load = val;
        break;
    case TIMER_VALUE:
        /* Writing to VALUE resets the timer count to LOAD */
        break;
    case TIMER_CTRL:
        ch->ctrl = val & 0xFF;
        mt7621_timer_update(ch);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt7621_timer: bad write offset 0x%" HWADDR_PRIx "\n",
                      offset);
        return;
    }
}

static const MemoryRegionOps mt7621_timer_ops = {
    .read  = mt7621_timer_read,
    .write = mt7621_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mt7621_timer_reset_hold(Object *obj, ResetType type)
{
    MT7621TimerState *s = MT7621_TIMER(obj);

    for (int i = 0; i < 2; i++) {
        MT7621TimerChannel *ch = &s->channel[i];
        ch->load = 0;
        ch->ctrl = 0;
        ch->int_pending = false;
        ptimer_transaction_begin(ch->ptimer);
        ptimer_stop(ch->ptimer);
        ptimer_transaction_commit(ch->ptimer);
        qemu_irq_lower(ch->irq);
    }
}

static void mt7621_timer_realize(DeviceState *dev, Error **errp)
{
    MT7621TimerState *s = MT7621_TIMER(dev);

    for (int i = 0; i < 2; i++) {
        MT7621TimerChannel *ch = &s->channel[i];
        const char *name;

        /* Create a ptimer for each channel, 50 MHz default clock */
        ch->clk_freq = 50000000;    /* 50 MHz peripheral clock */
        ch->ptimer = ptimer_init(mt7621_timer_tick, ch,
                                 PTIMER_POLICY_LEGACY);
        ptimer_transaction_begin(ch->ptimer);
        ptimer_set_period(ch->ptimer,
                          (uint64_t)1000000000 / ch->clk_freq);
        ptimer_transaction_commit(ch->ptimer);

        /* Initialize IRQ lines */
        name = g_strdup_printf("timer%d", i);
        qdev_init_gpio_out_named(dev, &ch->irq, name, 1);
        g_free((void *)name);
    }
}

static void mt7621_timer_init(Object *obj)
{
    MT7621TimerState *s = MT7621_TIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &mt7621_timer_ops, s,
                          TYPE_MT7621_TIMER, 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void mt7621_timer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = mt7621_timer_realize;
    dc->desc = "MT7621 Timer/Watchdog";
    rc->phases.hold = mt7621_timer_reset_hold;
}

static const TypeInfo mt7621_timer_info = {
    .name          = TYPE_MT7621_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT7621TimerState),
    .instance_init = mt7621_timer_init,
    .class_init    = mt7621_timer_class_init,
};

static void mt7621_timer_register_types(void)
{
    type_register_static(&mt7621_timer_info);
}

type_init(mt7621_timer_register_types)
