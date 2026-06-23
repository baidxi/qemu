/*
 * Cluster Power Controller emulation
 *
 * Copyright (c) 2016 Imagination Technologies
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "target/mips/cpu.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/cpu.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"

#include "hw/misc/mips_cpc.h"
#include "hw/core/qdev-properties.h"

static inline uint64_t cpc_vp_run_mask(MIPSCPCState *cpc)
{
    return (1ULL << cpc->num_vp) - 1;
}

static void mips_cpu_reset_async_work(CPUState *cs, run_on_cpu_data data)
{
    MIPSCPCState *cpc = (MIPSCPCState *) data.host_ptr;

    cpu_reset(cs);
    cs->halted = 0;
    cpc->vp_running |= 1ULL << cs->cpu_index;
}

static void cpc_run_vp(MIPSCPCState *cpc, uint64_t vp_run)
{
    CPUState *cs;

    CPU_FOREACH(cs) {
        uint64_t i = 1ULL << cs->cpu_index;
        if (i & vp_run & ~cpc->vp_running) {
            /*
             * To avoid racing with a CPU we are just kicking off.
             * We do the final bit of preparation for the work in
             * the target CPUs context.
             */
            async_safe_run_on_cpu(cs, mips_cpu_reset_async_work,
                                  RUN_ON_CPU_HOST_PTR(cpc));
        }
    }
}

static void cpc_stop_vp(MIPSCPCState *cpc, uint64_t vp_stop)
{
    CPUState *cs;

    CPU_FOREACH(cs) {
        uint64_t i = 1ULL << cs->cpu_index;
        if (i & vp_stop & cpc->vp_running) {
            cpu_interrupt(cs, CPU_INTERRUPT_HALT);
            cpc->vp_running &= ~i;
        }
    }
}

/*
 * Boot the VPE0 of the core selected via CPC_CL_OTHER.CORENUM.  MT7621
 * has 2 VPEs per core, so core N's VPE0 is at cpu_index N*vpes_per_core.
 */
static void cpc_boot_selected_core(MIPSCPCState *s)
{
    unsigned int num_cores = s->num_vp / 2;
    unsigned int vpes_per_core, vpe0;

    if (num_cores == 0) {
        num_cores = 1;
    }
    vpes_per_core = s->num_vp / num_cores;
    vpe0 = s->other_core * vpes_per_core;
    if (vpe0 < s->num_vp) {
        cpc_run_vp(s, 1ULL << vpe0);
    }
}

/*
 * Translate a per-core VP mask (written to the core-other VP_RUN/VP_STOP
 * registers) into a global cpu_index mask.  For MT7621A (2 VPEs/core) core
 * C's VP i maps to cpu_index C*vpes_per_core + i.
 */
static uint64_t cpc_co_vp_global_mask(MIPSCPCState *s, uint64_t per_core_mask)
{
    unsigned int num_cores = s->num_vp / 2;
    unsigned int vpc, base;
    uint64_t global = 0;
    int i;

    if (num_cores == 0) {
        num_cores = 1;
    }
    vpc = s->num_vp / num_cores;
    base = s->other_core * vpc;
    for (i = 0; i < (int)vpc && (base + i) < s->num_vp; i++) {
        if (per_core_mask & (1ULL << i)) {
            global |= 1ULL << (base + i);
        }
    }
    return global & cpc_vp_run_mask(s);
}

static void cpc_write(void *opaque, hwaddr offset, uint64_t data,
                      unsigned size)
{
    MIPSCPCState *s = opaque;

    switch (offset) {
    case CPC_CL_BASE_OFS + CPC_VP_RUN_OFS:
        cpc_run_vp(s, data & cpc_vp_run_mask(s));
        break;
    case CPC_CO_BASE_OFS + CPC_VP_RUN_OFS:
        /* core-other: mask is per-core, map to the selected core's VPs */
        cpc_run_vp(s, cpc_co_vp_global_mask(s, data));
        break;
    case CPC_CL_BASE_OFS + CPC_VP_STOP_OFS:
        cpc_stop_vp(s, data & cpc_vp_run_mask(s));
        break;
    case CPC_CO_BASE_OFS + CPC_VP_STOP_OFS:
        cpc_stop_vp(s, cpc_co_vp_global_mask(s, data));
        break;
    case CPC_CL_BASE_OFS + CPC_Cx_OTHER_OFS:
        /* CPC_CL_OTHER: select the core targeted by core-other accesses */
        s->other_core = (data >> CPC_Cx_OTHER_CORENUM_SHIFT) & 0xff;
        break;
    case CPC_CL_BASE_OFS + CPC_Cx_CMD_OFS:
    case CPC_CO_BASE_OFS + CPC_Cx_CMD_OFS:
        /*
         * CPC_Cx_CMD: the guest (kernel boot_core / U-Boot SPL) requests a
         * core power-up/reset.  For RESET/PWRUP, start the selected core's
         * VPE0 so the kernel's boot_core() sequence completes.
         */
        if ((data & 0xf) == CPC_Cx_CMD_RESET) {
            cpc_boot_selected_core(s);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: Bad offset 0x%x\n",  __func__, (int)offset);
        break;
    }
}

static uint64_t cpc_read(void *opaque, hwaddr offset, unsigned size)
{
    MIPSCPCState *s = opaque;

    switch (offset) {
    case CPC_CL_BASE_OFS + CPC_VP_RUNNING_OFS:
    case CPC_CO_BASE_OFS + CPC_VP_RUNNING_OFS:
        return s->vp_running;
    case CPC_CL_BASE_OFS + CPC_Cx_STAT_CONF_OFS:
    case CPC_CO_BASE_OFS + CPC_Cx_STAT_CONF_OFS:
        /*
         * boot_core() polls SEQSTATE for U6 (coherent execution) to decide
         * the core has powered up.  Report U6 unconditionally so the poll
         * completes (the VPE is started synchronously via cpc_run_vp above).
         */
        return CPC_SEQSTATE_U6;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: Bad offset 0x%x\n",  __func__, (int)offset);
        return 0;
    }
}

static const MemoryRegionOps cpc_ops = {
    .read = cpc_read,
    .write = cpc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .max_access_size = 8,
    },
};

static void mips_cpc_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    MIPSCPCState *s = MIPS_CPC(obj);

    memory_region_init_io(&s->mr, OBJECT(s), &cpc_ops, s, "mips-cpc",
                          CPC_ADDRSPACE_SZ);
    sysbus_init_mmio(sbd, &s->mr);
}

static void mips_cpc_realize(DeviceState *dev, Error **errp)
{
    MIPSCPCState *s = MIPS_CPC(dev);

    if (s->vp_start_running > cpc_vp_run_mask(s)) {
        error_setg(errp,
                   "incorrect vp_start_running 0x%" PRIx64 " for num_vp = %d",
                   s->vp_running, s->num_vp);
        return;
    }
}

static void mips_cpc_reset(DeviceState *dev)
{
    MIPSCPCState *s = MIPS_CPC(dev);

    /* Reflect the fact that all VPs are halted on reset */
    s->vp_running = 0;

    /* Put selected VPs into run state */
    cpc_run_vp(s, s->vp_start_running);
}

static const VMStateDescription vmstate_mips_cpc = {
    .name = "mips-cpc",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(vp_running, MIPSCPCState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property mips_cpc_properties[] = {
    DEFINE_PROP_UINT32("num-vp", MIPSCPCState, num_vp, 0x1),
    DEFINE_PROP_UINT64("vp-start-running", MIPSCPCState, vp_start_running, 0x1),
};

static void mips_cpc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mips_cpc_realize;
    device_class_set_legacy_reset(dc, mips_cpc_reset);
    dc->vmsd = &vmstate_mips_cpc;
    device_class_set_props(dc, mips_cpc_properties);
}

static const TypeInfo mips_cpc_info = {
    .name          = TYPE_MIPS_CPC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MIPSCPCState),
    .instance_init = mips_cpc_init,
    .class_init    = mips_cpc_class_init,
};

static void mips_cpc_register_types(void)
{
    type_register_static(&mips_cpc_info);
}

type_init(mips_cpc_register_types)
