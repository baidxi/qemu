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

#ifndef MIPS_CPC_H
#define MIPS_CPC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define CPC_ADDRSPACE_SZ    0x6000

/* CPC blocks offsets relative to base address */
#define CPC_CL_BASE_OFS     0x2000
#define CPC_CO_BASE_OFS     0x4000

/* CPC register offsets relative to block offsets */
#define CPC_Cx_CMD_OFS      0x00    /* command sequencer */
#define CPC_Cx_CMD_RESET    0x4
#define CPC_Cx_STAT_CONF_OFS 0x08   /* status / config */
#define CPC_Cx_OTHER_OFS    0x10    /* core-other selection */
#define CPC_Cx_OTHER_CORENUM_SHIFT  16
#define CPC_VP_STOP_OFS     0x20
#define CPC_VP_RUN_OFS      0x28
#define CPC_VP_RUNNING_OFS  0x30
#define CPC_SEQSTATE_U6     (0x7 << 19)  /* coherent execution (core up) */

#define TYPE_MIPS_CPC "mips-cpc"
OBJECT_DECLARE_SIMPLE_TYPE(MIPSCPCState, MIPS_CPC)

struct MIPSCPCState {
    SysBusDevice parent_obj;

    uint32_t num_vp;
    uint64_t vp_start_running; /* VPs running from restart */

    MemoryRegion mr;
    uint64_t vp_running; /* Indicates which VPs are in the run state */

    /*
     * Core selected by the guest via CPC_CL_OTHER.CORENUM, used to target
     * the per-core command (CPC_Cx_CMD) at a specific core's VPE0.
     */
    uint32_t other_core;
};

#endif /* MIPS_CPC_H */
