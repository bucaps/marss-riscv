/**
 * RISCV Simulated CPU State
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2017-2019 Gaurav Kothari {gkothar1@binghamton.edu}
 * State University of New York at Binghamton
 *
 * Copyright (c) 2018-2019 Parikshit Sarnaik {psarnai1@binghamton.edu}
 * State University of New York at Binghamton
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef _RISCV_SIM_CPU_H_
#define _RISCV_SIM_CPU_H_

#include "sim_params_stats.h"
#include "bpu.h"
#include "mmu.h"
#include "riscv_sim_typedefs.h"
#include "common_core_utils.h"

/* Forward declare */
struct RISCVCPUState;

typedef struct RISCVSIMCPUState
{
    /*----------  Common parameters for both core types  ----------*/
    target_ulong pc;        /* Next PC to fetch from */
    uint64_t clock;         /* Clock cycles elapsed */
    IMapEntry *imap;        /* Instruction map to store data of active instructions */
    SimStats *stats;        /* Simulation Stats */
    SimParams *params;      /* Simulation Parameters */
    BranchPredUnit *bpu;    /* Branch prediction unit */
    int (*pfn_branch_handler)(struct RISCVCPUState *, IMapEntry *);
    void (*pfn_branch_frontend_probe_handler)(struct RISCVCPUState *,IMapEntry *);
    int (*pfn_branch_frontend_decode_handler)(struct RISCVCPUState *, IMapEntry *);

    MMU *mmu;               /* Memory controller */

    /*----------  Based on core type: in-order or out-of-order  ----------*/
    void* core;
    void (*pfn_core_reset)(void *core);
    int (*pfn_core_run)(void *core);
    void (*pfn_core_free)(void *core);

    struct RISCVCPUState *emu_cpu_state;   /* Pointer to emulated CPU state */
} RISCVSIMCPUState;

RISCVSIMCPUState *riscv_sim_cpu_init(const SimParams *p, struct RISCVCPUState *s);
void riscv_sim_cpu_reset(RISCVSIMCPUState *simcpu);
int riscv_sim_cpu_run(RISCVSIMCPUState *simcpu);
void riscv_sim_cpu_free(RISCVSIMCPUState **simcpu);
#endif
