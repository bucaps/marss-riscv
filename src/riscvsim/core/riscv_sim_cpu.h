/**
 * RISCV Simulated CPU State
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2017-2020 Gaurav Kothari {gkothar1@binghamton.edu}
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

#include <time.h>

#include "../bpu/bpu.h"
#include "../memory_hierarchy/memory_hierarchy.h"
#include "../memory_hierarchy/temu_mem_map_wrapper.h"
#include "../riscv_sim_typedefs.h"
#include "../utils/cpu_latches.h"
#include "../utils/sim_exception.h"
#include "../utils/sim_params.h"
#include "../utils/sim_stats.h"
#include "../utils/sim_trace.h"

/* Forward declare */
struct RISCVCPUState;

typedef struct RISCVSIMCPUState
{
    target_ulong pc; /* Next PC to fetch from */
    uint64_t clock;  /* Clock cycles elapsed since simulation start  */

    /* Simulator maintains a pool of free instruction latches known as
     * insn_latch_pool. Every instruction fetched into the pipeline is allocated
     * a instruction latch from this pool. The pointer of this latch is passed
     * across the pipeline stages when the instruction advances through the
     * pipeline. When the instruction commits, the latch is added back to the
     * pool for reuse by following instructions. */
    InstructionLatch *insn_latch_pool;

    SimStats *stats;
    SimParams *params;
    BranchPredUnit *bpu;

    /* Memory hierarchy to simulate the delays We do not model the actual data
     * in the hierarchy for simplicity, but just the addresses for simulating
     * the delays.*/
    MemoryHierarchy *mem_hierarchy;

    /* Wrapper over TinyEMU memory map used by the simulator to fetch
     * instructions and read/write data (loads, stores and atomics) from the
     * emulated guest physical memory */
    TemuMemMapWrapper *temu_mem_map_wrapper;

    /* If an exception occurs, simulator sets the exception object, which is
     * used by TinyEMU helper functions to set up CPU context */
    SimException *exception;

    /* For generating simulation trace */
    SimTrace *trace;

    /* Pointer to shared memory area to write stats, which is read by
     * sim-stats-display tool */
    SimStats *stats_shm_ptr;

    /* Used to enable/disable simulation mode, measure simulation time */
    int simulation;
    int return_to_sim;
    struct timespec sim_start_time;
    struct timespec sim_end_time;

    /* BPU handler routines when BPU is enabled or disabled */
    void (*bpu_fetch_stage_handler)(struct RISCVCPUState *, InstructionLatch *);
    int (*bpu_decode_stage_handler)(struct RISCVCPUState *, InstructionLatch *);
    int (*bpu_execute_stage_handler)(struct RISCVCPUState *,
                                     InstructionLatch *);

    struct RISCVCPUState *emu_cpu_state; /* Pointer to emulated CPU state */

    /*----------  Set based on core type: in-order or out-of-order  ----------*/
    void *core;
    void (*core_reset)(void *core);
    void (*core_free)(void *core);
    int (*core_run)(void *core);
} RISCVSIMCPUState;

RISCVSIMCPUState *riscv_sim_cpu_init(const SimParams *p,
                                     struct RISCVCPUState *s);
int riscv_sim_cpu_switch_to_cpu_simulation(RISCVSIMCPUState *simcpu);
void riscv_sim_cpu_start(RISCVSIMCPUState *simcpu, target_ulong pc);
void riscv_sim_cpu_stop(RISCVSIMCPUState *simcpu, target_ulong pc);
void riscv_sim_cpu_reset(RISCVSIMCPUState *simcpu);
void riscv_sim_cpu_free(RISCVSIMCPUState **simcpu);

int get_data_mem_access_latency(struct RISCVCPUState *s, InstructionLatch *e);
void fetch_cpu_stage_exec(struct RISCVCPUState *s, InstructionLatch *e);
void mem_cpu_stage_exec(struct RISCVCPUState *s, InstructionLatch *e);
void decode_cpu_stage_exec(struct RISCVCPUState *s, InstructionLatch *e);
void update_arch_reg_int(struct RISCVCPUState *s, InstructionLatch *e);
void update_arch_reg_fp(struct RISCVCPUState *s, InstructionLatch *e);
void update_insn_commit_stats(struct RISCVCPUState *s, InstructionLatch *e);
void write_stats_to_stats_display_shm(RISCVSIMCPUState *simcpu);
int set_max_clock_cycles_for_non_pipe_fu(struct RISCVCPUState *s, int fu_type,
                                         InstructionLatch *e);
#endif
