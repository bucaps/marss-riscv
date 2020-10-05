/**
 * Contains top-level routines to manage the in-order core
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
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "../../cutils.h"
#include "../../riscv_cpu_priv.h"
#include "../utils/circular_queue.h"
#include "../utils/sim_log.h"
#include "inorder.h"
#include "riscv_sim_cpu.h"

static void
in_core_log_config(const INCore *core)
{
    sim_log_event_to_file(sim_log, "%s", "Setting up in-order RISC-V core");
    sim_log_param_to_file(sim_log, "%s: %d", "core_id", core->simcpu->core_id);
    sim_log_param_to_file(sim_log, "%s: %s", "core_name",
                  core->simcpu->params->core_name);
    sim_log_param_to_file(sim_log, "%s: %s", "core_type",
                  core_type_str[core->simcpu->params->core_type]);
    sim_log_param_to_file(sim_log, "%s: %d", "num_cpu_stages",
                  core->simcpu->params->num_cpu_stages);
    sim_log_param_to_file(sim_log, "%s: %d", "num_data_fwd_buses", NUM_FWD_BUS);
    sim_log_param_to_file(sim_log, "%s: %d", "ex_to_mem_selector_queue_size",
                  INCORE_EX_TO_MEM_QUEUE_SIZE);
}

INCore *
in_core_init(const SimParams *p, struct RISCVSIMCPUState *simcpu)
{
    INCore *core;

    core = calloc(1, sizeof(INCore));
    assert(core);

    /* Create execution units */
    core->ialu = (CPUStage *)calloc(p->num_alu_stages, sizeof(CPUStage));
    assert(core->ialu);

    core->imul = (CPUStage *)calloc(p->num_mul_stages, sizeof(CPUStage));
    assert(core->imul);

    core->idiv = (CPUStage *)calloc(p->num_div_stages, sizeof(CPUStage));
    assert(core->idiv);

    core->fpu_fma = (CPUStage *)calloc(p->num_fpu_fma_stages, sizeof(CPUStage));
    assert(core->fpu_fma);

    /* Create FU to Memory selection queue */
    cq_init(&core->ex_to_mem_queue.cq, INCORE_EX_TO_MEM_QUEUE_SIZE);
    memset((void *)core->ex_to_mem_queue.data, 0,
           sizeof(uint64_t) * INCORE_EX_TO_MEM_QUEUE_SIZE);

    /* Set pointer to 5 or 6 stage run() function */
    switch (p->num_cpu_stages)
    {
        case 5:
        {
            core->pfn_incore_run_internal = &in_core_run_5_stage;
            break;
        }
    }

    core->simcpu = simcpu;
    in_core_log_config(core);
    return core;
}

void
in_core_reset(void *core_type)
{
    int i;
    INCore *core;

    core = (INCore *)core_type;

    /* Reset stages */
    cpu_stage_flush(&core->fetch);
    cpu_stage_flush(&core->decode);
    cpu_stage_flush(&core->memory1);
    cpu_stage_flush(&core->memory2);
    cpu_stage_flush(&core->commit);

    /* To start fetching */
    core->fetch.has_data = TRUE;

    /* Reset register valid bits */
    for (i = 0; i < NUM_INT_REG; ++i)
    {
        core->int_reg_status[i] = TRUE;
    }

    for (i = 0; i < NUM_FP_REG; ++i)
    {
        core->fp_reg_status[i] = TRUE;
    }

    /* Reset execution units */
    cpu_stage_flush_pipe(core->ialu, core->simcpu->params->num_alu_stages);
    cpu_stage_flush_pipe(core->imul, core->simcpu->params->num_mul_stages);
    cpu_stage_flush_pipe(core->idiv, core->simcpu->params->num_div_stages);
    cpu_stage_flush_pipe(core->fpu_fma,
                         core->simcpu->params->num_fpu_fma_stages);
    cpu_stage_flush(&core->fpu_alu);

    /* Reset EX to Memory queue */
    core->ins_dispatch_id = 0;
    cq_reset(&core->ex_to_mem_queue.cq);

    /* Reset Data FWD latches */
    memset((void *)core->fwd_latch, 0, sizeof(DataFWDLatch) * NUM_FWD_BUS);
}

void
in_core_free(void *core_type)
{
    INCore *core;

    core = (INCore *)(*(INCore **)core_type);
    free(core->fpu_fma);
    core->fpu_fma = NULL;
    free(core->idiv);
    core->idiv = NULL;
    free(core->imul);
    core->imul = NULL;
    free(core->ialu);
    core->ialu = NULL;
    free(core);
}

static int
in_core_pipeline_drained(const INCore *core)
{
    int i;
    RISCVSIMCPUState *simcpu = core->simcpu;

    if (core->fetch.has_data || core->decode.has_data
        || core->fpu_alu.has_data || core->memory1.has_data || core->memory2.has_data
        || core->commit.has_data)
    {
        return PIPELINE_NOT_DRAINED;
    }

    for (i = 0; i < simcpu->params->num_alu_stages; ++i)
    {
        if (core->ialu[i].has_data)
        {
            return PIPELINE_NOT_DRAINED;
        }
    }

    for (i = 0; i < simcpu->params->num_mul_stages; ++i)
    {
        if (core->imul[i].has_data)
        {
            return PIPELINE_NOT_DRAINED;
        }
    }

    for (i = 0; i < simcpu->params->num_div_stages; ++i)
    {
        if (core->idiv[i].has_data)
        {
            return PIPELINE_NOT_DRAINED;
        }
    }

    for (i = 0; i < simcpu->params->num_fpu_fma_stages; ++i)
    {
        if (core->fpu_fma[i].has_data)
        {
            return PIPELINE_NOT_DRAINED;
        }
    }

    return PIPELINE_DRAINED;
}

int
in_core_run(void *core_type)
{
    INCore *core = (INCore *)core_type;
    RISCVCPUState *s = core->simcpu->emu_cpu_state;

    while (1)
    {
        /* Advance DRAM clock */
        mem_controller_clock(s->simcpu->mem_hierarchy->mem_controller);

        /* For 5-stage pipeline calls in_core_run_5_stage(), For 6-stage
         * pipeline calls in_core_run_6_stage() */
        if (core->pfn_incore_run_internal(core))
        {
            return s->simcpu->exception->cause;
        }

        /* If an exception occurred and pipeline is drained, safely exit from
         * simulation */
        if (s->simcpu->exception->pending && in_core_pipeline_drained(core))
        {
            return s->simcpu->exception->cause;
        }

        /* Advance simulation cycle */
        ++s->simcpu->clock;
        ++s->simcpu->stats[s->priv].cycles;
    }
}

int
in_core_run_5_stage(INCore *core)
{
    if (in_core_commit(core))
    {
        /* Timeout */
        return -1;
    }

    in_core_memory2(core);
    in_core_memory1(core);
    in_core_execute_all(core);
    in_core_decode(core);

    /* After the instruction in decode reads forwarded value, clear
     * forwarding latches. This keeps the data on forwarding latches valid
     * for exactly one cycle */
    memset((void *)core->fwd_latch, 0, sizeof(DataFWDLatch) * NUM_FWD_BUS);
    in_core_fetch(core);
    return 0;
}