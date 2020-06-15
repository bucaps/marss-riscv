/**
 * Out Of Order Core Top Level Functions
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2017-2019 Gaurav Kothari {gkothar1@binghamton.edu}
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
#include <stdlib.h>

#include "../cutils.h"
#include "../riscv_cpu_priv.h"
#include "circular_queue.h"
#include "ooo.h"
#include "riscv_sim_cpu.h"

OOCore *
oo_core_init(const SimParams *p, struct RISCVSIMCPUState *simcpu)
{
    OOCore *core;

    core = calloc(1, sizeof(OOCore));
    assert(core);

    /* Create ROB */
    cq_init(&core->rob.cq, p->rob_size);
    core->rob.entries = (ROBEntry *)calloc(p->rob_size, sizeof(ROBEntry));
    assert(core->rob.entries);

    /* Create LSQ */
    cq_init(&core->lsq.cq, p->lsq_size);
    core->lsq.entries = (LSQEntry *)calloc(p->lsq_size, sizeof(LSQEntry));
    assert(core->lsq.entries);

    /* Create Rename tables */
    core->int_rat
        = (RenameTableEntry *)calloc(NUM_INT_REG, sizeof(RenameTableEntry));
    core->fp_rat
        = (RenameTableEntry *)calloc(NUM_FP_REG, sizeof(RenameTableEntry));

    /* Create IQ */
    core->iq
        = (IssueQueueEntry *)calloc(p->iq_size, sizeof(IssueQueueEntry));

    /* Create execution units */
    core->ialu = (CPUStage *)calloc(p->num_alu_stages, sizeof(CPUStage));
    assert(core->ialu);

    core->imul = (CPUStage *)calloc(p->num_mul_stages, sizeof(CPUStage));
    assert(core->imul);

    core->idiv = (CPUStage *)calloc(p->num_div_stages, sizeof(CPUStage));
    assert(core->idiv);

    core->fpu_fma = (CPUStage *)calloc(p->num_fpu_fma_stages, sizeof(CPUStage));
    assert(core->fpu_fma);

    core->simcpu = simcpu;
    return core;
}

void
oo_core_reset(void *core_type)
{
    int i;
    OOCore *core;

    core = (OOCore *)core_type;

    core->ins_dispatch_id = 0;

    /* Reset front-end stages */
    cpu_stage_flush(&core->fetch);
    cpu_stage_flush(&core->decode);
    cpu_stage_flush(&core->dispatch);

    /* Flush LSU */
    cpu_stage_flush(&core->lsu);

    /* To start fetching */
    core->fetch.has_data = TRUE;

    /* Reset rename tables */
    for (i = 0; i < NUM_INT_REG; ++i)
    {
        core->int_rat[i].rob_idx = -1;
        core->int_rat[i].read_from_rob = FALSE;
    }

    for (i = 0; i < NUM_FP_REG; ++i)
    {
        core->fp_rat[i].rob_idx = -1;
        core->fp_rat[i].read_from_rob = FALSE;
    }

    cq_reset(&core->rob.cq);
    cq_reset(&core->lsq.cq);
    iq_reset(core->iq, core->simcpu->params->iq_size);

    /* Reset execution units */
    exec_unit_flush(core->ialu, core->simcpu->params->num_alu_stages);
    exec_unit_flush(core->imul, core->simcpu->params->num_mul_stages);
    exec_unit_flush(core->idiv, core->simcpu->params->num_div_stages);
    exec_unit_flush(core->fpu_fma, core->simcpu->params->num_fpu_fma_stages);
    cpu_stage_flush(&core->fpu_alu);
}

void
iq_reset(IssueQueueEntry *iq_entry, int size)
{
    int i;

    for (i = 0; i < size; ++i)
    {
        iq_entry[i].valid = FALSE;
        iq_entry[i].ready = FALSE;
        iq_entry[i].e = NULL;
    }
}

int
iq_full(IssueQueueEntry *iq, int size)
{
    int i;

    for (i = 0; i < size; ++i)
    {
        if (iq[i].valid == FALSE)
        {
            return FALSE;
        }
    }

    return TRUE;
}

int
iq_get_free_entry(IssueQueueEntry *iq, int size)
{
    int i;

    for (i = 0; i < size; ++i)
    {
        if (iq[i].valid == FALSE)
        {
            return i;
        }
    }

    assert(0);
    return 0;
}

void
oo_core_free(void *core_type)
{
    OOCore *core;

    core = (OOCore *)(*((OOCore **)core_type));
    free(core->int_rat);
    core->int_rat = NULL;
    free(core->fp_rat);
    core->fp_rat = NULL;
    free(core->rob.entries);
    core->rob.entries = NULL;
    free(core->lsq.entries);
    core->lsq.entries = NULL;
    free(core->iq);
    core->iq = NULL;
    free(core->ialu);
    core->ialu = NULL;
    free(core->imul);
    core->imul = NULL;
    free(core->idiv);
    core->idiv = NULL;
    free(core->fpu_fma);
    core->fpu_fma = NULL;
    free(core);
}

int
oo_core_run(void *core_type)
{
    OOCore *core;

    core = (OOCore *)core_type;
    while (1)
    {
        /* Advance DRAM clock */
        core->simcpu->mmu->mem_controller->clock(
            core->simcpu->mmu->mem_controller);

        if (oo_core_rob_commit(core))
        {
            return core->simcpu->emu_cpu_state->sim_exception_cause;
        }

        oo_core_lsq(core);
        oo_core_lsu(core);

        /* Call lsq again to mark ROB entries as complete for memory
         * instructions which completed in a single cycle */
        oo_core_lsq(core);

        oo_core_execute_all(core);
        oo_core_issue(core);
        oo_core_dispatch(core);
        oo_core_decode(core);
        oo_core_fetch(core);

        /* Advance CPU clock */
        ++core->simcpu->clock;
        ++core->simcpu->stats[core->simcpu->emu_cpu_state->priv].cycles;
    }
}

/** Optimize */
/** When trying to read operand from ROB index, this checks whether the given
 * ROB index is already committed  */
int
rob_entry_committed(ROB *rob, int src_idx, int current_idx)
{
    if (rob->cq.rear >= rob->cq.front)
    {
        if ((src_idx >= rob->cq.front) && (src_idx < current_idx))
        {
            return FALSE;
        }
    }
    else
    {
        if ((current_idx >= rob->cq.front) && (current_idx < rob->cq.max_size)
            && (src_idx >= rob->cq.front) && (src_idx < rob->cq.max_size) && (src_idx < current_idx))
        {
            return FALSE;
        }
        if ((current_idx >= 0) && (current_idx <= rob->cq.rear)
            && (((src_idx >= rob->cq.front) && (src_idx < rob->cq.max_size))
                || (src_idx < current_idx)))
        {
            return FALSE;
        }
    }
    return TRUE;
}

void
read_int_operand_from_rob_slot(OOCore *core, IMapEntry *e, int arch_src,
                               int src_rob_idx, int current_rob_idx,
                               uint64_t *buffer, int *read_flag)
{
    ROBEntry *rbe;

    assert(src_rob_idx != current_rob_idx);

    if (rob_entry_committed(&core->rob, src_rob_idx, current_rob_idx))
    {
        /* ROB entry we are trying to read from is committed, so read value from
         * register file */
        *buffer = core->simcpu->emu_cpu_state->reg[arch_src];
        *read_flag = TRUE;
    }
    else
    {
        /* ROB entry we are trying to read from is not yet committed*/
        rbe = &core->rob.entries[src_rob_idx];
        if (rbe->ready && !rbe->e->ins.exception && rbe->e->ins.has_dest
            && (rbe->e->ins.rd == arch_src))
        {
            *buffer = rbe->e->ins.buffer;
            *read_flag = TRUE;
        }
    }
}

void
read_fp_operand_from_rob_slot(OOCore *core, IMapEntry *e, int arch_src,
                               int src_rob_idx, int current_rob_idx,
                               uint64_t *buffer, int *read_flag)
{
    ROBEntry *rbe;

    assert(src_rob_idx != current_rob_idx);

    if (rob_entry_committed(&core->rob, src_rob_idx, current_rob_idx))
    {
        /* ROB entry we are trying to read from is committed, so read value from
         * register file */
        *buffer = core->simcpu->emu_cpu_state->fp_reg[arch_src];
        *read_flag = TRUE;
    }
    else
    {
        /* ROB entry we are trying to read from is not yet committed*/
        rbe = &core->rob.entries[src_rob_idx];
        if (rbe->ready & !rbe->e->ins.exception && rbe->e->ins.has_fp_dest
            && (rbe->e->ins.rd == arch_src))
        {
            *buffer = rbe->e->ins.buffer;
            *read_flag = TRUE;
        }
    }
}