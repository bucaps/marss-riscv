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

    /* Create physical register write-back queues */
    core->prf_int_wb_queue
        = (WbQueueEntry *)calloc(p->prf_int_write_ports, sizeof(WbQueueEntry));
    assert(core->prf_int_wb_queue);

    core->prf_fp_wb_queue
        = (WbQueueEntry *)calloc(p->prf_fp_write_ports, sizeof(WbQueueEntry));
    assert(core->prf_fp_wb_queue);

    /* Create ROB */
    cq_init(&core->rob.cq, p->rob_size);
    core->rob.entries = (ROBEntry *)calloc(p->rob_size, sizeof(ROBEntry));
    assert(core->rob.entries);

    /* Create LSQ */
    cq_init(&core->lsq.cq, p->lsq_size);
    core->lsq.entries = (LSQEntry *)calloc(p->lsq_size, sizeof(LSQEntry));
    assert(core->lsq.entries);

    /* Create BIS */
    cq_init(&core->bis.cq, p->bis_size);
    core->bis.entries = (BISEntry *)calloc(p->bis_size, sizeof(BISEntry));
    assert(core->bis.entries);

    /* Create physical register files */
    core->prf_int = (PRFEntry *)calloc(p->prf_int_size, sizeof(PRFEntry));
    assert(core->prf_int);
    core->prf_fp = (PRFEntry *)calloc(p->prf_fp_size, sizeof(PRFEntry));
    assert(core->prf_fp);

    /* Create physical register free-lists */
    cq_init(&core->free_pr_int.cq, p->prf_int_size);
    core->free_pr_int.entries = (int *)calloc(p->prf_int_size, sizeof(int));
    assert(core->free_pr_int.entries);

    cq_init(&core->free_pr_fp.cq, p->prf_fp_size);
    core->free_pr_fp.entries = (int *)calloc(p->prf_fp_size, sizeof(int));
    assert(core->free_pr_fp.entries);

    /* Create issue-queues */
    core->iq_int
        = (IssueQueueEntry *)calloc(p->iq_int_size, sizeof(IssueQueueEntry));
    assert(core->iq_int);
    core->iq_fp
        = (IssueQueueEntry *)calloc(p->iq_fp_size, sizeof(IssueQueueEntry));
    assert(core->iq_fp);
    core->iq_mem
        = (IssueQueueEntry *)calloc(p->iq_mem_size, sizeof(IssueQueueEntry));
    assert(core->iq_mem);

    /* Create execution units */
    core->ialu = (CPUStage *)calloc(p->num_alu_stages, sizeof(CPUStage));
    assert(core->ialu);

    core->imul = (CPUStage *)calloc(p->num_mul_stages, sizeof(CPUStage));
    assert(core->imul);

    core->idiv = (CPUStage *)calloc(p->num_div_stages, sizeof(CPUStage));
    assert(core->idiv);

    core->fpu_alu = (CPUStage *)calloc(p->num_fpu_alu_stages, sizeof(CPUStage));
    assert(core->fpu_alu);

    core->fpu_fma = (CPUStage *)calloc(p->num_fpu_fma_stages, sizeof(CPUStage));
    assert(core->fpu_fma);

    core->simcpu = simcpu;
    return core;
}

void
oo_core_reset(void *core_type)
{
    int i, index;
    OOCore *core;
    int last_phy_reg_id;

    core = (OOCore *)core_type;

    /* Reset front-end stages */
    cpu_stage_flush(&core->fetch);
    cpu_stage_flush(&core->decode);
    cpu_stage_flush(&core->dispatch);

    /* Flush LSU */
    cpu_stage_flush(&core->lsu);

    /* To start fetching */
    core->fetch.has_data = TRUE;

    /* Reset INT and FP rename tables such that Xi maps to Pi */
    for (i = 0; i < NUM_INT_REG; ++i)
    {
        core->spec_rat_int[i] = i;
        core->commit_rat_int[i] = i;
        core->prf_int[i].valid = TRUE;
        core->prf_int[i].val = core->simcpu->emu_cpu_state->reg[i];
    }

    for (i = 0; i < NUM_FP_REG; ++i)
    {
        core->spec_rat_fp[i] = i;
        core->commit_rat_fp[i] = i;
        core->prf_fp[i].valid = TRUE;
        core->prf_fp[i].val = core->simcpu->emu_cpu_state->fp_reg[i];
    }

    /* Rest of the physical registers are added to
     * the free list. */
    cq_reset(&core->free_pr_int.cq);
    cq_reset(&core->free_pr_fp.cq);

    last_phy_reg_id
        = NUM_INT_REG + (core->simcpu->params->prf_int_size - NUM_INT_REG);
    for (i = NUM_INT_REG; i < last_phy_reg_id; ++i)
    {
        index = cq_enqueue(&core->free_pr_int.cq);
        core->free_pr_int.entries[index] = i;
        core->prf_int[i].valid = FALSE;
    }

    last_phy_reg_id
        = NUM_FP_REG + (core->simcpu->params->prf_fp_size - NUM_FP_REG);
    for (i = NUM_FP_REG; i < last_phy_reg_id; ++i)
    {
        index = cq_enqueue(&core->free_pr_fp.cq);
        core->free_pr_fp.entries[index] = i;
        core->prf_fp[i].valid = FALSE;
    }

    /* Reset remaining structures */
    memset((void *)core->prf_int_wb_queue, 0,
           core->simcpu->params->prf_int_write_ports * sizeof(WbQueueEntry));
    memset((void *)core->prf_fp_wb_queue, 0,
           core->simcpu->params->prf_fp_write_ports * sizeof(WbQueueEntry));

    cq_reset(&core->rob.cq);
    cq_reset(&core->lsq.cq);
    cq_reset(&core->bis.cq);

    iq_reset(core->iq_int, core->simcpu->params->iq_int_size);
    iq_reset(core->iq_fp, core->simcpu->params->iq_fp_size);
    iq_reset(core->iq_mem, core->simcpu->params->iq_mem_size);

    /* Reset execution units */
    exec_unit_flush(core->ialu, core->simcpu->params->num_alu_stages);
    exec_unit_flush(core->imul, core->simcpu->params->num_mul_stages);
    exec_unit_flush(core->idiv, core->simcpu->params->num_div_stages);
    exec_unit_flush(core->fpu_alu, core->simcpu->params->num_fpu_alu_stages);
    exec_unit_flush(core->fpu_fma, core->simcpu->params->num_fpu_fma_stages);

    /* Reset Data FWD latches */
    memset((void *)core->fwd_latch, 0, sizeof(DataFWDLatch) * NUM_FWD_BUS);
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
    free(core->prf_int);
    core->prf_int = NULL;
    free(core->prf_fp);
    core->prf_fp = NULL;
    free(core->free_pr_int.entries);
    core->free_pr_int.entries = NULL;
    free(core->free_pr_fp.entries);
    core->free_pr_fp.entries = NULL;
    free(core->prf_int_wb_queue);
    core->prf_int_wb_queue = NULL;
    free(core->prf_fp_wb_queue);
    core->prf_fp_wb_queue = NULL;
    free(core->rob.entries);
    core->rob.entries = NULL;
    free(core->lsq.entries);
    core->lsq.entries = NULL;
    free(core->iq_int);
    core->iq_int = NULL;
    free(core->iq_fp);
    core->iq_fp = NULL;
    free(core->iq_mem);
    core->iq_mem = NULL;
    free(core->ialu);
    core->ialu = NULL;
    free(core->imul);
    core->imul = NULL;
    free(core->idiv);
    core->idiv = NULL;
    free(core->fpu_alu);
    core->fpu_alu = NULL;
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
        core->simcpu->mmu->mem_controller->mem_controller_update_internal(
            core->simcpu->mmu->mem_controller);

        oo_core_writeback(core);
        oo_core_lsq(core);
        oo_core_lsu(core);

        /* Call lsq again to mark ROB entries as complete for memory
         * instructions which completed in a single cycle */
        oo_core_lsq(core);

        oo_core_execute_all(core);
        oo_core_issue(core);

        /* After the instructions in IQs reads forwarded value, clear
         * forwarding latches. This keeps the data on forwarding latches valid
         * for exactly one cycle */
        memset((void *)core->fwd_latch, 0, sizeof(DataFWDLatch) * NUM_FWD_BUS);

        oo_core_dispatch(core);
        oo_core_decode(core);
        oo_core_fetch(core);

        /* Do this so that the instructions which have completed PRF writeback
         * in the current cycle, are committed from the ROB in the same cycle */
        if (oo_core_rob_commit(core))
        {
            return core->simcpu->emu_cpu_state->sim_exception_cause;
        }

        /* Advance CPU clock */
        ++core->simcpu->clock;
        ++core->simcpu->stats[core->simcpu->emu_cpu_state->priv].cycles;
    }
}

static int
wb_queue_get_free_entry(WbQueueEntry *q, int max_size)
{
    int i, index = -1;

    for (i = 0; i < max_size; ++i)
    {
        if (!q[i].valid)
        {
            index = i;
            return index;
        }
    }

    return index;
}

int
send_phy_reg_write_request(WbQueueEntry *q, int max_size, IMapEntry *e)
{
    int wb_queue_idx = wb_queue_get_free_entry(q, max_size);

    if (wb_queue_idx < 0)
    {
        return -1;
    }

    q[wb_queue_idx].e = e;
    q[wb_queue_idx].valid = TRUE;
    return 0;
}