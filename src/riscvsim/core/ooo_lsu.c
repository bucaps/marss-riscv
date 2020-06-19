/**
 * Out of order core Load Store Unit
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
#include "ooo.h"
#include "../../riscv_cpu_priv.h"
#include "../utils/circular_queue.h"
#include "common_core_utils.h"
#include "riscv_sim_cpu.h"

void
oo_core_lsu(OOCore *core)
{
    InstructionLatch *e;
    RISCVCPUState *s = core->simcpu->emu_cpu_state;

    if (core->lsu.has_data)
    {
        e = get_insn_latch(s->simcpu->insn_latch_pool, core->lsu.insn_latch_index);
        if (!core->lsu.stage_exec_done)
        {
            s->hw_pg_tb_wlk_latency = 1;
            s->hw_pg_tb_wlk_stage_id = MEMORY;

            /* elasped_clock_cycles: number of CPU cycles spent by this instruction
             * in memory stage so far */
            e->elasped_clock_cycles = 1;

            if (execute_load_store(s, e))
            {
                /* This load, store or atomic instruction raised a page
                 * fault exception */
                e->ins.exception = TRUE;
                e->ins.exception_cause = SIM_MMU_EXCEPTION;

                /* In case of page fault, hardware page table walk has been done
                 * and its latency must be simulated */
                e->max_clock_cycles = s->hw_pg_tb_wlk_latency;
            }
            else
            {
                /* Memory access was successful, no page fault, so set the total
                 * number of CPU cycles required for memory instruction */
                e->max_clock_cycles = s->hw_pg_tb_wlk_latency
                                 + get_data_mem_access_latency(s, e);

                if (s->sim_params->enable_l1_caches)
                {
                    /* L1 caches and TLB are probed in parallel */
                    if (e->ins.is_load)
                    {
                        e->max_clock_cycles
                            -= min_int(s->hw_pg_tb_wlk_latency,
                                       s->simcpu->mem_hierarchy->dcache->read_latency);
                    }
                    if (e->ins.is_store)
                    {
                        e->max_clock_cycles
                            -= min_int(s->hw_pg_tb_wlk_latency,
                                       s->simcpu->mem_hierarchy->dcache->write_latency);
                    }
                    if (e->ins.is_atomic)
                    {
                        e->max_clock_cycles -= min_int(
                            s->hw_pg_tb_wlk_latency,
                            min_int(s->simcpu->mem_hierarchy->dcache->read_latency,
                                    s->simcpu->mem_hierarchy->dcache->write_latency));
                    }
                }
            }
            core->lsu.stage_exec_done = TRUE;
        }

        if (e->elasped_clock_cycles == e->max_clock_cycles)
        {
            /* Number of CPU cycles spent by this instruction in memory stage
             * equals memory access delay for this instruction */
            if (!s->simcpu->mem_hierarchy->mem_controller->backend_mem_access_queue.cur_size)
            {
                s->simcpu->mem_hierarchy->mem_controller->backend_mem_access_queue.cur_idx
                    = 0;
                core->lsq.entries[e->lsq_idx].mem_request_complete = TRUE;
                cpu_stage_flush(&core->lsu);
            }
            else
            {
                ++s->simcpu->stats[s->priv].data_mem_delay;
            }
        }
        else
        {
            e->elasped_clock_cycles++;
        }
    }
}

static void
process_lsq_entry_load(OOCore *core, LSQEntry *lsqe)
{
    InstructionLatch *e;

    e = lsqe->e;
    if (!lsqe->mem_request_sent)
    {
        if (!core->lsu.has_data)
        {
            /* Send request from LSQ top to LSU */
            core->lsu.has_data = TRUE;
            core->lsu.stage_exec_done = FALSE;
            core->lsu.insn_latch_index = e->insn_latch_index;
            lsqe->mem_request_sent = TRUE;
        }
    }
    else
    {
        /* Request was sent, waiting for memory access to complete */
        if (lsqe->mem_request_complete)
        {
            if (e->ins.exception)
            {
                /* MMU exception occurred, mark ROB entry valid */
                core->rob.entries[e->rob_idx].ready = TRUE;

                /* Stop fetching */
                cpu_stage_flush(&core->fetch);
                cpu_stage_flush(&core->decode);
                cpu_stage_flush(&core->dispatch);
            }
            else
            {
                if (e->ins.has_dest || e->ins.has_fp_dest)
                {
                    core->rob.entries[e->rob_idx].ready = TRUE;
                }
            }
            cq_dequeue(&core->lsq.cq);
        }
    }
}

static void
process_lsq_entry_store(OOCore *core, LSQEntry *lsqe)
{
    InstructionLatch *e;
    ROBEntry *rbe;

    e = lsqe->e;

    /* Extract ROB top */
    rbe = &core->rob.entries[cq_front(&core->rob.cq)];

    if (rbe->e->ins.pc == lsqe->e->ins.pc)
    {
        if (!lsqe->mem_request_sent)
        {
            if (!core->lsu.has_data)
            {
                /* Send request from LSQ top to LSU */
                core->lsu.has_data = TRUE;
                core->lsu.stage_exec_done = FALSE;
                core->lsu.insn_latch_index = e->insn_latch_index;
                lsqe->mem_request_sent = TRUE;
            }
        }
        else
        {
            /* Request was sent, waiting for memory access to
             * complete */
            if (lsqe->mem_request_complete)
            {
                if (e->ins.exception)
                {
                    /* Stop fetching */
                    cpu_stage_flush(&core->fetch);
                    cpu_stage_flush(&core->decode);
                    cpu_stage_flush(&core->dispatch);
                }

                /* Store is complete, remove its LSQ entry */
                cq_dequeue(&core->lsq.cq);

                /* Mark ROB entry valid */
                core->rob.entries[e->rob_idx].ready = TRUE;
            }
        }
    }
}

void
oo_core_lsq(OOCore *core)
{
    InstructionLatch *e;
    LSQEntry *lsqe;

    if (!cq_empty(&core->lsq.cq))
    {
        lsqe = &core->lsq.entries[cq_front(&core->lsq.cq)];
        e = lsqe->e;

        if (lsqe->ready == TRUE)
        {
            if (e->ins.is_load || e->ins.is_atomic)
            {
                process_lsq_entry_load(core, lsqe);
            }
            else if (e->ins.is_store)
            {
                process_lsq_entry_store(core, lsqe);
            }
        }
    }
}