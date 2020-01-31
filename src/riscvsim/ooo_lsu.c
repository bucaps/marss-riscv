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
#include "../riscv_cpu_priv.h"
#include "circular_queue.h"
#include "common_core_utils.h"
#include "riscv_sim_cpu.h"

void
oo_core_lsu(OOCore *core)
{
    IMapEntry *e;
    RISCVSIMCPUState *simcpu = core->simcpu;
    RISCVCPUState *s = core->simcpu->emu_cpu_state;

    if (core->lsu.has_data)
    {
        e = &simcpu->imap[core->lsu.imap_index];
        if (!core->lsu.stage_exec_done)
        {
            s->hw_pg_tb_wlk_latency = 1;
            s->hw_pg_tb_wlk_stage_id = MEMORY;

            /* current_latency: number of CPU cycles spent by this instruction
             * in memory stage so far */
            e->current_latency = 1;

            if (execute_load_store(s, e))
            {
                /* This load, store or atomic instruction raised a page
                 * fault exception */
                e->ins.exception = TRUE;
                e->ins.exception_cause = SIM_EXCEPTION;

                /* In case of page fault, hardware page table walk has been done
                 * and its latency must be simulated */
                e->max_latency = s->hw_pg_tb_wlk_latency;
            }
            else
            {
                /* Memory access was successful, no page fault, so set the total
                 * number of CPU cycles required for memory instruction */
                e->max_latency = s->hw_pg_tb_wlk_latency
                                 + get_data_mem_access_latency(s, e);

                if (s->sim_params->enable_l1_caches)
                {
                    /* L1 caches and TLB are probed in parallel */
                    e->max_latency
                        -= min_int(s->hw_pg_tb_wlk_latency,
                                   simcpu->mmu->dcache->probe_latency);
                }
            }
            core->lsu.stage_exec_done = TRUE;
        }

        if (e->current_latency == e->max_latency)
        {
            /* Number of CPU cycles spent by this instruction in memory stage
             * equals memory access delay for this instruction */
            if (!simcpu->mmu->mem_controller->backend_mem_access_queue.cur_size)
            {
                simcpu->mmu->mem_controller->backend_mem_access_queue.cur_idx
                    = 0;
                core->lsq.entries[e->lsq_idx].mem_request_complete = TRUE;
                cpu_stage_flush(&core->lsu);
            }
        }
        else
        {
            e->current_latency++;
        }
    }
}

static int
process_lsq_entry_load(OOCore *core, LSQEntry *lsqe)
{
    IMapEntry *e;

    e = lsqe->e;
    if (!lsqe->mem_request_sent)
    {
        if (!core->lsu.has_data)
        {
            /* Send request from LSQ top to LSU */
            core->lsu.has_data = TRUE;
            core->lsu.stage_exec_done = FALSE;
            core->lsu.imap_index = e->imap_index;
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
                /* Push the data read by loads/atomics on forwarding bus */
                if (!e->data_fwd_done
                    && ((e->ins.has_dest && e->ins.rd != 0)
                          || e->ins.has_fp_dest))
                {
                    /* (NUM_FWD_BUS - 1) is the ID of memory to IQ forwarding
                     * path */
                    core->fwd_latch[NUM_FWD_BUS - 1].rd = e->ins.pdest;
                    core->fwd_latch[NUM_FWD_BUS - 1].buffer = e->ins.buffer;
                    core->fwd_latch[NUM_FWD_BUS - 1].int_dest = e->ins.has_dest;
                    core->fwd_latch[NUM_FWD_BUS - 1].fp_dest
                        = e->ins.has_fp_dest;
                    core->fwd_latch[NUM_FWD_BUS - 1].valid = TRUE;
                    e->data_fwd_done = TRUE;
                }

                /* Write request to INT prf */
                if (e->ins.has_dest)
                {
                    if (send_phy_reg_write_request(&core->prf_int_wb_queue, e))
                    {
                        /* All write ports occupied */
                        return 0;
                    }
                }

                /* Write request to FP prf */
                if (e->ins.has_fp_dest)
                {
                    if (send_phy_reg_write_request(&core->prf_fp_wb_queue, e))
                    {
                        /* All write ports occupied */
                        return 0;
                    }
                }
            }

            /* Result read from memory is sent to wb queue, it is safe
             * to remove LSQ entry*/
            cq_dequeue(&core->lsq.cq);
            return 1;
        }
    }

    /* ROB entry for load stalls */
    return 0;
}

static int
process_lsq_entry_store(OOCore *core, LSQEntry *lsqe)
{
    IMapEntry *e;
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
                core->lsu.imap_index = e->imap_index;
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
                return 1;
            }
        }
    }

    /* Rob commit for store stalls */
    return 0;
}

void
oo_core_lsq(OOCore *core)
{
    IMapEntry *e;
    LSQEntry *lsqe;

    if (!cq_empty(&core->lsq.cq))
    {
        lsqe = &core->lsq.entries[cq_front(&core->lsq.cq)];
        e = lsqe->e;

        if (lsqe->ready == TRUE)
        {
            if (e->ins.is_load || e->ins.is_atomic)
            {
                if (!process_lsq_entry_load(core, lsqe))
                {
                    /* Active load */
                    return;
                }
            }
            else if (e->ins.is_store)
            {
                if (!process_lsq_entry_store(core, lsqe))
                {
                    /* Active Store */
                    return;
                }
            }
        }
    }
}
