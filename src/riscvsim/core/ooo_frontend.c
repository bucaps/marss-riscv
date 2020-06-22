/**
 * Out of order core front-end stages: fetch, decode, dispatch
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2017-2020 Gaurav Kothari {gkothar1@binghamton.edu}
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
#include "riscv_sim_cpu.h"

/*===============================================
=            Instruction Fetch Stage            =
===============================================*/

void
oo_core_fetch(OOCore *core)
{
    InstructionLatch *e;
    RISCVCPUState *s;

    s = core->simcpu->emu_cpu_state;
    if (core->fetch.has_data)
    {
        if (!core->fetch.stage_exec_done)
        {
            /* Calculate current PC*/
            s->simcpu->pc
                = (target_ulong)((uintptr_t)s->code_ptr + s->code_to_pc_addend);

            e = insn_latch_allocate(s->simcpu->insn_latch_pool);

            /* Setup the allocated latch */
            e->ins.pc = s->simcpu->pc;
            e->ins.create_str = s->sim_params->create_ins_str;

            /* Store latch index in the stage and the actual decoded instruction
             * info is stored in this latch entry. This avoids copying of whole
             * decoded instruction info when instruction flows to next stage */
            core->fetch.insn_latch_index = e->insn_latch_index;

            fetch_cpu_stage_exec(s, e);
            core->fetch.stage_exec_done = TRUE;
        }
        else
        {
            e = get_insn_latch(s->simcpu->insn_latch_pool, core->fetch.insn_latch_index);
        }

        if (e->elasped_clock_cycles == e->max_clock_cycles)
        {
            /* Number of CPU cycles spent by this instruction in fetch stage
             * equals cache lookup delay for this instruction */

            /* Wait on memory controller callback for any pending memory
             * accesses */
            if (!s->simcpu->mem_hierarchy->mem_controller
                     ->frontend_mem_access_queue.cur_size)
            {
                /* If the next stage is available, send this instruction to next
                   stage, else stall fetch */
                if (!core->decode.has_data)
                {
                    s->simcpu->mem_hierarchy->mem_controller->frontend_mem_access_queue
                        .cur_idx
                        = 0;

                    core->fetch.stage_exec_done = FALSE;
                    e->max_clock_cycles = 0;
                    e->elasped_clock_cycles = 0;
                    core->decode = core->fetch;
                    core->fetch.insn_latch_index = -1;

                    /* Stop fetching new instructions on a MMU exception */
                    if (e->ins.exception)
                    {
                        cpu_stage_flush(&core->fetch);
                    }
                }
            }
            else
            {
                ++s->simcpu->stats[s->priv].insn_mem_delay;
            }
        }
        else
        {
            e->elasped_clock_cycles++;
        }
    }
}

/*=====  End of Instruction Fetch Stage  ======*/

/*================================================
=            Instruction Decode Stage            =
================================================*/

void
oo_core_decode(OOCore *core)
{
    InstructionLatch *e;
    RISCVCPUState *s;

    s = core->simcpu->emu_cpu_state;
    if (core->decode.has_data)
    {
        e = get_insn_latch(s->simcpu->insn_latch_pool, core->decode.insn_latch_index);

        if (!core->decode.stage_exec_done)
        {
            if (!e->ins.exception && !e->is_decoded)
            {
                decode_cpu_stage_exec(s, e);
                if (s->simcpu->bpu_decode_stage_handler(s, e))
                {
                    cpu_stage_flush_free_insn_latch(&core->fetch, s->simcpu->insn_latch_pool);
                    core->fetch.has_data = TRUE;
                }
                e->is_decoded = TRUE;
            }

            /* Handle exception caused during decoding */
            if (unlikely(e->ins.exception))
            {
                cpu_stage_flush(&core->fetch);
            }

            core->decode.stage_exec_done = TRUE;
        }

        if (!core->dispatch.has_data)
        {
            core->decode.stage_exec_done = FALSE;
            core->dispatch = core->decode;
            cpu_stage_flush(&core->decode);
        }
    }
}

/*=====  End of Instruction Decode Stage  ======*/

/*==================================================
=            Instruction Dispatch Stage            =
==================================================*/

static void
rob_entry_create(ROB *rob, InstructionLatch *e, int rob_entry_status)
{
    int rob_idx;

    rob_idx = cq_enqueue(&rob->cq);
    e->rob_idx = rob_idx;
    rob->entries[rob_idx].ready = rob_entry_status;
    rob->entries[rob_idx].e = e;
}

static void
iq_entry_create(IssueQueueEntry *iq, int iq_size, InstructionLatch *e)
{
    int iq_idx;

    iq_idx = iq_get_free_entry(iq, iq_size);
    e->iq_idx = iq_idx;
    iq[iq_idx].valid = TRUE;
    iq[iq_idx].ready = FALSE;
    iq[iq_idx].e = e;
}

static void
lsq_entry_create(LSQ *lsq, InstructionLatch *e)
{
    int lsq_idx;

    lsq_idx = cq_enqueue(&lsq->cq);
    e->lsq_idx = lsq_idx;
    lsq->entries[lsq_idx].ready = FALSE;
    lsq->entries[lsq_idx].mem_request_sent = FALSE;
    lsq->entries[lsq_idx].mem_request_complete = FALSE;
    lsq->entries[lsq_idx].e = e;
}

static void
update_rd_rat_mapping(OOCore *core, InstructionLatch *e)
{
    /* INT destination */
    if (e->ins.has_dest)
    {
        if (e->ins.rd)
        {
            e->ins.old_pdest = core->int_rat[e->ins.rd].rob_idx;
            core->int_rat[e->ins.rd].read_from_rob = TRUE;
            core->int_rat[e->ins.rd].rob_idx = e->rob_idx;
            assert(e->ins.old_pdest != core->int_rat[e->ins.rd].rob_idx);
        }
    }
    else if (e->ins.has_fp_dest)
    {
        e->ins.old_pdest = core->fp_rat[e->ins.rd].rob_idx;
        core->fp_rat[e->ins.rd].read_from_rob = TRUE;
        core->fp_rat[e->ins.rd].rob_idx = e->rob_idx;
        assert(e->ins.old_pdest != core->fp_rat[e->ins.rd].rob_idx);
    }
}

static int
stall_insn_dispatch(const OOCore *core, const InstructionLatch *e)
{
    /* Before dispatching atomic instruction, make sure that all the prior
     * instructions are committed */
    if (e->ins.is_atomic && !cq_empty(&core->rob.cq))
    {
        return TRUE;
    }

    if (cq_full(&core->rob.cq) || iq_full(core->iq, core->simcpu->params->iq_size)
        || ((e->ins.is_load || e->ins.is_store || e->ins.is_atomic)
            && cq_full(&core->lsq.cq)))
    {
        return TRUE;
    }

    /* Ready to dispatch */
    return FALSE;
}

static void
do_insn_rename_and_read_reg_file(OOCore *core, InstructionLatch *e)
{
    if (e->ins.has_src1)
    {
        if (core->int_rat[e->ins.rs1].read_from_rob)
        {
            e->ins.prs1 = core->int_rat[e->ins.rs1].rob_idx;
            assert(e->ins.prs1 != -1);
        }
        else
        {
            e->ins.rs1_val = core->simcpu->emu_cpu_state->reg[e->ins.rs1];
            e->read_rs1 = TRUE;
        }
    }
    else if (e->ins.has_fp_src1)
    {
        if (core->fp_rat[e->ins.rs1].read_from_rob)
        {
            e->ins.prs1 = core->fp_rat[e->ins.rs1].rob_idx;
            assert(e->ins.prs1 != -1);
        }
        else
        {
            e->ins.rs1_val = core->simcpu->emu_cpu_state->fp_reg[e->ins.rs1];
            e->read_rs1 = TRUE;
        }
    }
    else
    {
        /* Do this, so that this instruction won't wait for rs1,
         * while in IQ */
        e->read_rs1 = TRUE;
    }

    if (e->ins.has_src2)
    {
        if (core->int_rat[e->ins.rs2].read_from_rob)
        {
            e->ins.prs2 = core->int_rat[e->ins.rs2].rob_idx;
            assert(e->ins.prs2 != -1);
        }
        else
        {
            e->ins.rs2_val = core->simcpu->emu_cpu_state->reg[e->ins.rs2];
            e->read_rs2 = TRUE;
        }
    }
    else if (e->ins.has_fp_src2)
    {
        if (core->fp_rat[e->ins.rs2].read_from_rob)
        {
            e->ins.prs2 = core->fp_rat[e->ins.rs2].rob_idx;
            assert(e->ins.prs2 != -1);
        }
        else
        {
            e->ins.rs2_val = core->simcpu->emu_cpu_state->fp_reg[e->ins.rs2];
            e->read_rs2 = TRUE;
        }
    }
    else
    {
        e->read_rs2 = TRUE;
    }

    /* Only FP-FMA instructions have rs3 */
    if (e->ins.has_fp_src3)
    {
        if (core->fp_rat[e->ins.rs3].read_from_rob)
        {
            e->ins.prs3 = core->fp_rat[e->ins.rs3].rob_idx;
            assert(e->ins.prs3 != -1);
        }
        else
        {
            e->ins.rs3_val = core->simcpu->emu_cpu_state->fp_reg[e->ins.rs3];
            e->read_rs3 = TRUE;
        }
    }
    else
    {
        e->read_rs3 = TRUE;
    }
}

void
oo_core_dispatch(OOCore *core)
{
    InstructionLatch *e;
    RISCVCPUState *s;

    s = core->simcpu->emu_cpu_state;
    if (core->dispatch.has_data)
    {
        e = get_insn_latch(s->simcpu->insn_latch_pool, core->dispatch.insn_latch_index);
        if (!core->dispatch.stage_exec_done)
        {
            /* If this instruction has caused an exception, only create ROB
             * entry for this instruction and let ROB handle this exception */
            if (e->ins.exception)
            {
                if (cq_full(&core->rob.cq))
                {
                    /* Stall */
                    return;
                }

                /* Mark ROB entry valid so that its processed immediately once
                 * it becomes ROB top */
                rob_entry_create(&core->rob, e, TRUE);
            }
            else
            {
                if (stall_insn_dispatch(core, e))
                {
                    return;
                }
                do_insn_rename_and_read_reg_file(core, e);
                rob_entry_create(&core->rob, e, FALSE);
                iq_entry_create(core->iq, s->simcpu->params->iq_size, e);
                if (e->ins.is_load || e->ins.is_store || e->ins.is_atomic)
                {
                    lsq_entry_create(&core->lsq, e);
                }
                update_rd_rat_mapping(core, e);
            }
            e->ins_dispatch_id = core->ins_dispatch_id++;
            cpu_stage_flush(&core->dispatch);
        }
    }
}
/*=====  End of Instruction Dispatch Stage  ======*/
