/**
 * In-order Pipeline Back-end Stages: execute, memory and commit
 *
 * Copyright (c) 2016-2017 Fabrice Bellard
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "circular_queue.h"
#include "inorder.h"
#include "memory_controller.h"
#include "riscv_ins_execute_lib.h"
#include "../riscv_cpu_priv.h"
#include "riscv_sim_cpu.h"

/*=================================================
=            Instruction Execute Stage            =
=================================================*/

static CPUStage *
get_next_exec_stage(INCore *core, int cur_stage_id, int fu_type)
{
    CPUStage *stage = NULL;

    switch (fu_type)
    {
        case FU_ALU:
        {
            stage = &core->ialu[cur_stage_id + 1];
            break;
        }
        case FU_MUL:
        {
            stage = &core->imul[cur_stage_id + 1];
            break;
        }
        case FU_DIV:
        {
            stage = &core->idiv[cur_stage_id + 1];
            break;
        }
        case FU_FPU_FMA:
        {
            stage = &core->fpu_fma[cur_stage_id + 1];
            break;
        }
    }
    return stage;
}

static void
do_exec_insn(RISCVCPUState *s, INCore *core, IMapEntry *e, int fu_type)
{
    if (e->ins.has_dest)
    {
        if (e->ins.rd != 0)
        {
            core->int_reg_status[e->ins.rd] = FALSE;
        }
    }
    else if (e->ins.has_fp_dest)
    {
        core->fp_reg_status[e->ins.rd] = FALSE;
    }

    e->current_latency = 1;
    execute_riscv_instruction(&e->ins, &s->fflags);
    ++s->simcpu->stats[s->priv].fu_access[fu_type];
}

static void
do_fwd_data_from_ex_to_drf(INCore *core, IMapEntry *e, int fu_type)
{
    if (!e->data_fwd_done
        && !(e->ins.is_load || e->ins.is_store || e->ins.is_atomic)
        && !e->keep_dest_busy
        && ((e->ins.has_dest && e->ins.rd != 0) || e->ins.has_fp_dest))
    {
        core->fwd_latch[fu_type].rd = e->ins.rd;
        core->fwd_latch[fu_type].buffer = e->ins.buffer;
        core->fwd_latch[fu_type].int_dest = e->ins.has_dest;
        core->fwd_latch[fu_type].fp_dest = e->ins.has_fp_dest;
        core->fwd_latch[fu_type].valid = TRUE;
        e->data_fwd_done = TRUE;
    }
}

static void
push_insn_from_ex_to_mem(INCore *core, IMapEntry *e, CPUStage *stage)
{
    if (core->ins_dispatch_queue.data[cq_front(&core->ins_dispatch_queue.cq)]
        == e->ins_dispatch_id)
    {
        if (!core->memory.has_data)
        {
            cq_dequeue(&core->ins_dispatch_queue.cq);
            e->current_latency = 0;
            e->data_fwd_done = FALSE;
            stage->stage_exec_done = FALSE;
            core->memory = *stage;
            cpu_stage_flush(stage);
        }
    }
}

static void
in_core_execute_non_pipe(INCore *core, int fu_type, CPUStage *stage)
{
    IMapEntry *e;
    RISCVCPUState *s;

    s = core->simcpu->emu_cpu_state;
    if (stage->has_data)
    {
        e = get_imap_entry(s->simcpu->imap, stage->imap_index);
        ++s->simcpu->stats[s->priv].exec_unit_delay;
        if (!stage->stage_exec_done)
        {
            do_exec_insn(s, core, e, fu_type);
            e->max_latency = set_max_latency_for_non_pipe_fu(s, fu_type, e);
            assert(e->max_latency);
            stage->stage_exec_done = TRUE;
        }

        if (e->current_latency == e->max_latency)
        {
            do_fwd_data_from_ex_to_drf(core, e, fu_type);
            push_insn_from_ex_to_mem(core, e, stage);
        }
        else
        {
            e->current_latency++;
        }
    }
}

static void
in_core_execute_pipe(INCore *core, int cur_stage_id, int fu_type, CPUStage *stage,
                int max_latency, int max_stage_id)
{
    IMapEntry *e;
    CPUStage *next;
    RISCVCPUState *s;

    s = core->simcpu->emu_cpu_state;
    if (stage->has_data)
    {
        e = get_imap_entry(s->simcpu->imap, stage->imap_index);
        ++s->simcpu->stats[s->priv].exec_unit_delay;
        if (!stage->stage_exec_done)
        {
            do_exec_insn(s, core, e, fu_type);
            stage->stage_exec_done = TRUE;
        }

        if (e->current_latency == max_latency)
        {
            /* Instruction is in last stage of FU*/
            if (cur_stage_id == max_stage_id)
            {
                do_fwd_data_from_ex_to_drf(core, e, fu_type);
                push_insn_from_ex_to_mem(core, e, stage);
            }
            else
            {
                /* Pass the instruction into next stage for this FU */
                next = get_next_exec_stage(core, cur_stage_id, fu_type);
                if (!next->has_data)
                {
                    e->current_latency = 1;
                    *next = *stage;
                    cpu_stage_flush(stage);
                }
            }
        }
        else
        {
            e->current_latency++;
        }
    }
}

void
in_core_execute_all(INCore *core)
{
    int i;

    for (i = core->simcpu->params->num_fpu_fma_stages - 1; i >= 0; i--)
    {
        in_core_execute_pipe(core, i, FU_FPU_FMA, &core->fpu_fma[i],
                             core->simcpu->params->fpu_fma_stage_latency[i],
                             core->simcpu->params->num_fpu_fma_stages - 1);
    }
    in_core_execute_non_pipe(core, FU_FPU_ALU, &core->fpu_alu);
    for (i = core->simcpu->params->num_div_stages - 1; i >= 0; i--)
    {
        in_core_execute_pipe(core, i, FU_DIV, &core->idiv[i],
                             core->simcpu->params->div_stage_latency[i],
                             core->simcpu->params->num_div_stages - 1);
    }
    for (i = core->simcpu->params->num_mul_stages - 1; i >= 0; i--)
    {
        in_core_execute_pipe(core, i, FU_MUL, &core->imul[i],
                             core->simcpu->params->mul_stage_latency[i],
                             core->simcpu->params->num_mul_stages - 1);
    }
    for (i = core->simcpu->params->num_alu_stages - 1; i >= 0; i--)
    {
        in_core_execute_pipe(core, i, FU_ALU, &core->ialu[i],
                             core->simcpu->params->alu_stage_latency[i],
                             core->simcpu->params->num_alu_stages - 1);
    }
}

/*=====  End of Instruction Execute  ======*/

/*==========================================
=            Instruction Memory            =
==========================================*/
static void
flush_fu_stage(INCore *core, CPUStage *fu, int stages)
{
    int i;
    IMapEntry *e;

    for (i = 0; i < stages; ++i)
    {
        if (fu[i].has_data)
        {
            /* This resets the valid bits for INT and FP destination registers
             * on the speculated path */
            e = get_imap_entry(core->simcpu->imap, fu[i].imap_index);

            if (e->ins.has_dest)
            {
                core->int_reg_status[e->ins.rd] = TRUE;
            }
            else if (e->ins.has_fp_dest)
            {
                core->fp_reg_status[e->ins.rd] = TRUE;
            }
        }
        cpu_stage_flush(&fu[i]);
    }
}

static void
flush_speculated_cpu_state(INCore *core, IMapEntry *e)
{
    int i;
    RISCVCPUState *s = core->simcpu->emu_cpu_state;

    /* Send target PC to pcgen */
    s->code_ptr = NULL;
    s->code_end = NULL;
    s->code_to_pc_addend = e->branch_target;

    /* Keep track of this branch to resume at this target, in case the timeout
     * happens after this branch commits */
    e->is_branch_taken = TRUE;
    e->branch_target = e->branch_target;

    /* Flush all the preceding stages */
    cpu_stage_flush(&core->pcgen);
    cpu_stage_flush(&core->fetch);
    cpu_stage_flush(&core->decode);
    flush_fu_stage(core, core->ialu, s->simcpu->params->num_alu_stages);
    flush_fu_stage(core, core->imul, s->simcpu->params->num_mul_stages);
    flush_fu_stage(core, core->idiv, s->simcpu->params->num_div_stages);
    flush_fu_stage(core, &core->fpu_alu, 1);
    flush_fu_stage(core, core->fpu_fma, s->simcpu->params->num_fpu_fma_stages);

    /* Reset FU to MEM selector queue */
    cq_reset(&core->ins_dispatch_queue.cq);

    /* Flush FWD latches */
    memset((void *)core->fwd_latch, 0, sizeof(DataFWDLatch) * NUM_FWD_BUS);

    /* Flush memory controller queues on flush */
    mem_controller_reset(s->simcpu->mmu->mem_controller);

    /* To start fetching */
    core->pcgen.has_data = TRUE;

    /* Reset exception on speculated path */
    s->sim_exception = FALSE;

    /* Reset all the imap entries allocated on the speculated path */
    for (i = 0; i < NUM_IMAP_ENTRY; ++i)
    {
        if ((i != core->memory.imap_index) && (i != core->commit.imap_index))
        {
            s->simcpu->imap[i].status = IMAP_ENTRY_STATUS_FREE;
        }
    }
}

void
in_core_memory(INCore *core)
{
    IMapEntry *e;
    RISCVCPUState *s;

    s = core->simcpu->emu_cpu_state;
    if (core->memory.has_data)
    {
        e = get_imap_entry(s->simcpu->imap, core->memory.imap_index);
        if (!core->memory.stage_exec_done)
        {
            s->hw_pg_tb_wlk_latency = 1;
            s->hw_pg_tb_wlk_stage_id = MEMORY;

            /* current_latency: number of CPU cycles spent by this instruction
             * in memory stage so far */
            e->current_latency = 1;

            /* Set default total number of CPU cycles required for this
             * instruction in memory stage Note: This is the default latency for
             * non-memory instructions */
            e->max_latency = 1;

            if (e->ins.is_load || e->ins.is_store || e->ins.is_atomic)
            {
                if (execute_load_store(s, e))
                {
                    /* This load, store or atomic instruction raised a page
                     * fault exception */
                    e->ins.exception = TRUE;
                    e->ins.exception_cause = SIM_MMU_EXCEPTION;

                    /* In case of page fault, hardware page table walk has been
                       done and its latency must be simulated */
                    e->max_latency = s->hw_pg_tb_wlk_latency;

                    /* Safety check */
                    assert(e->max_latency);
                }
                else
                {
                    /* Memory access was successful, no page fault, so set the
                       total number of CPU cycles required for memory
                       instruction */
                    e->max_latency = s->hw_pg_tb_wlk_latency
                                     + get_data_mem_access_latency(s, e);

                    if (s->sim_params->enable_l1_caches)
                    {
                        /* L1 caches and TLB are probed in parallel */
                        if (e->ins.is_load)
                        {
                            e->max_latency
                                -= min_int(s->hw_pg_tb_wlk_latency,
                                           s->simcpu->mmu->dcache->read_latency);
                        }
                        if (e->ins.is_store)
                        {
                            e->max_latency
                                -= min_int(s->hw_pg_tb_wlk_latency,
                                           s->simcpu->mmu->dcache->write_latency);
                        }
                        if (e->ins.is_atomic)
                        {
                            e->max_latency -= min_int(
                                s->hw_pg_tb_wlk_latency,
                                min_int(s->simcpu->mmu->dcache->read_latency,
                                        s->simcpu->mmu->dcache->write_latency));
                        }
                    }
                }
            }
            else if (e->ins.is_branch)
            {
                if (s->simcpu->pfn_branch_handler(s, e))
                {
                    flush_speculated_cpu_state(core, e);
                }
            }

            core->memory.stage_exec_done = TRUE;
        }

        if (e->current_latency == e->max_latency)
        {
            /* Number of CPU cycles spent by this instruction in memory stage
               equals memory access delay for this instruction */

            /* Lookup latency is completed, now waiting for pending DRAM
             * accesses */
            if ((e->ins.is_load || e->ins.is_store || e->ins.is_atomic))
            {
                if (s->simcpu->mmu->mem_controller->backend_mem_access_queue
                        .cur_size)
                {
                    ++s->simcpu->stats[s->priv].data_mem_delay;
                    return;
                }
            }

            /* MMU exception */
            if (e->ins.exception)
            {
                set_exception_state(s, e);
                cpu_stage_flush(&core->pcgen);
                cpu_stage_flush(&core->fetch);
                cpu_stage_flush(&core->decode);
                cpu_stage_flush(&core->memory);
                exec_unit_flush(core->ialu,
                                s->simcpu->params->num_alu_stages);
                exec_unit_flush(core->imul,
                                s->simcpu->params->num_mul_stages);
                exec_unit_flush(core->idiv,
                                s->simcpu->params->num_div_stages);
                exec_unit_flush(&core->fpu_alu, 1);
                exec_unit_flush(core->fpu_fma,
                                s->simcpu->params->num_fpu_fma_stages);
                return;
            }

            /* Push the data read by loads/atomics on forwarding bus*/
            if (!e->ins.exception && !e->data_fwd_done && !e->keep_dest_busy
                && ((e->ins.has_dest && e->ins.rd != 0) || e->ins.has_fp_dest))
            {
                core->fwd_latch[NUM_FWD_BUS-1].rd = e->ins.rd;
                core->fwd_latch[NUM_FWD_BUS-1].buffer = e->ins.buffer;
                core->fwd_latch[NUM_FWD_BUS-1].int_dest = e->ins.has_dest;
                core->fwd_latch[NUM_FWD_BUS-1].fp_dest = e->ins.has_fp_dest;
                core->fwd_latch[NUM_FWD_BUS-1].valid = TRUE;
                e->data_fwd_done = TRUE;
            }

            /* If the next stage is available, send this instruction to next
             * stage, else stall memory stage */
            if (!core->commit.has_data)
            {
                s->simcpu->mmu->mem_controller->backend_mem_access_queue.cur_idx = 0;
                core->memory.stage_exec_done = FALSE;
                e->max_latency = 0;
                e->current_latency = 0;
                e->data_fwd_done = FALSE;
                core->commit = core->memory;
                cpu_stage_flush(&core->memory);
            }
        }
        else
        {
            e->current_latency++;
        }
    }
}

/*=====  End of Instruction Memory  ======*/

/*================================================
=            Instruction Commit Stage            =
================================================*/

int
in_core_commit(INCore *core)
{
    IMapEntry *e;
    RISCVCPUState *s;

    s = core->simcpu->emu_cpu_state;
    if (core->commit.has_data)
    {
        e = get_imap_entry(s->simcpu->imap, core->commit.imap_index);

        if (e->ins.has_dest)
        {
            /* Update the integer register if rd is not x0 */
            if (e->ins.rd)
            {
                update_arch_reg_int(s, e);
                if (!e->keep_dest_busy)
                {
                    core->int_reg_status[e->ins.rd] = TRUE;
                }
            }
        }
        else if (e->ins.has_fp_dest)
        {
            update_arch_reg_fp(s, e);
            if (!e->keep_dest_busy)
            {
                core->fp_reg_status[e->ins.rd] = TRUE;
            }
        }

        update_insn_commit_stats(s, e);

        /* Dump commit trace if trace mode enabled */
        if (s->simcpu->params->do_sim_trace)
        {
            setup_sim_trace_pkt(s, e);
        }

        if (s->sim_params->enable_stats_display)
        {
            write_stats_to_stats_display_shm(s);
        }

        /* Commit success */
        e->status = IMAP_ENTRY_STATUS_FREE;
        cpu_stage_flush(&core->commit);

        /* Check for timeout */
        if ((--s->sim_n_cycles) == 0)
        {
            set_timer_exception_state(s, e);
            return -1;
        }
    }
    return 0;
}
/*=====  End of Instruction Commit Stage  ======*/
