/**
 * In-order Pipelined Memory stage
 *
 * Copyright (c) 2016-2017 Fabrice Bellard
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../riscv_cpu_priv.h"
#include "../memory_hierarchy/memory_controller.h"
#include "../utils/circular_queue.h"
#include "inorder.h"
#include "riscv_sim_cpu.h"

/*==========================================
=            Instruction Memory            =
==========================================*/
static void
flush_fu_stage(INCore *core, CPUStage *fu, int stages)
{
    int i;
    InstructionLatch *e;

    for (i = 0; i < stages; ++i)
    {
        if (fu[i].has_data)
        {
            /* Reset the valid bits for INT, and FP destination registers on the
             * speculated path */
            e = get_insn_latch(core->simcpu->insn_latch_pool,
                               fu[i].insn_latch_index);

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
flush_speculated_cpu_state(INCore *core, InstructionLatch *e)
{
    int i;
    RISCVCPUState *s = core->simcpu->emu_cpu_state;

    /* Send target PC to fetch */
    s->code_ptr = NULL;
    s->code_end = NULL;
    s->code_to_pc_addend = e->branch_target;

    /* Keep track of this branch to resume at this target, in case the timeout
     * happens after this branch commits */
    e->is_branch_taken = TRUE;
    e->branch_target = e->branch_target;

    /* Flush all the preceding stages */
    cpu_stage_flush(&core->fetch);
    cpu_stage_flush(&core->decode);

    /* Flush all the functional unit and reset rd valid bit */
    flush_fu_stage(core, core->ialu, s->simcpu->params->num_alu_stages);
    flush_fu_stage(core, core->imul, s->simcpu->params->num_mul_stages);
    flush_fu_stage(core, core->idiv, s->simcpu->params->num_div_stages);
    flush_fu_stage(core, &core->fpu_alu, 1);
    flush_fu_stage(core, core->fpu_fma, s->simcpu->params->num_fpu_fma_stages);

    /* Reset FU to MEM selector queue */
    cq_reset(&core->ex_to_mem_queue.cq);

    /* Flush FWD latches */
    memset((void *)core->fwd_latch, 0, sizeof(DataFWDLatch) * NUM_FWD_BUS);

    /* Flush memory controller queues on flush */
    mem_controller_reset(s->simcpu->mem_hierarchy->mem_controller);

    /* To start fetching */
    core->fetch.has_data = TRUE;
    core->fetch.stage_exec_done = FALSE;

    /* To start fetching target instruction from next cycle */
    core->simcpu->skip_fetch_cycle = TRUE;

    /* Reset exception on speculated path */
    s->simcpu->exception->pending = FALSE;

    /* Reset all the insn_latch_pool entries allocated on the speculated path */
    for (i = 0; i < INSN_LATCH_POOL_SIZE; ++i)
    {
        if ((i != core->memory1.insn_latch_index)
            && (i != core->memory2.insn_latch_index)
            && (i != core->commit.insn_latch_index))
        {
            s->simcpu->insn_latch_pool[i].status = INSN_LATCH_FREE;
        }
    }
}

static void
adjust_latency_hack(INCore *core, InstructionLatch *e)
{
    if ((e->ins.is_load || e->ins.is_atomic)
        & core->simcpu->params->enable_l1_caches)
    {
        e->max_clock_cycles -= core->simcpu->params->l1_data_cache_read_latency;

        if (e->max_clock_cycles <= 0)
        {
            e->max_clock_cycles = 1;
        }
    }
}

static int
push_insn_from_memory1_to_commit(INCore *core, InstructionLatch *e)
{
    InstructionLatch *e1;

    /* Check if memory 2 has earlier instruction */
    if (core->memory2.has_data)
    {
        e1 = get_insn_latch(core->simcpu->insn_latch_pool,
                            core->memory2.insn_latch_index);

        if (e1->ins_dispatch_id < e->ins_dispatch_id)
        {
            return FALSE;
        }
    }

    return TRUE;
}

/**
 * Memory 1 stage: main stage for all the instructions
 */
void
in_core_memory1(INCore *core)
{
    InstructionLatch *e;
    RISCVCPUState *s;

    s = core->simcpu->emu_cpu_state;
    if (core->memory1.has_data)
    {
        e = get_insn_latch(s->simcpu->insn_latch_pool,
                           core->memory1.insn_latch_index);
        if (!core->memory1.stage_exec_done)
        {
            e->elasped_clock_cycles = 1;
            e->max_clock_cycles = 1;

            if (e->ins.is_load || e->ins.is_store || e->ins.is_atomic)
            {
                mem_cpu_stage_exec(s, e);
            }
            else if (e->ins.is_branch)
            {
                if (s->simcpu->bpu_execute_stage_handler(s, e))
                {
                    flush_speculated_cpu_state(core, e);
                }
            }

            /* Hacky way to adjust latency, please verify */
            adjust_latency_hack(core, e);

            /* Load for non-word quantities such as byte or half word take an
             * extra cycle */
            if ((e->ins.is_load) && (e->ins.bytes_to_rw < 4))
            {
                e->max_clock_cycles += 1;
            }

            core->memory1.stage_exec_done = TRUE;
        }

        if (e->elasped_clock_cycles == e->max_clock_cycles)
        {
            if (s->simcpu->mem_hierarchy->mem_controller
                    ->backend_mem_access_queue.cur_size
                && !e->cache_lookup_complete_signal_sent)
            {
                mem_controller_cache_lookup_complete_signal(
                    s->simcpu->mem_hierarchy->mem_controller,
                    &s->simcpu->mem_hierarchy->mem_controller
                         ->backend_mem_access_queue);
                e->cache_lookup_complete_signal_sent = TRUE;
            }

            if ((e->ins.is_load || e->ins.is_store || e->ins.is_atomic))
            {
                /* Wait on memory controller callback for any pending memory
                 * accesses */
                if (s->simcpu->mem_hierarchy->mem_controller
                        ->backend_mem_access_queue.cur_size)
                {
                    ++s->simcpu->stats[s->priv].data_mem_delay;
                    return;
                }
            }

            /* MMU exception */
            if (e->ins.exception)
            {
                sim_exception_set(s->simcpu->exception, e);
                cpu_stage_flush(&core->fetch);
                cpu_stage_flush(&core->decode);
                cpu_stage_flush(&core->memory1);
                cpu_stage_flush_pipe(core->ialu,
                                     s->simcpu->params->num_alu_stages);
                cpu_stage_flush_pipe(core->imul,
                                     s->simcpu->params->num_mul_stages);
                cpu_stage_flush_pipe(core->idiv,
                                     s->simcpu->params->num_div_stages);
                cpu_stage_flush_pipe(&core->fpu_alu, 1);
                cpu_stage_flush_pipe(core->fpu_fma,
                                     s->simcpu->params->num_fpu_fma_stages);
                return;
            }

            /* Push the data read by loads/atomics on forwarding bus*/
            if (!e->ins.exception && !e->data_fwd_done && !e->keep_dest_busy
                && !(e->ins.is_load || e->ins.is_atomic)
                && ((e->ins.has_dest && e->ins.rd != 0) || e->ins.has_fp_dest))
            {
                core->fwd_latch[NUM_FWD_BUS - 2].rd = e->ins.rd;
                core->fwd_latch[NUM_FWD_BUS - 2].buffer = e->ins.buffer;
                core->fwd_latch[NUM_FWD_BUS - 2].int_dest = e->ins.has_dest;
                core->fwd_latch[NUM_FWD_BUS - 2].fp_dest = e->ins.has_fp_dest;
                core->fwd_latch[NUM_FWD_BUS - 2].valid = TRUE;
                e->data_fwd_done = TRUE;
            }

            /* If the commit stage is available, send this instruction to commit
             * stage, else stall memory stage */
            if (e->ins.is_load || e->ins.is_atomic)
            {
                if (!core->memory2.has_data)
                {
                    s->simcpu->mem_hierarchy->mem_controller
                        ->backend_mem_access_queue.cur_idx
                        = 0;
                    core->memory1.stage_exec_done = FALSE;
                    e->max_clock_cycles = 0;
                    e->elasped_clock_cycles = 0;
                    e->data_fwd_done = FALSE;
                    core->memory2 = core->memory1;
                    cpu_stage_flush(&core->memory1);
                }
            }
            else
            {
                if (!core->commit.has_data
                    && push_insn_from_memory1_to_commit(core, e))
                {
                    s->simcpu->mem_hierarchy->mem_controller
                        ->backend_mem_access_queue.cur_idx
                        = 0;
                    core->memory1.stage_exec_done = FALSE;
                    e->max_clock_cycles = 0;
                    e->elasped_clock_cycles = 0;
                    e->data_fwd_done = FALSE;
                    core->commit = core->memory1;
                    cpu_stage_flush(&core->memory1);
                }
            }
        }
        else
        {
            e->elasped_clock_cycles++;
        }
    }
}

/**
 * Memory 2 stage: dummy stage for load and atomics, just pass through to
 * consume 1 cycle
 */
void
in_core_memory2(INCore *core)
{
    InstructionLatch *e;
    RISCVCPUState *s;

    s = core->simcpu->emu_cpu_state;
    if (core->memory2.has_data)
    {
        e = get_insn_latch(s->simcpu->insn_latch_pool,
                           core->memory2.insn_latch_index);
        if (!core->memory2.stage_exec_done)
        {
            e->elasped_clock_cycles = 1;
            e->max_clock_cycles = 1;
            core->memory2.stage_exec_done = TRUE;
        }

        if (e->elasped_clock_cycles == e->max_clock_cycles)
        {
            /* Push the data read by loads/atomics on forwarding bus*/
            if (!e->ins.exception && !e->data_fwd_done && !e->keep_dest_busy
                && (e->ins.is_load || e->ins.is_atomic)
                && ((e->ins.has_dest && e->ins.rd != 0) || e->ins.has_fp_dest))
            {
                core->fwd_latch[NUM_FWD_BUS - 1].rd = e->ins.rd;
                core->fwd_latch[NUM_FWD_BUS - 1].buffer = e->ins.buffer;
                core->fwd_latch[NUM_FWD_BUS - 1].int_dest = e->ins.has_dest;
                core->fwd_latch[NUM_FWD_BUS - 1].fp_dest = e->ins.has_fp_dest;
                core->fwd_latch[NUM_FWD_BUS - 1].valid = TRUE;
                e->data_fwd_done = TRUE;
            }

            if (!core->commit.has_data)
            {
                s->simcpu->mem_hierarchy->mem_controller
                    ->backend_mem_access_queue.cur_idx
                    = 0;
                core->memory2.stage_exec_done = FALSE;
                e->max_clock_cycles = 0;
                e->elasped_clock_cycles = 0;
                e->data_fwd_done = FALSE;
                core->commit = core->memory2;
                cpu_stage_flush(&core->memory2);
            }
        }
        else
        {
            e->elasped_clock_cycles++;
        }
    }
}