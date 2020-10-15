/**
 * In-order Pipeline Back-end Stages: execute, memory and commit
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

/*=================================================
=            Instruction Execute Stage            =
=================================================*/

static CPUStage *
get_next_exec_pipe_stage(const INCore *core, int cur_stage_id, int fu_type)
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
exec_insn_and_invalidate_rd(RISCVCPUState *s, INCore *core, InstructionLatch *e,
                            int fu_type)
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

    e->elasped_clock_cycles = 1;
    e->ins.rm = get_insn_rm(s, (e->ins.binary >> 12) & 7);
    execute_riscv_instruction(&e->ins, &s->fflags);
    ++s->simcpu->stats[s->priv].fu_access[fu_type];
}

static void
fwd_data_from_ex_to_decode(INCore *core, InstructionLatch *e, int fu_type)
{
    /* Bypass all integer instructions except div */
    /* No bypass for memory and floating point instructions */
    if (!e->data_fwd_done
        && !(e->ins.is_load || e->ins.is_store || e->ins.is_atomic)
        && !e->keep_dest_busy && ((e->ins.has_dest && (e->ins.rd != 0)))
        && !(e->ins.type == INS_TYPE_INT_DIV))
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
push_insn_from_ex_to_mem(INCore *core, InstructionLatch *e, CPUStage *stage)
{
    if (core->ex_to_mem_queue.data[cq_front(&core->ex_to_mem_queue.cq)]
        == e->ins_dispatch_id)
    {
        if (e->ins.is_load || e->ins.is_atomic)
        {
            if (!core->memory1.has_data)
            {
                cq_dequeue(&core->ex_to_mem_queue.cq);
                e->elasped_clock_cycles = 0;
                e->data_fwd_done = FALSE;
                stage->stage_exec_done = FALSE;
                core->memory1 = *stage;
                cpu_stage_flush(stage);
            }
        }
        else
        {

            if (!core->memory1.has_data && !core->memory2.has_data)
            {
                cq_dequeue(&core->ex_to_mem_queue.cq);
                e->elasped_clock_cycles = 0;
                e->data_fwd_done = FALSE;
                stage->stage_exec_done = FALSE;
                core->memory1 = *stage;
                cpu_stage_flush(stage);
            }
        }
    }
}

static void
in_core_execute_non_pipe(INCore *core, int fu_type, CPUStage *stage)
{
    RISCVCPUState *s;
    InstructionLatch *e;

    s = core->simcpu->emu_cpu_state;
    if (stage->has_data)
    {
        e = get_insn_latch(s->simcpu->insn_latch_pool, stage->insn_latch_index);
        ++s->simcpu->stats[s->priv].exec_unit_delay;
        if (!stage->stage_exec_done)
        {
            exec_insn_and_invalidate_rd(s, core, e, fu_type);
            e->max_clock_cycles
                = set_max_clock_cycles_for_non_pipe_fu(s, fu_type, e);

            sim_assert((e->max_clock_cycles),
                       "error: %s at line %d in %s(): %s", __FILE__, __LINE__,
                       __func__,
                       "max_clock_cycles execution latency for an instruction "
                       "must be non_zero");

            stage->stage_exec_done = TRUE;
        }

        if (e->elasped_clock_cycles == e->max_clock_cycles)
        {
            fwd_data_from_ex_to_decode(core, e, fu_type);
            push_insn_from_ex_to_mem(core, e, stage);
        }
        else
        {
            e->elasped_clock_cycles++;
        }
    }
}

static void
in_core_execute_pipe(INCore *core, int cur_stage_id, int fu_type,
                     CPUStage *stage, int max_clock_cycles, int max_stage_id)
{
    CPUStage *next;
    RISCVCPUState *s;
    InstructionLatch *e;

    s = core->simcpu->emu_cpu_state;
    if (stage->has_data)
    {
        e = get_insn_latch(s->simcpu->insn_latch_pool, stage->insn_latch_index);
        ++s->simcpu->stats[s->priv].exec_unit_delay;
        if (!stage->stage_exec_done)
        {
            exec_insn_and_invalidate_rd(s, core, e, fu_type);
            stage->stage_exec_done = TRUE;
        }

        if (e->elasped_clock_cycles == max_clock_cycles)
        {
            /* Instruction is in the last stage of FU */
            if (cur_stage_id == max_stage_id)
            {
                fwd_data_from_ex_to_decode(core, e, fu_type);
                push_insn_from_ex_to_mem(core, e, stage);
            }
            else
            {
                /* Pass the instruction into next stage for this FU */
                next = get_next_exec_pipe_stage(core, cur_stage_id, fu_type);
                if (!next->has_data)
                {
                    e->elasped_clock_cycles = 1;
                    *next = *stage;
                    cpu_stage_flush(stage);
                }
            }
        }
        else
        {
            e->elasped_clock_cycles++;
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

/*================================================
=            Instruction Commit Stage            =
================================================*/

int
in_core_commit(INCore *core)
{
    InstructionLatch *e;
    RISCVCPUState *s;

    s = core->simcpu->emu_cpu_state;
    if (core->commit.has_data)
    {
        e = get_insn_latch(s->simcpu->insn_latch_pool,
                           core->commit.insn_latch_index);

        if (e->ins.has_dest)
        {
            /* Update the integer register if rd is not x0 */
            if (e->ins.rd)
            {
                update_arch_reg_int(s, e);

                /* If keep_dest_busy is TRUE, there is following instruction
                 * writing to the same rd (WAW). So let that instruction make
                 * the destination valid when it commits */
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

        if (s->simcpu->params->do_sim_trace)
        {
            sim_trace_commit(s->simcpu->trace, s->simcpu->clock, s->priv, e);
        }

        if (s->sim_params->enable_stats_display)
        {
            write_stats_to_stats_display_shm(s->simcpu);
        }

        /* Commit success */
        e->status = INSN_LATCH_FREE;
        cpu_stage_flush(&core->commit);

        /* Check for user specified sim_emulate_after_icount instructions */
        if (s->sim_params->sim_emulate_after_icount
            && (s->simcpu->icount >= s->sim_params->sim_emulate_after_icount))
        {
            e->ins.exception_cause = SIM_ICOUNT_COMPLETE_EXCEPTION;
            sim_exception_set(s->simcpu->exception, e);
            return -1;
        }

        /* Check for timeout */
        if ((--s->n_cycles) == 0)
        {
            e->ins.exception_cause = SIM_TEMU_TIMEOUT_EXCEPTION;
            sim_exception_set(s->simcpu->exception, e);
            return -1;
        }
    }
    return 0;
}
/*=====  End of Instruction Commit Stage  ======*/