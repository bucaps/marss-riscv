/**
 * Out of order core back-end stages : issue, execute, rob-commit
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
#include <stdio.h>

#include "../riscv_cpu_priv.h"
#include "circular_queue.h"
#include "ooo.h"
#include "riscv_ins_execute_lib.h"
#include "riscv_sim_cpu.h"

/*===============================================
=            Instruction Issue Stage            =
===============================================*/

static int
issue_ins_to_exec_unit(OOCore *core, IMapEntry *e)
{
    CPUStage *fu;

    switch (e->ins.fu_type)
    {
        case FU_ALU:
        {
            fu = &core->ialu[0];
            break;
        }
        case FU_MUL:
        {
            fu = &core->imul[0];
            break;
        }
        case FU_DIV:
        {
            fu = &core->idiv[0];
            break;
        }
        case FU_FPU_ALU:
        {
            fu = &core->fpu_alu;
            break;
        }
        case FU_FPU_FMA:
        {
            fu = &core->fpu_fma[0];
            break;
        }
        default:
        {
            assert(0);
        }
    }

    if (!fu->has_data)
    {
        fu->has_data = TRUE;
        fu->stage_exec_done = FALSE;
        fu->imap_index = e->imap_index;
        return 0;
    }

    /* FU busy */
    return -1;
}

static void
read_int_operand(OOCore *core, int has_src, int *read_src, int arch_src,
                 int phy_src, int current_rob_idx, uint64_t *buffer,
                 IMapEntry *e)
{
    if (has_src && !(*read_src))
    {
        read_int_operand_from_rob_slot(core, e, arch_src, phy_src,
                                       current_rob_idx, buffer, read_src);
    }
}

static void
read_fp_operand(OOCore *core, int has_src, int *read_src, int arch_src,
                int phy_src, int current_rob_idx, uint64_t *buffer,
                IMapEntry *e)
{
    if (has_src && !(*read_src))
    {
        read_fp_operand_from_rob_slot(core, e, arch_src, phy_src,
                                      current_rob_idx, buffer, read_src);
    }
}

static int
issue_instruction(OOCore *core, IssueQueueEntry *iqe, IMapEntry *e)
{
    if (!issue_ins_to_exec_unit(core, e))
    {
        /* Instruction issued, deallocate IQ entry */
        iqe->valid = FALSE;
        iqe->ready = FALSE;
        iqe->e = NULL;

        return TRUE;
    }

    /* Issue failed */
    return FALSE;
}

static void
process_iq(OOCore *core, IssueQueueEntry *iq, int iq_size, int max_issue_ports)
{
    int i;
    IMapEntry *e;
    IssueQueueEntry *iqe;
    int current_issue_count = 0;

    for (i = 0; (i < iq_size) && (current_issue_count < max_issue_ports); ++i)
    {
        iqe = &iq[i];
        e = iqe->e;

        if (iqe->valid == TRUE)
        {
            /* Entry is ready to issue */
            if (iqe->ready)
            {
                if (issue_instruction(core, iqe, e))
                {
                    current_issue_count++;
                }
                continue;
            }
            else
            {
                /* IQ entry not ready, try to read sources */
                read_int_operand(core, e->ins.has_src1, &e->read_rs1,
                                 e->ins.rs1, e->ins.prs1, e->rob_idx,
                                 &e->ins.rs1_val, e);
                read_int_operand(core, e->ins.has_src2, &e->read_rs2,
                                 e->ins.rs2, e->ins.prs2, e->rob_idx,
                                 &e->ins.rs2_val, e);
                read_fp_operand(core, e->ins.has_fp_src1, &e->read_rs1,
                                e->ins.rs1, e->ins.prs1, e->rob_idx,
                                &e->ins.rs1_val, e);
                read_fp_operand(core, e->ins.has_fp_src2, &e->read_rs2,
                                e->ins.rs2, e->ins.prs2, e->rob_idx,
                                &e->ins.rs2_val, e);
                read_fp_operand(core, e->ins.has_fp_src3, &e->read_rs3,
                                e->ins.rs3, e->ins.prs3, e->rob_idx,
                                &e->ins.rs3_val, e);

                if (e->read_rs1 && e->read_rs2 && e->read_rs3)
                {
                    iqe->ready = TRUE;
                }
            }

            /* Try to issue instruction again after reading the sources */
            if (iqe->ready)
            {
                if (issue_instruction(core, iqe, e))
                {
                    current_issue_count++;
                }
                continue;
            }
        }
    }
}

void
oo_core_issue(OOCore *core)
{
    process_iq(core, core->iq, core->simcpu->params->iq_size,
               core->simcpu->params->iq_issue_ports);
}

/*=====  End of Instruction Issue Stage  ======*/

/*===================================================
=            Instruction Execution Stage            =
===================================================*/
static CPUStage *
get_next_exec_stage(OOCore *core, int cur_stage_id, int fu_type)
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
oo_core_execute_non_pipe(OOCore *core, int fu_type, CPUStage *stage)
{
    IMapEntry *e;
    RISCVCPUState *s = core->simcpu->emu_cpu_state;

    if (stage->has_data)
    {
        e = get_imap_entry(s->simcpu->imap, stage->imap_index);
        if (!stage->stage_exec_done)
        {
            execute_riscv_instruction(&e->ins, &s->fflags);

            /* Update FU stats */
            ++s->simcpu->stats[s->priv].fu_access[fu_type];

            /* current_latency: number of CPU cycles spent by this instruction
             * in execute stage so far */
            e->current_latency = 1;
            e->max_latency = set_max_latency_for_non_pipe_fu(s, fu_type, e);
            assert(e->max_latency);
            stage->stage_exec_done = TRUE;
        }

        /* If the latency is completed and next stage is free, pass this
         * instruction to the next stage, else stall */
        if (e->current_latency == e->max_latency)
        {
            if (e->ins.is_load || e->ins.is_store || e->ins.is_atomic)
            {
                /* Inform the LSQ entry that address is calculated */
                core->lsq.entries[e->lsq_idx].ready = TRUE;
            }
            else
            {
                if (e->ins.is_branch)
                {
                    if (!e->branch_processed)
                    {
                        oo_process_branch(core, e);
                    }
                }

                if (e->ins.has_dest || e->ins.has_fp_dest)
                {
                    core->rob.entries[e->rob_idx].ready = TRUE;
                }
            }

            /* Execution complete */
            e->max_latency = 0;
            e->current_latency = 0;
            cpu_stage_flush(stage);
        }
        else
        {
            e->current_latency++;
        }
    }
}

static void
oo_core_execute_pipe(OOCore *core, int cur_stage_id, int fu_type, CPUStage *stage,
                int max_latency, int max_stage_id)
{
    IMapEntry *e;
    CPUStage *next;
    RISCVCPUState *s = core->simcpu->emu_cpu_state;

    if (stage->has_data)
    {
        e = get_imap_entry(s->simcpu->imap, stage->imap_index);
        if (!stage->stage_exec_done)
        {
            execute_riscv_instruction(&e->ins, &s->fflags);

            /* Update FU stats */
            ++s->simcpu->stats[s->priv].fu_access[fu_type];

            /* current_latency: number of CPU cycles spent by this instruction
             * in execute stage so far */
            e->current_latency = 1;
            stage->stage_exec_done = TRUE;
        }

        /* If the latency is completed and next stage is free, pass this
         * instruction to the next stage, else stall */
        if (e->current_latency == max_latency)
        {
            /* Instruction is in last stage of FU*/
            if (cur_stage_id == max_stage_id)
            {
                if (e->ins.is_load || e->ins.is_store || e->ins.is_atomic)
                {
                    /* Inform the LSQ entry that address is calculated */
                    core->lsq.entries[e->lsq_idx].ready = TRUE;
                }
                else
                {
                    if (e->ins.is_branch)
                    {
                        if (!e->branch_processed)
                        {
                            oo_process_branch(core, e);
                        }
                    }

                    if (e->ins.has_dest || e->ins.has_fp_dest)
                    {
                        core->rob.entries[e->rob_idx].ready = TRUE;
                    }
                }

                /* Execution complete */
                e->max_latency = 0;
                e->current_latency = 0;
                cpu_stage_flush(stage);
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
oo_core_execute_all(OOCore *core)
{
    int i;

    for (i = core->simcpu->params->num_fpu_fma_stages - 1; i >= 0; i--)
    {
        oo_core_execute_pipe(core, i, FU_FPU_FMA, &core->fpu_fma[i],
                        core->simcpu->params->fpu_fma_stage_latency[i],
                        core->simcpu->params->num_fpu_fma_stages - 1);
    }
    oo_core_execute_non_pipe(core, FU_FPU_ALU, &core->fpu_alu);
    for (i = core->simcpu->params->num_div_stages - 1; i >= 0; i--)
    {
        oo_core_execute_pipe(core, i, FU_DIV, &core->idiv[i],
                        core->simcpu->params->div_stage_latency[i],
                        core->simcpu->params->num_div_stages - 1);
    }
    for (i = core->simcpu->params->num_mul_stages - 1; i >= 0; i--)
    {
        oo_core_execute_pipe(core, i, FU_MUL, &core->imul[i],
                        core->simcpu->params->mul_stage_latency[i],
                        core->simcpu->params->num_mul_stages - 1);
    }
    for (i = core->simcpu->params->num_alu_stages - 1; i >= 0; i--)
    {
        oo_core_execute_pipe(core, i, FU_ALU, &core->ialu[i],
                        core->simcpu->params->alu_stage_latency[i],
                        core->simcpu->params->num_alu_stages - 1);
    }
}

/*=====  End of Instruction Execution Stage  ======*/

/*========================================
=            ROB Commit Stage            =
========================================*/

static int
rob_can_commit(ROB *rob)
{
    if (!cq_empty(&rob->cq) && (rob->entries[cq_front(&rob->cq)].ready))
    {
        return TRUE;
    }

    return FALSE;
}

int
oo_core_rob_commit(OOCore *core)
{
    IMapEntry *e;
    ROBEntry *rbe;
    RISCVCPUState *s;
    int commits = 0;

    s = core->simcpu->emu_cpu_state;
    while (rob_can_commit(&core->rob))
    {
        rbe = &core->rob.entries[cq_front(&core->rob.cq)];
        e = rbe->e;
        assert(rbe->ready);
        if (e->ins.exception)
        {
            set_exception_state(s, e);
            return -1;
        }
        else
        {
            if (e->ins.has_dest)
            {
                if (e->ins.rd)
                {
                    update_arch_reg_int(s, e);

                    if (core->int_rat[e->ins.rd].rob_idx == e->rob_idx)
                    {
                        core->int_rat[e->ins.rd].read_from_rob = FALSE;
                        core->int_rat[e->ins.rd].rob_idx = -1;
                    }
                }
            }
            else if (e->ins.has_fp_dest)
            {
                update_arch_reg_fp(s, e);

                if (core->fp_rat[e->ins.rd].rob_idx == e->rob_idx)
                {
                    core->fp_rat[e->ins.rd].read_from_rob = FALSE;
                    core->fp_rat[e->ins.rd].rob_idx = -1;
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

            /* Free up imap entry */
            e->status = IMAP_ENTRY_STATUS_FREE;

            /* All the dependent instructions will have to lookup ARF for
             * operand value */
            rbe->ready = FALSE;

            /* Deallocate ROB entry */
            cq_dequeue(&core->rob.cq);

            /* Check for timeout */
            if ((--s->sim_n_cycles) == 0)
            {
                set_timer_exception_state(s, e);
                return -1;
            }

            commits++;
            if (commits == core->simcpu->params->rob_commit_ports)
            {
                break;
            }
        }
    }

    return 0;
}
/*=====  End of ROB Commit Stage  ======*/
