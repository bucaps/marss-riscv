/**
 * Out of order core back-end stages : issue, execute, write-back, rob-commit
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
            fu = &core->fpu_alu[0];
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
read_int_operand(OOCore *core, int has_src, int *read_src, int src,
                 uint64_t *buffer, PRFEntry *prf)
{
    int i;
    PRFEntry *pre;

    if (has_src && !(*read_src))
    {
        pre = &prf[src];
        if (!pre->valid)
        {
            for (i = 0; i < NUM_FWD_BUS; ++i)
            {
                if (core->fwd_latch[i].valid && core->fwd_latch[i].int_dest
                    && (core->fwd_latch[i].rd == src))
                {
                    *buffer = (target_ulong)core->fwd_latch[i].buffer;
                    *read_src = TRUE;
                    break;
                }
            }
        }
        else
        {
            *buffer = (target_ulong)pre->val;
            *read_src = TRUE;
        }
    }
}

static void
read_fp_operand(OOCore *core, int has_src, int *read_src, int src,
                uint64_t *buffer, PRFEntry *prf)
{
    int i;
    PRFEntry *pre;

    if (has_src && !(*read_src))
    {
        pre = &prf[src];
        if (!pre->valid)
        {
            /* Floating point execution units start from ID 3 onwards */
            for (i = 3; i < NUM_FWD_BUS; ++i)
            {
                if (core->fwd_latch[i].valid && core->fwd_latch[i].fp_dest
                    && (core->fwd_latch[i].rd == src))
                {
                    *buffer = core->fwd_latch[i].buffer;
                    *read_src = TRUE;
                    break;
                }
            }
        }
        else
        {
            *buffer = pre->val;
            *read_src = TRUE;
        }
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
                                 e->ins.prs1, &e->ins.rs1_val, core->prf_int);
                read_int_operand(core, e->ins.has_src2, &e->read_rs2,
                                 e->ins.prs2, &e->ins.rs2_val, core->prf_int);
                read_fp_operand(core, e->ins.has_fp_src1, &e->read_rs1,
                                e->ins.prs1, &e->ins.rs1_val, core->prf_fp);
                read_fp_operand(core, e->ins.has_fp_src2, &e->read_rs2,
                                e->ins.prs2, &e->ins.rs2_val, core->prf_fp);
                read_fp_operand(core, e->ins.has_fp_src3, &e->read_rs3,
                                e->ins.prs3, &e->ins.rs3_val, core->prf_fp);

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
    process_iq(core, core->iq_int, core->simcpu->params->iq_int_size,
               core->simcpu->params->iq_int_issue_ports);
    process_iq(core, core->iq_fp, core->simcpu->params->iq_fp_size,
               core->simcpu->params->iq_fp_issue_ports);
    process_iq(core, core->iq_mem, core->simcpu->params->iq_mem_size,
               core->simcpu->params->iq_mem_issue_ports);
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
        case FU_FPU_ALU:
        {
            stage = &core->fpu_alu[cur_stage_id + 1];
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

/**

    TODO:
    - Optimize

 */
void
oo_core_execute(OOCore *core, int cur_stage_id, int fu_type, CPUStage *stage,
                int max_latency, int max_stage_id)
{
    IMapEntry *e;
    CPUStage *next;
    RISCVSIMCPUState *simcpu = core->simcpu;

    if (stage->has_data)
    {
        e = &simcpu->imap[stage->imap_index];
        if (!stage->stage_exec_done)
        {
            execute_riscv_instruction(&e->ins,
                                      &core->simcpu->emu_cpu_state->fflags);

            /* Update FU stats */
            ++core->simcpu->stats[core->simcpu->emu_cpu_state->priv]
                  .fu_access[fu_type];

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
                    /* Push out the result and the register address on the
                     * forwarding bus for this FU for all the operate
                     * instructions */
                    if (!e->data_fwd_done
                        && ((e->ins.has_dest && e->ins.rd != 0)
                            || e->ins.has_fp_dest))
                    {
                        core->fwd_latch[fu_type].rd = e->ins.pdest;
                        core->fwd_latch[fu_type].buffer = e->ins.buffer;
                        core->fwd_latch[fu_type].int_dest = e->ins.has_dest;
                        core->fwd_latch[fu_type].fp_dest = e->ins.has_fp_dest;
                        core->fwd_latch[fu_type].valid = TRUE;
                        e->data_fwd_done = TRUE;
                    }

                    if (e->ins.is_branch)
                    {
                        if (!e->branch_processed)
                        {
                            oo_process_branch(core, e);
                        }
                    }

                    /* Write request to INT prf */
                    if (e->ins.has_dest)
                    {
                        if (send_phy_reg_write_request(&core->prf_int_wb_queue,
                                                       e))
                        {
                            /* All write ports occupied */
                            return;
                        }
                    }
                    else if (e->ins.has_fp_dest)
                    {
                        /* Write request to FP prf */
                        if (send_phy_reg_write_request(&core->prf_fp_wb_queue,
                                                       e))
                        {
                            /* All write ports occupied */
                            return;
                        }
                    }
                }

                /* Execution complete */
                e->max_latency = 0;
                e->current_latency = 0;
                e->data_fwd_done = FALSE;
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
        oo_core_execute(core, i, FU_FPU_FMA, &core->fpu_fma[i],
                        core->simcpu->params->fpu_fma_stage_latency[i],
                        core->simcpu->params->num_fpu_fma_stages - 1);
    }
    for (i = core->simcpu->params->num_fpu_alu_stages - 1; i >= 0; i--)
    {
        oo_core_execute(core, i, FU_FPU_ALU, &core->fpu_alu[i],
                        core->simcpu->params->fpu_alu_stage_latency[i],
                        core->simcpu->params->num_fpu_alu_stages - 1);
    }
    for (i = core->simcpu->params->num_div_stages - 1; i >= 0; i--)
    {
        oo_core_execute(core, i, FU_DIV, &core->idiv[i],
                        core->simcpu->params->div_stage_latency[i],
                        core->simcpu->params->num_div_stages - 1);
    }
    for (i = core->simcpu->params->num_mul_stages - 1; i >= 0; i--)
    {
        oo_core_execute(core, i, FU_MUL, &core->imul[i],
                        core->simcpu->params->mul_stage_latency[i],
                        core->simcpu->params->num_mul_stages - 1);
    }
    for (i = core->simcpu->params->num_alu_stages - 1; i >= 0; i--)
    {
        oo_core_execute(core, i, FU_ALU, &core->ialu[i],
                        core->simcpu->params->alu_stage_latency[i],
                        core->simcpu->params->num_alu_stages - 1);
    }
}

/*=====  End of Instruction Execution Stage  ======*/

/*==========================================================
=            Physical Register Write-back Stage            =
==========================================================*/

void
oo_core_writeback(OOCore *core)
{
    IMapEntry *e;
    PRFEntry *pre;
    int wb_queue_idx;

    /* Process all the write requests to INT PRF */
    while (!cq_empty(&core->prf_int_wb_queue.cq))
    {
        wb_queue_idx = cq_front(&core->prf_int_wb_queue.cq);

        if (core->prf_int_wb_queue.entries[wb_queue_idx].valid)
        {
            e = core->prf_int_wb_queue.entries[wb_queue_idx].e;

            /* Ignore writes to P0, since its always zero */
            if (e->ins.pdest)
            {
                pre = &core->prf_int[e->ins.pdest];
                pre->valid = TRUE;
                pre->val = (target_ulong)e->ins.buffer;
            }

            /* Inform the ROB that the instruction is ready to commit */
            core->rob.entries[e->rob_idx].ready = TRUE;
        }

        cq_dequeue(&core->prf_int_wb_queue.cq);
    }

    /* Process all the write requests to FP PRF */
    while (!cq_empty(&core->prf_fp_wb_queue.cq))
    {
        wb_queue_idx = cq_front(&core->prf_fp_wb_queue.cq);

        if (core->prf_fp_wb_queue.entries[wb_queue_idx].valid)
        {
            e = core->prf_fp_wb_queue.entries[wb_queue_idx].e;

            pre = &core->prf_fp[e->ins.pdest];
            pre->valid = TRUE;

            /* Add mask to result if needed */
            if (e->ins.f32_mask)
            {
                e->ins.buffer |= F32_HIGH;
            }
            else if (e->ins.f64_mask)
            {
                e->ins.buffer |= F64_HIGH;
            }

            if (e->ins.set_fs)
            {
                core->simcpu->emu_cpu_state->fs = 3;
            }

            pre->val = e->ins.buffer;

            /* Inform the ROB that the instruction is ready to commit */
            core->rob.entries[e->rob_idx].ready = TRUE;
        }

        cq_dequeue(&core->prf_fp_wb_queue.cq);
    }
}

/*=====  End of Physical Register Write-back Stage  ======*/

/*========================================
=            ROB Commit Stage            =
========================================*/

static void
phy_reg_deallocate(CQInt *free_list, int preg, PRFEntry *prf)
{
    int idx;

    idx = cq_enqueue(&free_list->cq);
    free_list->entries[idx] = preg;
    prf[preg].valid = FALSE;
}

int
oo_core_rob_commit(OOCore *core)
{
    IMapEntry *e;
    ROBEntry *rbe;
    RISCVCPUState *s;
    int current_commit_count = 0;
    int commmit_success = TRUE;

    while (commmit_success
           && (current_commit_count < core->simcpu->params->rob_commit_ports))
    {
        if (!cq_empty(&core->rob.cq))
        {
            rbe = &core->rob.entries[cq_front(&core->rob.cq)];
            s = core->simcpu->emu_cpu_state;
            e = rbe->e;
            if (rbe->ready == TRUE)
            {
                if (e->ins.exception)
                {
                    set_exception_state(s, e);
                    return -1;
                }
                else
                {
                    if (e->ins.has_dest)
                    {
                        /* Update commit RAT mapping */
                        core->commit_rat_int[e->ins.rd] = e->ins.pdest;

                        /* Update arch register, NOTE: Performance hack */
                        if (e->ins.rd)
                        {
                            s->reg[e->ins.rd] = core->prf_int[e->ins.pdest].val;
                        }

                        /* Free up the previous physical destination */
                        if (e->ins.old_pdest)
                        {
                            phy_reg_deallocate(&core->free_pr_int,
                                               e->ins.old_pdest, core->prf_int);
                        }
                    }
                    else if (e->ins.has_fp_dest)
                    {
                        /* Update commit RAT mapping */
                        core->commit_rat_fp[e->ins.rd] = e->ins.pdest;

                        /* Free up the previous physical destination */
                        phy_reg_deallocate(&core->free_pr_fp, e->ins.old_pdest,
                                           core->prf_fp);

                        /* Update FP arch register, NOTE: Performance hack */
                        s->fp_reg[e->ins.rd] = core->prf_fp[e->ins.pdest].val;
                    }

                    if (e->ins.is_branch && e->mispredict)
                    {
                        riscv_sim_cpu_reset(core->simcpu);
                        s->code_ptr = NULL;
                        s->code_end = NULL;
                        s->code_to_pc_addend = (target_ulong)e->branch_target;
                        core->fetch.has_data = TRUE;
                    }

                    ++core->simcpu->stats[s->priv].ins_simulated;
                    ++core->simcpu->stats[s->priv].ins_type[e->ins.type];

                    if ((e->ins.type == INS_TYPE_COND_BRANCH)
                        && e->is_branch_taken)
                    {
                        ++core->simcpu->stats[s->priv].ins_cond_branch_taken;
                    }

                    /* Free up imap entry */
                    e->status = IMAP_ENTRY_STATUS_FREE;

                    /* Deallocate ROB entry */
                    cq_dequeue(&core->rob.cq);

                    current_commit_count++;

#if defined(CONFIG_SIM_TRACE)
                    print_ins_trace(s, core->simcpu->clock, e->ins.pc,
                                    e->ins.binary, e->ins.str,
                                    (e->ins.has_dest | e->ins.has_fp_dest),
                                    e->ins.has_dest, e->ins.rd, core->prf_int[e->ins.pdest].val,
                                    e->ins.mem_addr, s->priv, "sim-ooo");
#endif

                    if (s->sim_params->enable_stats_display)
                    {
                        if ((s->simcpu->clock
                             % REALTIME_STATS_CLOCK_CYCLES_INTERVAL)
                            == 0)
                        {
                            /* Since cache stats are stored separately inside
                             * the Cache
                             * structure, they have to be copied to SimStats,
                             * before writing
                             * stats to shared memory. */
                            copy_cache_stats_to_global_stats(s);
                            memcpy(s->stats_shm_ptr, s->simcpu->stats,
                                   NUM_MAX_PRV_LEVELS * sizeof(SimStats));
                        }
                    }

                    /* Check for timeout */
                    if ((--s->n_cycles) == 1)
                    {
                        set_timer_exception_state(s, e);
                        return -1;
                    }
                }
            }
            else
            {
                /* ROB top not yet ready to commit, so stop commits */
                commmit_success = FALSE;
            }
        }
        else
        {
            /* ROB is empty */
            commmit_success = FALSE;
        }
    }
    return 0;
}
/*=====  End of ROB Commit Stage  ======*/
