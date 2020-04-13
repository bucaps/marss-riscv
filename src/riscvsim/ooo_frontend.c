/**
 * Out of order core front-end stages: fetch, decode, dispatch
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
#include "riscv_isa_decoder_lib.h"
#include "riscv_sim_cpu.h"

/*===============================================
=            Instruction Fetch Stage            =
===============================================*/

void
oo_core_fetch(OOCore *core)
{
    IMapEntry *e;
    RISCVCPUState *s;
    RISCVSIMCPUState *simcpu;

    if (core->fetch.has_data)
    {
        s = core->simcpu->emu_cpu_state;
        simcpu = core->simcpu;
        if (!core->fetch.stage_exec_done)
        {
            /* Calculate current PC*/
            simcpu->pc
                = (target_ulong)((uintptr_t)s->code_ptr + s->code_to_pc_addend);

            e = allocate_imap_entry(simcpu->imap);

            /* Setup the allocated imap entry */
            e->ins.pc = simcpu->pc;
            e->ins.create_str = s->sim_params->create_ins_str;

            /* Store IMAP index in the stage and the actual decoded instruction
             * info is stored in this IMAP entry. This avoids copying of whole
             * decoded instruction info when instruction flows to next stage */
            core->fetch.imap_index = e->imap_index;

            /* Set default minimum page walk latency. If the page walk does
             * occur, hw_pg_tb_wlk_latency will be higher than this default
             * value because it will also include the cache lookup latency for
             * reading/writing page table entries. Page table entries are looked
             * up in L1 data cache. */
            s->hw_pg_tb_wlk_latency = 1;
            s->hw_pg_tb_wlk_stage_id = FETCH;
            s->hw_pg_tb_wlk_latency_accounted = 0;
            s->ins_page_walks_accounted = 0;
            s->ins_tlb_lookup_accounted = 0;
            s->ins_tlb_hit_accounted = 0;

            /* current_latency: number of CPU cycles spent by this instruction
             * in fetch stage so far */
            e->current_latency = 1;

            /* Fetch instruction from RISCVEMU memory map */
            if (code_tlb_access_and_ins_fetch(s, e))
            {
                /* This instruction has raised a page fault exception during
                 * fetch */
                e->ins.exception = TRUE;
                e->ins.exception_cause = SIM_EXCEPTION;

                /* Hardware page table walk has been done and its latency must
                 * be simulated */
                e->max_latency = s->hw_pg_tb_wlk_latency;
            }
            else
            {
                /* max_latency: Number of CPU cycles required for TLB and Cache
                 * look-up */
                e->max_latency
                    = s->hw_pg_tb_wlk_latency
                      + mmu_insn_read(simcpu->mmu, s->code_guest_paddr, 4,
                                      FETCH, s->priv);

                /* Keep track of physical address of this instruction for later use */
                e->ins.phy_pc = s->code_guest_paddr;

                /* If true, it indicates that some sort of memory access request
                 * are sent to memory controller for this instruction, so
                 * request the fast wrap-around read for this address */
                if (simcpu->mmu->mem_controller->frontend_mem_access_queue
                        .cur_size)
                {
                    mem_controller_req_fast_read_for_addr(
                        &simcpu->mmu->mem_controller->frontend_mem_access_queue,
                        e->ins.phy_pc);
                }

                if (s->sim_params->enable_l1_caches)
                {
                    /* L1 caches and TLB are probed in parallel */
                    e->max_latency
                        -= min_int(s->hw_pg_tb_wlk_latency,
                                   simcpu->mmu->icache->read_latency);
                }

                /* Increment PC for the next instruction */
                if (3 == (e->ins.binary & 3))
                {
                    s->code_ptr = s->code_ptr + 4;
                    s->code_guest_paddr = s->code_guest_paddr + 4;
                }
                else
                {
                    /* For compressed */
                    s->code_ptr = s->code_ptr + 2;
                    s->code_guest_paddr = s->code_guest_paddr + 2;
                }
                ++simcpu->stats[s->priv].ins_fetch;
            }

            /* Probe the branch predictor front-end */
            core->simcpu->pfn_branch_frontend_probe_handler(
                core->simcpu->emu_cpu_state, e);

            core->fetch.stage_exec_done = TRUE;
        }
        else
        {
            e = &simcpu->imap[core->fetch.imap_index];
        }

        if (e->current_latency == e->max_latency)
        {
            /* Number of CPU cycles spent by this instruction in fetch stage
             * equals lookup delay for this instruction */

            /* Check if all the dram accesses, if required for this instruction,
             * are complete */
            if (!simcpu->mmu->mem_controller->frontend_mem_access_queue
                     .cur_size)
            {
                /* Memory controller read logic will install the tag in the cache line with
                 * the first word read, while the remaining words are still
                 * being fetched. This may cause a false hit on the following
                 * words. Check the memory controller to see if the word is
                 * received. Only then, proceed further. */
                if (!e->ins.exception
                    && mem_controller_wrap_around_read_pending(
                           simcpu->mmu->mem_controller, e->ins.phy_pc))
                {
                    return;
                }

                /* If the next stage is available, send this instruction to next
                   stage, else stall fetch */
                if (!core->decode.has_data)
                {
                    simcpu->mmu->mem_controller->frontend_mem_access_queue
                        .cur_idx
                        = 0;

                    core->fetch.stage_exec_done = FALSE;
                    e->max_latency = 0;
                    e->current_latency = 0;
                    core->decode = core->fetch;
                    core->fetch.imap_index = -1;

                    /* Stop fetching new instructions on a MMU exception */
                    if (e->ins.exception)
                    {
                        cpu_stage_flush(&core->fetch);
                    }
                }
            }
            else
            {
                ++simcpu->stats[s->priv].insn_mem_delay;
            }
        }
        else
        {
            e->current_latency++;
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
    IMapEntry *e;
    RISCVCPUState *s;
    RISCVSIMCPUState *simcpu;
    target_ulong ras_target = 0;

    if (core->decode.has_data)
    {
        s = core->simcpu->emu_cpu_state;
        simcpu = core->simcpu;
        e = &simcpu->imap[core->decode.imap_index];

        if (!core->decode.stage_exec_done)
        {
            if (!e->ins.exception && !e->is_decoded)
            {
                /* For decoding floating point instructions */
                e->ins.current_fs = s->fs;
                e->ins.rm = get_insn_rm(s, e->ins.rm);

                /* Decode the instruction */
                decode_riscv_binary(&e->ins, e->ins.binary);
                e->is_decoded = TRUE;

                /* If branch prediction is enabled and this instruction is a
                 * branch, add BPU entry for this branch if not present */
                if (s->sim_params->enable_bpu && e->ins.is_branch)
                {
                    if (!e->bpu_resp_pkt.bpu_probe_status)
                    {
                        bpu_add(core->simcpu->bpu, e->ins.pc,
                                e->ins.branch_type, &e->bpu_resp_pkt, s->priv,
                                e->ins.is_func_ret);
                    }

                    if (core->simcpu->params->ras_size)
                    {
                        if (e->ins.is_func_call)
                        {
                            ras_push(core->simcpu->bpu->ras, ((e->ins.binary & 3) == 3
                                                    ? e->ins.pc + 4
                                                    : e->ins.pc + 2));
                        }

                        if (e->ins.is_func_ret)
                        {
                            ras_target = ras_pop(core->simcpu->bpu->ras);

                            /* Start fetch from address returned by RAS if non-zero */
                            if (ras_target)
                            {
                                s->code_ptr = NULL;
                                s->code_end = NULL;
                                s->code_to_pc_addend = ras_target;
                                e->predicted_target = ras_target;

                                /* If memory access requests are submitted to dram
                                 * dispatch queue from fetch stage, remove them from
                                 * dram dispatch queue */
                                if (core->simcpu->mmu->mem_controller
                                        ->frontend_mem_access_queue.cur_size)
                                {
                                    mem_controller_flush_stage_queue_entry_from_dram_queue(
                                        &core->simcpu->mmu->mem_controller
                                             ->dram_dispatch_queue,
                                        &core->simcpu->mmu->mem_controller
                                             ->frontend_mem_access_queue);
                                }

                                mem_controller_flush_stage_mem_access_queue(
                                    &core->simcpu->mmu->mem_controller
                                         ->frontend_mem_access_queue);

                                cpu_stage_flush(&core->fetch);
                                core->fetch.has_data = TRUE;
                            }
                        }
                    }
                }
            }

            /* Unsupported opcode exception */
            if (e->ins.exception)
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
rob_entry_create(ROB *rob, IMapEntry *e, int rob_entry_status)
{
    int rob_idx;

    rob_idx = cq_enqueue(&rob->cq);
    e->rob_idx = rob_idx;
    rob->entries[rob_idx].ready = rob_entry_status;
    rob->entries[rob_idx].e = e;
}

static void
iq_entry_create(IssueQueueEntry *iq, int iq_size, IMapEntry *e)
{
    int iq_idx;

    iq_idx = iq_get_free_entry(iq, iq_size);
    e->iq_idx = iq_idx;
    iq[iq_idx].valid = TRUE;
    iq[iq_idx].ready = FALSE;
    iq[iq_idx].e = e;
}

static void
lsq_entry_create(LSQ *lsq, IMapEntry *e)
{
    int lsq_idx;

    lsq_idx = cq_enqueue(&lsq->cq);
    e->lsq_idx = lsq_idx;
    lsq->entries[lsq_idx].ready = FALSE;
    lsq->entries[lsq_idx].mem_request_sent = FALSE;
    lsq->entries[lsq_idx].mem_request_complete = FALSE;
    lsq->entries[lsq_idx].e = e;
}

/**
 *
 * Branch Index Stack:
 * This is for future implementation and currently is not fully implemented
 *
 */
#if 0
static void
bis_entry_create(BIS *bis, IMapEntry *e)
{
    int bis_idx;

    bis_idx = cq_enqueue(&bis->cq);
    assert(bis_idx >= 0);
    e->bis_idx = bis_idx;
    bis->entries[bis_idx].e = e;
}
#endif

static int
no_free_phy_reg(OOCore *core, IMapEntry *e)
{
    if (e->ins.has_dest && e->ins.rd)
    {
        return cq_empty(&core->free_pr_int.cq);
    }
    else if (e->ins.has_fp_dest)
    {
        return cq_empty(&core->free_pr_fp.cq);
    }
    else
    {
        return FALSE;
    }
}

static void
allocate_physical_register(CQInt *free_list, int *rename_table, PRFEntry *prf,
                           IMapEntry *e)
{
    int free_pdest_idx;

    /* Allocate free physical register */
    free_pdest_idx = cq_dequeue(&free_list->cq);

    e->ins.pdest = free_list->entries[free_pdest_idx];

    /* Save previous physical destination for this architectural
     * destination  This will be freed when this instruction
     * commits. */
    e->ins.old_pdest = rename_table[e->ins.rd];

    /* Update rename table mappings for destination */
    rename_table[e->ins.rd] = e->ins.pdest;

    /* Setup prf entry */
    prf[e->ins.pdest].valid = FALSE;
}

static void
allocate_destination(OOCore *core, IMapEntry *e)
{
    /* INT destination */
    if (e->ins.has_dest)
    {
        if (e->ins.rd)
        {
            allocate_physical_register(&core->free_pr_int, core->spec_rat_int,
                                       core->prf_int, e);
        }
        else
        {
            /* P0 is always zero */
            e->ins.pdest = 0;
        }
    }
    else if (e->ins.has_fp_dest)
    {
        allocate_physical_register(&core->free_pr_fp, core->spec_rat_fp,
                                   core->prf_fp, e);
    }
}

static int
dispatch_non_mem_instruction(OOCore *core, IssueQueueEntry *iq, int iq_size,
                             ROB *rob, IMapEntry *e, BIS *bis)
{
    if (cq_full(&rob->cq) || iq_full(iq, iq_size) || no_free_phy_reg(core, e))
    {
        return -1;
    }

    /* Tag this instruction with the BIS index of the earliest preceding branch
     */
    // e->bis_tag = cq_rear(&bis->cq);
    rob_entry_create(rob, e, FALSE);
    iq_entry_create(iq, iq_size, e);
    allocate_destination(core, e);
    return 0;
}

static int
dispatch_mem_instruction(OOCore *core, IssueQueueEntry *iq, int iq_size,
                         ROB *rob, LSQ *lsq, IMapEntry *e, BIS *bis)
{
    /* Before dispatching atomic instruction, make sure that all the prior
     * instructions are committed */
    if (e->ins.is_atomic && !cq_empty(&rob->cq))
    {
        return -1;
    }

    if (cq_full(&rob->cq) || iq_full(iq, iq_size) || cq_full(&lsq->cq)
        || no_free_phy_reg(core, e))
    {
        return -1;
    }

    /* Tag this instruction with the BIS index of the earliest preceding branch
     */
    // e->bis_tag = cq_rear(&bis->cq);
    rob_entry_create(rob, e, FALSE);
    iq_entry_create(iq, iq_size, e);
    lsq_entry_create(lsq, e);
    allocate_destination(core, e);
    return 0;
}

static int
dispatch_branch_instruction(OOCore *core, IssueQueueEntry *iq, int iq_size,
                            ROB *rob, BIS *bis, IMapEntry *e)
{
#if 0
    if (cq_full(&rob->cq) || iq_full(iq, iq_size) || cq_full(&bis->cq)
        || no_free_phy_reg(core, e))
    {
        return -1;
    }
#endif

    if (cq_full(&rob->cq) || iq_full(iq, iq_size) || no_free_phy_reg(core, e))
    {
        return -1;
    }

    /* Tag this instruction with the BIS index of the earliest preceding branch
     */
    // e->bis_tag = cq_rear(&bis->cq);
    rob_entry_create(rob, e, FALSE);
    iq_entry_create(iq, iq_size, e);
    // bis_entry_create(bis, e);
    allocate_destination(core, e);
    return 0;
}

void
oo_core_dispatch(OOCore *core)
{
    IMapEntry *e;
    RISCVSIMCPUState *simcpu;

    if (core->dispatch.has_data)
    {
        simcpu = core->simcpu;
        e = &simcpu->imap[core->dispatch.imap_index];

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

                /* Make ROB entry valid so that its processed immediately once it becomes
                 * ROB top */
                rob_entry_create(&core->rob, e, TRUE);
            }
            else
            {
                if (!e->renamed)
                {
                    /* No exception, proceed with renaming */
                    /* Rename the sources */
                    if (e->ins.has_src1)
                    {
                        e->ins.prs1 = core->spec_rat_int[e->ins.rs1];
                    }
                    else if (e->ins.has_fp_src1)
                    {
                        e->ins.prs1 = core->spec_rat_fp[e->ins.rs1];
                    }
                    else
                    {
                        /* Do this, so that this instruction won't wait for rs1,
                         * while in IQ */
                        e->read_rs1 = TRUE;
                    }

                    if (e->ins.has_src2)
                    {
                        e->ins.prs2 = core->spec_rat_int[e->ins.rs2];
                    }
                    else if (e->ins.has_fp_src2)
                    {
                        e->ins.prs2 = core->spec_rat_fp[e->ins.rs2];
                    }
                    else
                    {
                        e->read_rs2 = TRUE;
                    }

                    /* Only FP-FMA instructions have rs3 */
                    if (e->ins.has_fp_src3)
                    {
                        e->ins.prs3 = core->spec_rat_fp[e->ins.rs3];
                    }
                    else
                    {
                        e->read_rs3 = TRUE;
                    }

                    e->renamed = TRUE;
                }

                if (!(e->ins.is_load || e->ins.is_store || e->ins.is_atomic))
                {
                    if (e->ins.is_branch)
                    {
                        if (dispatch_branch_instruction(
                                core, core->iq_int, simcpu->params->iq_int_size,
                                &core->rob, &core->bis, e))
                        {
                            return;
                        }
                    }
                    else
                    {
                        /* Operate instructions */
                        if (e->ins.data_class == INS_CLASS_INT)
                        {
                            if (dispatch_non_mem_instruction(
                                    core, core->iq_int,
                                    simcpu->params->iq_int_size, &core->rob, e,
                                    &core->bis))
                            {
                                return;
                            }
                        }
                        else if (e->ins.data_class == INS_CLASS_FP)
                        {
                            if (dispatch_non_mem_instruction(
                                    core, core->iq_fp,
                                    simcpu->params->iq_fp_size, &core->rob, e,
                                    &core->bis))
                            {
                                return;
                            }
                        }
                    }
                }
                else
                {
                    /* Load Stores Atomics Dispatch */
                    if (dispatch_mem_instruction(
                            core, core->iq_mem, simcpu->params->iq_mem_size,
                            &core->rob, &core->lsq, e, &core->bis))
                    {
                        return;
                    }
                }
            }
            cpu_stage_flush(&core->dispatch);
        }
    }
}
/*=====  End of Instruction Dispatch Stage  ======*/
