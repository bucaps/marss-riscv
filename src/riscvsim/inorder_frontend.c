/*
 * In-order Pipeline Front-end Stages: pcgen, fetch and decode
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
#include "inorder.h"
#include "../riscv_cpu_priv.h"
#include "bpu.h"
#include "circular_queue.h"
#include "riscv_isa_decoder_lib.h"
#include "riscv_sim_cpu.h"

/*===========================================
=            PC Generation Stage            =
===========================================*/

void
in_core_pcgen(INCore *core)
{
    IMapEntry *e;
    RISCVCPUState *s;
    RISCVSIMCPUState *simcpu;

    if (core->pcgen.has_data)
    {
        s = core->simcpu->emu_cpu_state;
        simcpu = core->simcpu;
        if (!core->pcgen.stage_exec_done)
        {
            /* Calculate current PC*/
            simcpu->pc
                = (target_ulong)((uintptr_t)s->code_ptr + s->code_to_pc_addend);

            /* Allocate an entry for this instruction in imap */
            e = allocate_imap_entry(simcpu->imap);

            /* Setup the allocated imap entry */
            e->ins_dispatch_id = core->ins_dispatch_id++;
            e->ins.pc = simcpu->pc;
            e->ins.create_str = s->sim_params->create_ins_str;

            /* Store IMAP index in the stage and the actual decoded instruction
             * info is stored in this imap entry.
             * NOTE: This avoids copying of whole decoded instruction info when
             * instruction flows to next stage */
            core->pcgen.imap_index = e->imap_index;
            core->pcgen.stage_exec_done = TRUE;
        }

        /* If next stage is free, pass this instruction to the next stage, else
         * stall */
        if (!core->fetch.has_data)
        {
            core->pcgen.stage_exec_done = FALSE;
            core->fetch = core->pcgen;
            core->pcgen.imap_index = -1;
        }
    }
}

/*=====  End of PC Generation Stage  ======*/

/*===============================================
=            Instruction Fetch Stage            =
===============================================*/

void
in_core_fetch(INCore *core)
{
    IMapEntry *e;
    RISCVCPUState *s;
    RISCVSIMCPUState *simcpu;

    if (core->fetch.has_data)
    {
        s = core->simcpu->emu_cpu_state;
        simcpu = core->simcpu;
        e = &simcpu->imap[core->fetch.imap_index];

        if (!core->fetch.stage_exec_done)
        {
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

                /* Stop pcgen stage and save exception context */
                cpu_stage_flush(&core->pcgen);
                set_exception_state(s, e);
            }
            else
            {
                /* max_latency: Number of CPU cycles required for TLB and Cache
                 * look-up */
                e->max_latency
                    = s->hw_pg_tb_wlk_latency
                      + mmu_insn_read(simcpu->mmu, s->code_guest_paddr, 4,
                                      FETCH, s->priv);

                /* If true, it indicates that some sort of memory access request
                 * are sent to memory controller for this instruction, so
                 * request the fast wrap-around read for this address */
                if (simcpu->mmu->mem_controller->frontend_mem_access_queue
                        .cur_size)
                {
                    mem_controller_req_fast_read_for_addr(
                        &simcpu->mmu->mem_controller->frontend_mem_access_queue,
                        s->code_guest_paddr);
                }

                if (s->sim_params->enable_l1_caches)
                {
                    /* L1 caches and TLB are probed in parallel */
                    e->max_latency
                        -= min_int(s->hw_pg_tb_wlk_latency,
                                   simcpu->mmu->icache->probe_latency);
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

                core->simcpu->pfn_branch_frontend_probe_handler(
                    core->simcpu->emu_cpu_state, e);

                ++simcpu->stats[s->priv].ins_fetch;
            }
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

            /* Check if the dram accesses, if required for this instruction,
             * are completed */
            if (!simcpu->mmu->mem_controller->frontend_mem_access_queue
                     .cur_size)
            {
                /* Memory controller read logic will install the tag in the cache line with
                 * the first word read, while the remaining words are still
                 * being fetched. This may cause a false hit on the following
                 * words. Check the memory controller to see if the word is
                 * received. Only then, proceed further. */
                if (mem_controller_wrap_around_read_pending(simcpu->mmu->mem_controller,
                                             s->code_guest_paddr))
                {
                    return;
                }

                /* Stop fetching new instructions on a MMU exception */
                if (e->ins.exception)
                {
                    cpu_stage_flush(&core->fetch);
                }
                else
                {
                    /* No MMU exception, it is safe to pass instruction to
                     * decode stage */
                    if (!core->decode.has_data)
                    {
                        simcpu->mmu->mem_controller->frontend_mem_access_queue
                            .cur_idx
                            = 0;

                        core->fetch.stage_exec_done = FALSE;
                        e->max_latency = 0;
                        e->current_latency = 0;
                        core->decode = core->fetch;
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

static void
read_int_operand(INCore *core, int has_src, int *read_rs, int rs,
                 uint64_t *buffer, int *reg_file_read_done)
{
    int i;

    if (has_src && !(*read_rs))
    {
        if (!core->int_reg_status[rs])
        {
            for (i = 0; i < NUM_FWD_BUS; ++i)
            {
                if (core->fwd_latch[i].valid && core->fwd_latch[i].int_dest
                    && (core->fwd_latch[i].rd == rs))
                {
                    *buffer = (target_ulong)core->fwd_latch[i].buffer;
                    *read_rs = TRUE;
                    break;
                }
            }
        }
        else
        {
            *buffer = (target_ulong)core->simcpu->emu_cpu_state->reg[rs];
            *read_rs = TRUE;
            *reg_file_read_done = TRUE;
        }
    }
}

static void
read_fp_operand(INCore *core, int has_src, int *read_rs, int rs,
                uint64_t *buffer, int *reg_file_read_done)
{
    int i;

    if (has_src && !(*read_rs))
    {
        if (!core->fp_reg_status[rs])
        {
            /* Floating point execution units start from ID 3 onwards */
            for (i = 3; i < NUM_FWD_BUS; ++i)
            {
                if (core->fwd_latch[i].valid && core->fwd_latch[i].fp_dest
                    && (core->fwd_latch[i].rd == rs))
                {
                    *buffer = core->fwd_latch[i].buffer;
                    *read_rs = TRUE;
                    break;
                }
            }
        }
        else
        {
            *buffer = core->simcpu->emu_cpu_state->fp_reg[rs];
            *read_rs = TRUE;
            *reg_file_read_done = TRUE;
        }
    }
}

void
in_core_decode(INCore *core)
{
    IMapEntry *e;
    RISCVCPUState *s;
    int ins_issue_index;
    int read_int_rf = 0;
    int read_fp_rf = 0;
    target_ulong ras_target = 0;

    if (core->decode.has_data)
    {
        s = core->simcpu->emu_cpu_state;
        e = &core->simcpu->imap[core->decode.imap_index];
        if (!core->decode.stage_exec_done)
        {
            if (!e->is_decoded)
            {
                /* For decoding floating point instructions */
                e->ins.current_fs = s->fs;
                e->ins.rm = get_insn_rm(s, (e->ins.binary >> 12) & 7);

                /* Decode the instruction */
                decode_riscv_binary(&e->ins, e->ins.binary);

                /* If branch prediction is enabled and this instruction is a
                 * branch, add BPU entry for this branch if not present */
                if (s->sim_params->enable_bpu && e->ins.is_branch)
                {
                    if (!e->bpu_resp_pkt.bpu_probe_status)
                    {
                        bpu_add(core->simcpu->bpu, e->ins.pc,
                                e->ins.branch_type, &e->bpu_resp_pkt, s->priv,
                                e->ins.is_func_ret);
                        ++core->simcpu->stats[s->priv].btb_miss_for_branches;
                    }

                    /* If return address stack is enabled */
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

                                /* Restart PCGEN */
                                speculative_cpu_stage_flush(&core->pcgen, core->simcpu->imap);
                                core->pcgen.has_data = TRUE;

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

                                speculative_cpu_stage_flush(&core->fetch, core->simcpu->imap);
                            }
                        }
                    }
                }

                e->is_decoded = TRUE;
            }

            /* Handle exception caused during decoding */
            if (unlikely(e->ins.exception))
            {
                set_exception_state(s, e);
                cpu_stage_flush(&core->pcgen);
                cpu_stage_flush(&core->fetch);
                cpu_stage_flush(&core->decode);
                return;
            }

            /* Read integer operands */
            read_int_operand(core, e->ins.has_src1, &e->read_rs1, e->ins.rs1,
                             &e->ins.rs1_val, &read_int_rf);
            read_int_operand(core, e->ins.has_src2, &e->read_rs2, e->ins.rs2,
                             &e->ins.rs2_val, &read_int_rf);

            /* Read floating-point operands */
            read_fp_operand(core, e->ins.has_fp_src1, &e->read_rs1, e->ins.rs1,
                            &e->ins.rs1_val, &read_fp_rf);
            read_fp_operand(core, e->ins.has_fp_src2, &e->read_rs2, e->ins.rs2,
                            &e->ins.rs2_val, &read_fp_rf);
            read_fp_operand(core, e->ins.has_fp_src3, &e->read_rs3, e->ins.rs3,
                            &e->ins.rs3_val, &read_fp_rf);

            /* Stall if any of the source register value is not available */
            if (((e->ins.has_src1 || e->ins.has_fp_src1) && !e->read_rs1)
                || ((e->ins.has_src2 || e->ins.has_fp_src2) && !e->read_rs2)
                || (e->ins.has_fp_src3 && !e->read_rs3))
            {
                goto exit_decode;
            }

            /* Check for WAW hazard (dependency on destination register) on
             * integer rd */
            if (e->ins.has_dest)
            {
                if (!core->int_reg_status[e->ins.rd])
                {
                    goto exit_decode;
                }
            }

            /* Check for WAW hazard (dependency on destination register) on fp
             * rd */
            if (e->ins.has_fp_dest)
            {
                if (!core->fp_reg_status[e->ins.rd])
                {
                    goto exit_decode;
                }
            }

            core->decode.stage_exec_done = TRUE;
        }

        /* Decoding is complete and all the resources are acquired. If the
         * required FU is free, issue the instruction to the appropriate FU,
         * else stall */
        switch (e->ins.fu_type)
        {
            case FU_ALU:
                if (!core->ialu[0].has_data)
                {
                    core->decode.stage_exec_done = FALSE;
                    core->ialu[0] = core->decode;
                }
                else
                {
                    goto exit_decode;
                }
                break;
            case FU_MUL:
                if (!core->imul[0].has_data)
                {
                    core->decode.stage_exec_done = FALSE;
                    core->imul[0] = core->decode;
                }
                else
                {
                    goto exit_decode;
                }
                break;
            case FU_DIV:
                if (!core->idiv[0].has_data)
                {
                    core->decode.stage_exec_done = FALSE;
                    core->idiv[0] = core->decode;
                }
                else
                {
                    goto exit_decode;
                }
                break;
            case FU_FPU_ALU:
                if (!core->fpu_alu[0].has_data)
                {
                    core->decode.stage_exec_done = FALSE;
                    core->fpu_alu[0] = core->decode;
                }
                else
                {
                    goto exit_decode;
                }
                break;
            case FU_FPU_FMA:
                if (!core->fpu_fma[0].has_data)
                {
                    core->decode.stage_exec_done = FALSE;
                    core->fpu_fma[0] = core->decode;
                }
                else
                {
                    goto exit_decode;
                }
                break;
        }

        /* Add sequence number of this instruction to memory selection queue */
        ins_issue_index = cq_enqueue(&core->ins_dispatch_queue.cq);
        assert(ins_issue_index != -1);
        core->ins_dispatch_queue.data[ins_issue_index] = e->ins_dispatch_id;
        cpu_stage_flush(&core->decode);
    }
exit_decode:
    if (read_int_rf)
    {
        core->simcpu->stats[s->priv].int_regfile_reads++;
    }

    if (read_fp_rf)
    {
        core->simcpu->stats[s->priv].fp_regfile_reads++;
    }
}

/*=====  End of Instruction Decode Stage  ======*/
