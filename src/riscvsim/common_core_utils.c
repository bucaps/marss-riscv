/**
 * Common Utility functions used by In-Order core and Out-of-Order core
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
#include "common_core_utils.h"
#include "../cutils.h"
#include "../riscv_cpu_priv.h"
#include "riscv_isa_decoder_lib.h"

static char cpu_mode_str[][128] = { "U-mode", "S-mode", "H-mode", "M-mode" };

/*  TODO: also free up imap entry associated with this stage
*/
void
cpu_stage_flush(CPUStage *stage)
{
    stage->has_data = FALSE;
    stage->imap_index = -1;
    stage->stage_exec_done = FALSE;
}

void
exec_unit_flush(CPUStage *stage, int num_stages)
{
    int i;

    for (i = 0; i < num_stages; ++i)
    {
        cpu_stage_flush(&stage[i]);
    }
}

/* Verify */
void
speculative_cpu_stage_flush(CPUStage *stage, IMapEntry *imap)
{
    if (stage->imap_index != -1)
    {
        imap[stage->imap_index].status = IMAP_ENTRY_STATUS_FREE;
    }
    cpu_stage_flush(stage);
}

/* Verify */
void
speculative_exec_unit_flush(CPUStage *stage, int num_stages, IMapEntry *imap)
{
    int i;

    for (i = 0; i < num_stages; ++i)
    {
        speculative_cpu_stage_flush(&stage[i], imap);
    }
}

static int
get_free_imap_entry(IMapEntry *imap)
{
    int i;

    for (i = 0; i < NUM_IMAP_ENTRY; ++i)
    {
        if (imap[i].status == IMAP_ENTRY_STATUS_FREE)
        {
            return i;
        }
    }

    return -1;
}

IMapEntry *
allocate_imap_entry(IMapEntry *imap)
{
    IMapEntry *e;
    int imap_index;

    imap_index = get_free_imap_entry(imap);
    assert(imap_index != -1);
    e = &imap[imap_index];
    memset((void *)e, 0, sizeof(IMapEntry));
    e->status = IMAP_ENTRY_STATUS_ALLOCATED;
    e->imap_index = imap_index;
    return e;
}

void
reset_imap(IMapEntry *imap)
{
    int i;

    /* Reset all the instruction map slots */
    for (i = 0; i < NUM_IMAP_ENTRY; ++i)
    {
        imap[i].status = IMAP_ENTRY_STATUS_FREE;
    }
}

IMapEntry *
get_imap_entry(IMapEntry *imap, int index)
{
    return (&imap[index]);
}

int
code_tlb_access_and_ins_fetch(RISCVCPUState *s, IMapEntry *e)
{
    RISCVSIMCPUState *simcpu = s->simcpu;

    if (unlikely(s->code_ptr >= s->code_end))
    {
        uint32_t tlb_idx;
        uint16_t insn_high;
        uint8_t *ptr;
        target_ulong addr = simcpu->pc;

        ++simcpu->stats[s->priv].code_tlb_lookups;

        /* TLB Lookup */
        tlb_idx = (addr >> PG_SHIFT) & (TLB_SIZE - 1);
        if (likely(s->tlb_code[tlb_idx].vaddr == (addr & ~PG_MASK)))
        {
            /* TLB match */
            ptr = (uint8_t *)(s->tlb_code[tlb_idx].mem_addend
                              + (uintptr_t)addr);
            ++simcpu->stats[s->priv].code_tlb_hits;
        }
        else
        {
            if (unlikely(target_read_insn_slow(s, &ptr, addr)))
                goto exception;
        }

        s->code_ptr = ptr;
        s->code_end = ptr + (PG_MASK - 1 - (addr & PG_MASK));
        s->code_to_pc_addend = addr - (uintptr_t)s->code_ptr;

        s->code_guest_paddr = s->tlb_code[tlb_idx].guest_paddr
                              + (addr - s->tlb_code[tlb_idx].vaddr);

        if (unlikely(s->code_ptr >= s->code_end))
        {
            /* Instruction is potentially half way between two pages ? */
            e->ins.binary = *(uint16_t *)(s->code_ptr);
            if ((e->ins.binary & 3) == 3)
            {
                /* Instruction is half way between two pages */
                if (unlikely(target_read_insn_u16(s, &insn_high, addr + 2)))
                    goto exception;
                e->ins.binary |= insn_high << 16;
            }
        }
        else
        {
            e->ins.binary = get_insn32(s->code_ptr);
        }
    }
    else
    {
        /* Fast path */
        e->ins.binary = get_insn32(s->code_ptr);
    }
    return 0;
exception:
    return -1;
}

void
do_fetch_stage_exec(RISCVCPUState *s, IMapEntry *e)
{
    /* Set default minimum page walk latency. If the page walk does occur,
     * hw_pg_tb_wlk_latency will be higher than this default value because it
     * will also include the cache lookup latency for reading/writing page table
     * entries. Page table entries are looked up in L1 data cache. */
    s->hw_pg_tb_wlk_latency = 1;
    s->hw_pg_tb_wlk_stage_id = FETCH;
    s->ins_tlb_lookup_accounted = FALSE;
    s->ins_tlb_hit_accounted = FALSE;

    /* current_latency: number of CPU cycles spent by this instruction
     * in fetch stage so far */
    e->current_latency = 1;
    s->simcpu->mmu->mem_controller->frontend_mem_access_queue.cur_size = 0;

    /* Fetch instruction from TinyEMU memory map */
    if (code_tlb_access_and_ins_fetch(s, e))
    {
        /* This instruction has raised a page fault exception during
         * fetch */
        e->ins.exception = TRUE;
        e->ins.exception_cause = SIM_MMU_EXCEPTION;

        /* Hardware page table walk has been done and its latency must
         * be simulated */
        e->max_latency = s->hw_pg_tb_wlk_latency;
    }
    else
    {
        /* max_latency: Number of CPU cycles required for TLB and Cache
         * look-up */
        e->max_latency = s->hw_pg_tb_wlk_latency
                         + mmu_insn_read(s->simcpu->mmu, s->code_guest_paddr, 4,
                                         FETCH, s->priv);

        if (s->sim_params->enable_l1_caches)
        {
            /* L1 caches and TLB are probed in parallel */
            e->max_latency -= min_int(s->hw_pg_tb_wlk_latency,
                                      s->simcpu->mmu->icache->read_latency);
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

        /* Probe the branch predictor */
        s->simcpu->pfn_branch_frontend_probe_handler(s, e);

        ++s->simcpu->stats[s->priv].ins_fetch;
    }
}

void
do_decode_stage_exec(RISCVCPUState *s, IMapEntry *e)
{
    /* For decoding floating point instructions */
    e->ins.current_fs = s->fs;
    e->ins.rm = get_insn_rm(s, (e->ins.binary >> 12) & 7);

    /* Decode the instruction */
    decode_riscv_binary(&e->ins, e->ins.binary);
}

void
set_exception_state(RISCVCPUState *s, const IMapEntry *e)
{
    s->sim_exception = TRUE;
    s->sim_epc = e->ins.pc;
    s->sim_exception_cause = e->ins.exception_cause;
    s->sim_exception_ins = e->ins.binary;
    strncpy(s->sim_epc_str, e->ins.str, RISCV_INS_STR_MAX_LENGTH);
    s->sim_epc_str[RISCV_INS_STR_MAX_LENGTH - 1] = '\0';
}

void
set_timer_exception_state(RISCVCPUState *s, const IMapEntry *e)
{
    uint32_t insn = e->ins.binary;
    target_ulong pc = e->ins.pc;

    s->sim_exception = TRUE;
    s->sim_exception_cause = SIM_TIMEOUT_EXCEPTION;

    /* In case if the committed instruction was a branch, after processing
       the timer, we must resume at this branch's target if it was taken */
    if (unlikely(e->ins.is_branch && e->is_branch_taken))
    {
        s->sim_epc = e->branch_target;
    }
    else
    {
        /* Else resume at next PC in sequence */
        if ((insn & 3) != 3)
        {
            s->sim_epc = pc + 2; /* Current instruction is compressed */
        }
        else
        {
            s->sim_epc = pc + 4;
        }
    }
}

#define MEMORY_OP_A(size)                                                      \
    {                                                                          \
        uint##size##_t rval;                                                   \
        target_ulong addr = e->ins.mem_addr;                                   \
        uint32_t funct3 = e->ins.binary >> 27;                                 \
        switch (funct3)                                                        \
        {                                                                      \
            case 2: /* lr.w */                                                 \
                if (target_read_u##size(s, &rval, addr))                       \
                    goto mmu_exception;                                        \
                val = (int##size##_t)rval;                                     \
                s->load_res = e->ins.mem_addr;                                 \
                break;                                                         \
            case 3: /* sc.w */                                                 \
                if (s->load_res == addr)                                       \
                {                                                              \
                    if (target_write_u##size(s, addr, e->ins.rs2_val))         \
                        goto mmu_exception;                                    \
                    val = 0;                                                   \
                }                                                              \
                else                                                           \
                {                                                              \
                    val = 1;                                                   \
                }                                                              \
                break;                                                         \
            case 1:    /* amiswap.w */                                         \
            case 0:    /* amoadd.w */                                          \
            case 4:    /* amoxor.w */                                          \
            case 0xc:  /* amoand.w */                                          \
            case 0x8:  /* amoor.w */                                           \
            case 0x10: /* amomin.w */                                          \
            case 0x14: /* amomax.w */                                          \
            case 0x18: /* amominu.w */                                         \
            case 0x1c: /* amomaxu.w */                                         \
                if (target_read_u##size(s, &rval, addr))                       \
                {                                                              \
                    goto mmu_exception;                                        \
                }                                                              \
                val = (int##size##_t)rval;                                     \
                val2 = e->ins.rs2_val;                                         \
                switch (funct3)                                                \
                {                                                              \
                    case 1: /* amiswap.w */                                    \
                        break;                                                 \
                    case 0: /* amoadd.w */                                     \
                        val2 = (int##size##_t)(val + val2);                    \
                        break;                                                 \
                    case 4: /* amoxor.w */                                     \
                        val2 = (int##size##_t)(val ^ val2);                    \
                        break;                                                 \
                    case 0xc: /* amoand.w */                                   \
                        val2 = (int##size##_t)(val & val2);                    \
                        break;                                                 \
                    case 0x8: /* amoor.w */                                    \
                        val2 = (int##size##_t)(val | val2);                    \
                        break;                                                 \
                    case 0x10: /* amomin.w */                                  \
                        if ((int##size##_t)val < (int##size##_t)val2)          \
                            val2 = (int##size##_t)val;                         \
                        break;                                                 \
                    case 0x14: /* amomax.w */                                  \
                        if ((int##size##_t)val > (int##size##_t)val2)          \
                            val2 = (int##size##_t)val;                         \
                        break;                                                 \
                    case 0x18: /* amominu.w */                                 \
                        if ((uint##size##_t)val < (uint##size##_t)val2)        \
                            val2 = (int##size##_t)val;                         \
                        break;                                                 \
                    case 0x1c: /* amomaxu.w */                                 \
                        if ((uint##size##_t)val > (uint##size##_t)val2)        \
                            val2 = (int##size##_t)val;                         \
                        break;                                                 \
                }                                                              \
                if (target_write_u##size(s, addr, val2))                       \
                {                                                              \
                    goto mmu_exception;                                        \
                }                                                              \
                break;                                                         \
        }                                                                      \
    }

static int
execute_atomic(RISCVCPUState *s, IMapEntry *e)
{
    target_ulong val = 0, val2 = 0;

    switch (e->ins.funct3)
    {
        case 2:
            MEMORY_OP_A(32);
            break;
#if BIT_SIZE >= 64
        case 3:
            MEMORY_OP_A(64);
            break;
#endif
    }
    e->ins.buffer = val;
    return 0;
mmu_exception:
    return -1;
}

int
execute_load_store(RISCVCPUState *s, IMapEntry *e)
{
    target_ulong addr = e->ins.mem_addr;

    if (e->ins.is_load)
    {
        switch (e->ins.bytes_to_rw)
        {
            case 1:
            {
                uint8_t rval;
                if (target_read_u8(s, &rval, addr))
                    goto exception;
                if (e->ins.is_unsigned)
                {
                    e->ins.buffer = rval;
                }
                else
                {
                    e->ins.buffer = (int8_t)rval;
                }
                break;
            }

            case 2:
            {
                uint16_t rval;
                if (target_read_u16(s, &rval, addr))
                    goto exception;
                if (e->ins.is_unsigned)
                {
                    e->ins.buffer = rval;
                }
                else
                {
                    e->ins.buffer = (int16_t)rval;
                }
                break;
            }

            case 4:
            {
                uint32_t rval;
                if (target_read_u32(s, &rval, addr))
                    goto exception;
                if (e->ins.is_unsigned)
                {
                    e->ins.buffer = rval;
                }
                else
                {
                    e->ins.buffer = (int32_t)rval;
                }
                break;
            }

            case 8:
            {
                uint64_t rval;
                if (target_read_u64(s, &rval, addr))
                    goto exception;
                if (e->ins.is_unsigned)
                {
                    e->ins.buffer = rval;
                }
                else
                {
                    e->ins.buffer = (int64_t)rval;
                }
                break;
            }
        }
    }
    else if (e->ins.is_store)
    {
        switch (e->ins.bytes_to_rw)
        {
            case 1:
            {
                if (target_write_u8(s, addr, e->ins.rs2_val))
                    goto exception;
                break;
            }

            case 2:
            {
                if (target_write_u16(s, addr, e->ins.rs2_val))
                    goto exception;
                break;
            }

            case 4:
            {
                if (target_write_u32(s, addr, e->ins.rs2_val))
                    goto exception;
                break;
            }

            case 8:
            {
                if (target_write_u64(s, addr, e->ins.rs2_val))
                    goto exception;
                break;
            }
        }
    }
    else if (e->ins.is_atomic)
    {
        if (execute_atomic(s, e))
        {
            goto exception;
        }
    }
    else
    {
        /* Not a memory instruction, no processing needed */
    }
    return 0;
exception:
    return -1;
}

int
get_data_mem_access_latency(RISCVCPUState *s, IMapEntry *e)
{
    int latency = 0;
    RISCVSIMCPUState *simcpu = s->simcpu;

    if ((e->ins.is_load || e->ins.is_atomic_load))
    {
        latency += mmu_data_read(simcpu->mmu, s->data_guest_paddr,
                                 e->ins.bytes_to_rw, MEMORY, s->priv);
    }
    if ((e->ins.is_store || e->ins.is_atomic_store))
    {
        latency += mmu_data_write(simcpu->mmu, s->data_guest_paddr,
                                  e->ins.bytes_to_rw, MEMORY, s->priv);
    }
    if (latency)
    {
        return latency;
    }
    else
    {
        /* For non-memory instructions */
        return 1;
    }
}

void
handle_bpu_frontend_probe(struct RISCVCPUState *s, IMapEntry *e)
{
    target_ulong bpu_target;

    bpu_target = 0;
    bpu_probe(s->simcpu->bpu, e->ins.pc, &e->bpu_resp_pkt, s->priv);
    if (e->bpu_resp_pkt.bpu_probe_status)
    {
        bpu_target = bpu_get_target(s->simcpu->bpu, e->ins.pc,
                                    e->bpu_resp_pkt.btb_entry);

        /* Non-zero target means branch is taken, according to the prediction,
         * so set the predicted address into pcgen unit */
        if (bpu_target)
        {
            s->code_ptr = NULL;
            s->code_end = NULL;
            s->code_to_pc_addend = bpu_target;
        }
    }

    /* Keep track of the predicted address and probe status, to correct
     * miss-prediction later */
    e->predicted_target = bpu_target;
}

void
handle_no_bpu_frontend_probe(struct RISCVCPUState *s, IMapEntry *e)
{
    /* In the absence of BPU, no actions required */
    return;
}

int
handle_branch_decode_no_bpu(struct RISCVCPUState *s, IMapEntry *e)
{
    return FALSE;
}

int
handle_branch_decode_with_bpu(struct RISCVCPUState *s, IMapEntry *e)
{
    target_ulong ras_target = 0;

    /* Add the branch PC into BPU structures if probe during fetch results in
     * miss */
    if (!e->bpu_resp_pkt.bpu_probe_status)
    {
        bpu_add(s->simcpu->bpu, e->ins.pc, e->ins.branch_type, &e->bpu_resp_pkt,
                s->priv, e->ins.is_func_ret);
    }

    /* If return address stack is enabled */
    if (s->simcpu->params->ras_size)
    {
        if (e->ins.is_func_call)
        {
            ras_push(
                s->simcpu->bpu->ras,
                ((e->ins.binary & 3) == 3 ? e->ins.pc + 4 : e->ins.pc + 2));
        }

        if (e->ins.is_func_ret)
        {
            ras_target = ras_pop(s->simcpu->bpu->ras);

            /* Start fetch from address returned by RAS if non-zero */
            if (ras_target)
            {
                s->code_ptr = NULL;
                s->code_end = NULL;
                s->code_to_pc_addend = ras_target;
                e->predicted_target = ras_target;

                s->simcpu->mmu->mem_controller->flush_cpu_stage_queue(
                    &s->simcpu->mmu->mem_controller->frontend_mem_access_queue);

                /* Signal the calling stage to flush previous stages */
                return TRUE;
            }
        }
    }

    /* No flush required because no redirect by RAS */
    return FALSE;
}

/* Handles conditional branches with branch prediction enabled, probes the BPU
 * and updates the entry if BPU hit, corrects the control-flow in the case of
 * direction miss-prediction */
static int
handle_cond_bpu(RISCVCPUState *s, IMapEntry *e)
{
    target_ulong restore_pc;
    int pred = 0;
    int mispredict = FALSE;
    RISCVSIMCPUState *simcpu = s->simcpu;

    bpu_probe(simcpu->bpu, e->ins.pc, &e->bpu_resp_pkt, s->priv);

    if (e->ins.cond)
    {
        /* Branch is resolved to be taken*/
        pred = TRUE;
        if (!e->predicted_target)
        {
            ++simcpu->stats[s->priv].bpu_cond_incorrect;
            e->branch_target = e->ins.target;
            mispredict = TRUE;
        }
        else
        {
            /* TODO: Remove this assert, Prediction Success */
            assert(e->predicted_target == e->ins.target);
            e->is_pred_correct = TRUE;
            e->branch_target = e->predicted_target;
            ++simcpu->stats[s->priv].bpu_cond_correct;
        }
        e->is_branch_taken = TRUE;
    }
    else
    {
        /* Branch resolved to be not-taken */
        if (e->predicted_target)
        {
            /* Miss-prediction occurred, flush the pipeline, repair
               the control flow */
            restore_pc
                = ((e->ins.binary & 3) != 3) ? e->ins.pc + 2 : e->ins.pc + 4;
            ++simcpu->stats[s->priv].bpu_cond_incorrect;
            e->branch_target = restore_pc;
            mispredict = TRUE;
        }
        else
        {
            /* Correct Prediction */
            e->is_pred_correct = TRUE;
            ++simcpu->stats[s->priv].bpu_cond_correct;
        }
        e->is_branch_taken = FALSE;
    }

    /* Update BPU if hit, else skip the update */
    bpu_update(simcpu->bpu, e->ins.pc, e->ins.target, pred, BRANCH_COND,
               &e->bpu_resp_pkt, s->priv);

    return mispredict;
}

/* Handles unconditional branches with branch prediction enabled, probes the BPU
 * and updates the entry if BPU hit, corrects the control-flow in the case of
 * target miss-prediction */
static int
handle_uncond_bpu(RISCVCPUState *s, IMapEntry *e)
{
    RISCVSIMCPUState *simcpu = s->simcpu;
    int type = BRANCH_UNCOND;
    int mispredict = FALSE;
    bpu_probe(simcpu->bpu, e->ins.pc, &e->bpu_resp_pkt, s->priv);

    /* Update BPU if hit, else skip the update */
    bpu_update(simcpu->bpu, e->ins.pc, e->ins.target, TRUE, type,
               &e->bpu_resp_pkt, s->priv);

    if (e->predicted_target == e->ins.target)
    {
        /* Prediction Success */
        e->is_pred_correct = TRUE;

        /* Keep track of this branch to resume at this target, in case
           the timeout happens after this branch commits */
        e->branch_target = e->predicted_target;
        ++simcpu->stats[s->priv].bpu_uncond_correct;
    }
    else
    {
        /* Target Miss-prediction */
        ++simcpu->stats[s->priv].bpu_uncond_incorrect;
        e->branch_target = e->ins.target;
        mispredict = TRUE;
    }

    e->is_branch_taken = TRUE;
    return mispredict;
}

int
handle_branch_with_bpu(struct RISCVCPUState *s, IMapEntry *e)
{
    switch (e->ins.branch_type)
    {
        case BRANCH_UNCOND:
        {
            return handle_uncond_bpu(s, e);
        }
        case BRANCH_COND:
        {
            return handle_cond_bpu(s, e);
        }
    }

    return 0;
}

int
handle_branch_no_bpu(struct RISCVCPUState *s, IMapEntry *e)
{
    int mispredict = FALSE;

    switch (e->ins.branch_type)
    {
        case BRANCH_UNCOND:
        {
            e->is_branch_taken = TRUE;
            e->branch_target = e->ins.target;
            mispredict = TRUE;
            break;
        }
        case BRANCH_COND:
        {
            if (e->ins.cond)
            {
                e->is_branch_taken = TRUE;
                e->branch_target = e->ins.target;
                mispredict = TRUE;
            }
            break;
        }
    }

    return mispredict;
}

void
copy_cache_stats_to_global_stats(struct RISCVCPUState *s)
{
    int i;
    const CacheStats *cache_stats;

    /* Update cache stats */
    if (s->sim_params->enable_l1_caches)
    {
        for (i = 0; i < NUM_MAX_PRV_LEVELS; ++i)
        {
            cache_stats
                = s->simcpu->mmu->icache->get_stats(s->simcpu->mmu->icache);
            s->simcpu->stats[i].icache_read = cache_stats[i].total_read_cnt;
            s->simcpu->stats[i].icache_read_miss = cache_stats[i].read_miss_cnt;

            cache_stats
                = s->simcpu->mmu->dcache->get_stats(s->simcpu->mmu->dcache);
            s->simcpu->stats[i].dcache_read = cache_stats[i].total_read_cnt;
            s->simcpu->stats[i].dcache_read_miss = cache_stats[i].read_miss_cnt;
            s->simcpu->stats[i].dcache_write = cache_stats[i].total_write_cnt;
            s->simcpu->stats[i].dcache_write_miss = cache_stats[i].write_miss_cnt;

            if (s->sim_params->enable_l2_cache)
            {
                cache_stats = s->simcpu->mmu->l2_cache->get_stats(
                    s->simcpu->mmu->l2_cache);
                s->simcpu->stats[i].l2_cache_read = cache_stats[i].total_read_cnt;
                s->simcpu->stats[i].l2_cache_read_miss = cache_stats[i].read_miss_cnt;
                s->simcpu->stats[i].l2_cache_write = cache_stats[i].total_write_cnt;
                s->simcpu->stats[i].l2_cache_write_miss
                    = cache_stats[i].write_miss_cnt;
            }
        }
    }
}

void
sim_print_ins_trace(struct RISCVCPUState *s)
{
    fprintf(s->sim_trace, "cycle=%" TARGET_ULONG_FMT, s->simcpu->clock);
    fprintf(s->sim_trace, " pc=%" TARGET_ULONG_HEX,
            s->simcpu->sim_trace_pkt.e->ins.pc);
    fprintf(s->sim_trace, " insn=%" PRIx32, s->simcpu->sim_trace_pkt.e->ins.binary);
    fprintf(s->sim_trace, " %s", s->simcpu->sim_trace_pkt.e->ins.str);
    fprintf(s->sim_trace, " mode=%s", cpu_mode_str[s->priv]);
    fprintf(s->sim_trace, "\n");
}

void
sim_print_exp_trace(struct RISCVCPUState *s)
{
    fprintf(s->sim_trace, "cycle=%" TARGET_ULONG_FMT, s->simcpu->clock);
    fprintf(s->sim_trace, " pc=%" TARGET_ULONG_HEX, s->sim_epc);
    fprintf(s->sim_trace, " insn=%" PRIx32, s->sim_exception_ins);
    fprintf(s->sim_trace, " %s", s->sim_epc_str);
    fprintf(s->sim_trace, " mode=%s", cpu_mode_str[s->priv]);
    fprintf(s->sim_trace, "\n");
}

void
update_arch_reg_int(RISCVCPUState *s, IMapEntry *e)
{
    if (e->ins.rd)
    {
        s->reg[e->ins.rd] = e->ins.buffer;
        ++s->simcpu->stats[s->priv].int_regfile_writes;
    }
}

void
update_arch_reg_fp(RISCVCPUState *s, IMapEntry *e)
{
    if (e->ins.f32_mask)
    {
        e->ins.buffer |= F32_HIGH;
    }
    else if (e->ins.f64_mask)
    {
        e->ins.buffer |= F64_HIGH;
    }
    s->fp_reg[e->ins.rd] = e->ins.buffer;
    if (e->ins.set_fs)
    {
        s->fs = 3;
    }
    ++s->simcpu->stats[s->priv].fp_regfile_writes;
}

void
update_insn_commit_stats(RISCVCPUState *s, IMapEntry *e)
{
    ++s->simcpu->stats[s->priv].ins_simulated;
    ++s->simcpu->stats[s->priv].ins_type[e->ins.type];

    if ((e->ins.type == INS_TYPE_COND_BRANCH) && e->is_branch_taken)
    {
        ++s->simcpu->stats[s->priv].ins_cond_branch_taken;
    }
}

void
setup_sim_trace_pkt(RISCVCPUState *s, IMapEntry *e)
{
    s->simcpu->sim_trace_pkt.cycle = s->simcpu->clock;
    s->simcpu->sim_trace_pkt.e = e;
    sim_print_ins_trace(s);
}

void
write_stats_to_stats_display_shm(RISCVCPUState *s)
{
    if ((s->simcpu->clock % REALTIME_STATS_CLOCK_CYCLES_INTERVAL) == 0)
    {
        /* Since cache stats are stored separately inside the Cache structure,
         * they have to be copied to global stats structure before writing stats
         * to shared memory. */
        copy_cache_stats_to_global_stats(s);
        memcpy(s->stats_shm_ptr, s->simcpu->stats,
               NUM_MAX_PRV_LEVELS * sizeof(SimStats));
    }
}

int
set_max_latency_for_non_pipe_fu(RISCVCPUState *s, int fu_type, IMapEntry *e)
{
    switch (fu_type)
    {
        case FU_FPU_ALU:
        {
            return s->simcpu->params->fpu_alu_latency[e->ins.fpu_alu_type];
        }
    }

    /* Default */
    return 1;
}