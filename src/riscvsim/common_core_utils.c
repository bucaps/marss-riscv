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
#include "../riscv_cpu_shared.h"

static char cpu_mode_str[][128] = { "PRV_U", "PRV_S", "PRV_H", "PRV_M" };

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
    e->status = IMAP_ENTRY_STATUS_STATUS_ALLOCATED;
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

int
code_tlb_access_and_ins_fetch(RISCVCPUState *s, IMapEntry *e)
{
    RISCVSIMCPUState *simcpu = s->simcpu;

    if (unlikely(s->code_ptr >= s->code_end))
    {
        uint32_t tlb_idx;
        uint16_t insn_high;
        uintptr_t mem_addend;
        target_ulong addr = simcpu->pc;

        ++simcpu->stats[s->priv].code_tlb_lookups;

        /* TLB Lookup */
        tlb_idx = (addr >> PG_SHIFT) & (TLB_SIZE - 1);
        if (likely(s->tlb_code[tlb_idx].vaddr == (addr & ~PG_MASK)))
        {
            /* TLB match */
            mem_addend = s->tlb_code[tlb_idx].mem_addend;
            ++simcpu->stats[s->priv].code_tlb_hits;
        }
        else
        {
            if (unlikely(target_read_insn_slow(s, &mem_addend, addr)))
                goto exception;
        }

        s->code_ptr = (uint8_t *)(mem_addend + (uintptr_t)addr);
        s->code_end = (uint8_t *)(mem_addend + (uintptr_t)((addr & ~PG_MASK)
                                                           + PG_MASK - 1));

        s->code_to_pc_addend = addr - (uintptr_t)(s->code_ptr);
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
        uint##size##_t addr = e->ins.mem_addr;                                 \
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
            e->is_branch_taken = TRUE;
            e->branch_target = e->predicted_target;
            ++simcpu->stats[s->priv].bpu_cond_correct;
        }
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
        e->is_branch_taken = TRUE;

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
            cache_stats = get_cache_stats(s->simcpu->mmu->icache);
            s->simcpu->stats[i].l1i_read = cache_stats[i].total_read_cnt;
            s->simcpu->stats[i].l1i_read_miss = cache_stats[i].read_miss_cnt;

            cache_stats = get_cache_stats(s->simcpu->mmu->dcache);
            s->simcpu->stats[i].l1d_read = cache_stats[i].total_read_cnt;
            s->simcpu->stats[i].l1d_read_miss = cache_stats[i].read_miss_cnt;
            s->simcpu->stats[i].l1d_write = cache_stats[i].total_write_cnt;
            s->simcpu->stats[i].l1d_write_miss = cache_stats[i].write_miss_cnt;

            if (s->sim_params->enable_l2_cache)
            {
                cache_stats = get_cache_stats(s->simcpu->mmu->l2_cache);
                s->simcpu->stats[i].l2_read = cache_stats[i].total_read_cnt;
                s->simcpu->stats[i].l2_read_miss = cache_stats[i].read_miss_cnt;
                s->simcpu->stats[i].l2_write = cache_stats[i].total_write_cnt;
                s->simcpu->stats[i].l2_write_miss
                    = cache_stats[i].write_miss_cnt;
            }
        }
    }
}

void
print_ins_trace(struct RISCVCPUState *s, uint64_t cycle, target_ulong pc,
                uint32_t insn, const char *insn_str, int mode,
                const char *exception)
{
    fprintf(s->sim_trace, "cycle = %-10" PRIu64 " pc = 0x%-12" TARGET_ULONG_HEX
                          " insn = 0x%-12" PRIx32,
            cycle, pc, insn);

    if (s->sim_params->create_ins_str)
    {
        fprintf(s->sim_trace, " %-24s", insn_str);
    }

    fprintf(s->sim_trace, " mode = %-6s status = %s\n", cpu_mode_str[mode],
            exception);
}