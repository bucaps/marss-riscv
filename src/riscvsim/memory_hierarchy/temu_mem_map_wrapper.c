/**
 * TinyEMU memory map wrapper
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
#include "temu_mem_map_wrapper.h"
#include "../../riscv_cpu_priv.h"

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
temu_exec_atomic_insn(RISCVCPUState *s, InstructionLatch *e)
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

static int
temu_exec_load_store_insn(RISCVCPUState *s, InstructionLatch *e)
{
    target_ulong addr = e->ins.mem_addr;

    /* TLB lookup in target_read/write functions will fill this variable with
     * the guest data physical address for this memory access */
    s->data_guest_paddr = 0;

    if (e->ins.is_load)
    {
        switch (e->ins.bytes_to_rw)
        {
            case 1:
            {
                uint8_t rval;
                if (target_read_u8(s, &rval, addr))
                    goto mmu_exception;
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
                    goto mmu_exception;
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
                    goto mmu_exception;
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
                    goto mmu_exception;
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
                    goto mmu_exception;
                break;
            }

            case 2:
            {
                if (target_write_u16(s, addr, e->ins.rs2_val))
                    goto mmu_exception;
                break;
            }

            case 4:
            {
                if (target_write_u32(s, addr, e->ins.rs2_val))
                    goto mmu_exception;
                break;
            }

            case 8:
            {
                if (target_write_u64(s, addr, e->ins.rs2_val))
                    goto mmu_exception;
                break;
            }
        }
    }
    else if (e->ins.is_atomic)
    {
        if (temu_exec_atomic_insn(s, e))
        {
            goto mmu_exception;
        }
    }
    return 0;
mmu_exception:
    return -1;
}

static int
temu_read_insn(RISCVCPUState *s, InstructionLatch *e)
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
                goto mmu_exception;
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
                    goto mmu_exception;
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
mmu_exception:
    return -1;
}

TemuMemMapWrapper *
temu_mem_map_wrapper_init()
{
    TemuMemMapWrapper *t;

    t = calloc(1, sizeof(TemuMemMapWrapper));
    assert(t);

    t->read_insn = &temu_read_insn;
    t->exec_load_store_atomic = &temu_exec_load_store_insn;

    return t;
}

void
temu_mem_map_wrapper_free(TemuMemMapWrapper **t)
{
    free(*t);
    *t = NULL;
}