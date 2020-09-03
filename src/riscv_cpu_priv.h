/*
 * RISCV CPU emulator private definitions
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
#ifndef RISCV_CPU_PRIV_H
#define RISCV_CPU_PRIV_H

#include <sys/time.h>
#include <errno.h>
#include <stdio.h>
#include "riscv_cpu.h"
#include "cutils.h"

#include "riscvsim/core/riscv_sim_cpu.h"
#include "riscvsim/utils/sim_params.h"
#include "rtc_timer.h"

#define __exception __attribute__((warn_unused_result))

#define CONFIG_EXT_C /* compressed instructions */

#if defined(EMSCRIPTEN)
#define USE_GLOBAL_STATE
/* use local variables slows down the generated JS code */
#define USE_GLOBAL_VARIABLES
#endif

#include "riscv_cpu_xlen_typedefs.h"

#define CAUSE_MISALIGNED_FETCH    0x0
#define CAUSE_FAULT_FETCH         0x1
#define CAUSE_ILLEGAL_INSTRUCTION 0x2
#define CAUSE_BREAKPOINT          0x3
#define CAUSE_MISALIGNED_LOAD     0x4
#define CAUSE_FAULT_LOAD          0x5
#define CAUSE_MISALIGNED_STORE    0x6
#define CAUSE_FAULT_STORE         0x7
#define CAUSE_USER_ECALL          0x8
#define CAUSE_SUPERVISOR_ECALL    0x9
#define CAUSE_HYPERVISOR_ECALL    0xa
#define CAUSE_MACHINE_ECALL       0xb
#define CAUSE_FETCH_PAGE_FAULT    0xc
#define CAUSE_LOAD_PAGE_FAULT     0xd
#define CAUSE_STORE_PAGE_FAULT    0xf

/* Note: converted to correct bit position at runtime */
#define CAUSE_INTERRUPT  ((uint32_t)1 << 31) 

#define PRV_U 0
#define PRV_S 1
#define PRV_H 2
#define PRV_M 3

/* misa CSR */
#define MCPUID_SUPER   (1 << ('S' - 'A'))
#define MCPUID_USER    (1 << ('U' - 'A'))
#define MCPUID_I       (1 << ('I' - 'A'))
#define MCPUID_M       (1 << ('M' - 'A'))
#define MCPUID_A       (1 << ('A' - 'A'))
#define MCPUID_F       (1 << ('F' - 'A'))
#define MCPUID_D       (1 << ('D' - 'A'))
#define MCPUID_Q       (1 << ('Q' - 'A'))
#define MCPUID_C       (1 << ('C' - 'A'))

/* mstatus CSR */

#define MSTATUS_SPIE_SHIFT 5
#define MSTATUS_MPIE_SHIFT 7
#define MSTATUS_SPP_SHIFT 8
#define MSTATUS_MPP_SHIFT 11
#define MSTATUS_FS_SHIFT 13
#define MSTATUS_UXL_SHIFT 32
#define MSTATUS_SXL_SHIFT 34

#define MSTATUS_UIE (1 << 0)
#define MSTATUS_SIE (1 << 1)
#define MSTATUS_HIE (1 << 2)
#define MSTATUS_MIE (1 << 3)
#define MSTATUS_UPIE (1 << 4)
#define MSTATUS_SPIE (1 << MSTATUS_SPIE_SHIFT)
#define MSTATUS_HPIE (1 << 6)
#define MSTATUS_MPIE (1 << MSTATUS_MPIE_SHIFT)
#define MSTATUS_SPP (1 << MSTATUS_SPP_SHIFT)
#define MSTATUS_HPP (3 << 9)
#define MSTATUS_MPP (3 << MSTATUS_MPP_SHIFT)
#define MSTATUS_FS (3 << MSTATUS_FS_SHIFT)
#define MSTATUS_XS (3 << 15)
#define MSTATUS_MPRV (1 << 17)
#define MSTATUS_SUM (1 << 18)
#define MSTATUS_MXR (1 << 19)
//#define MSTATUS_TVM (1 << 20)
//#define MSTATUS_TW (1 << 21)
//#define MSTATUS_TSR (1 << 22)
#define MSTATUS_UXL_MASK ((uint64_t)3 << MSTATUS_UXL_SHIFT)
#define MSTATUS_SXL_MASK ((uint64_t)3 << MSTATUS_SXL_SHIFT)

#define PG_SHIFT 12
#define PG_MASK ((1 << PG_SHIFT) - 1)
#define TLB_SIZE (s->sim_params->tlb_size)

typedef struct {
    target_ulong vaddr;
    uintptr_t mem_addend;
    target_ulong guest_paddr;
} TLBEntry;

typedef struct RISCVCPUState {
    RISCVCPUCommonState common; /* must be first */
    
    target_ulong pc;
    target_ulong reg[32];

#ifdef USE_GLOBAL_VARIABLES
    /* faster to use global variables with emscripten */
    uint8_t *__code_ptr, *__code_end;
    target_ulong __code_to_pc_addend;
#endif
    
#if FLEN > 0
    fp_uint fp_reg[32];
    uint32_t fflags;
    uint8_t frm;
#endif
    
    uint8_t cur_xlen;  /* current XLEN value, <= MAX_XLEN */
    uint8_t priv; /* see PRV_x */
    uint8_t fs; /* MSTATUS_FS value */
    uint8_t mxl; /* MXL field in MISA register */
    
    int32_t n_cycles; /* only used inside the CPU loop */
    uint64_t insn_counter;
    BOOL power_down_flag;
    int pending_exception; /* used during MMU exception handling */
    target_ulong pending_tval;
    
    /* CSRs */
    target_ulong mstatus;
    target_ulong mtvec;
    target_ulong mscratch;
    target_ulong mepc;
    target_ulong mcause;
    target_ulong mtval;
    target_ulong mhartid; /* ro */
    uint32_t misa;
    uint32_t mie;
    uint32_t mip;
    uint32_t medeleg;
    uint32_t mideleg;
    uint32_t mcounteren;
    
    target_ulong stvec;
    target_ulong sscratch;
    target_ulong sepc;
    target_ulong scause;
    target_ulong stval;
#if MAX_XLEN == 32
    uint32_t satp;
#else
    uint64_t satp; /* currently 64 bit physical addresses max */
#endif
    uint32_t scounteren;

    target_ulong load_res; /* for atomic LR/SC */

    PhysMemoryMap *mem_map;

    TLBEntry *tlb_read;
    TLBEntry *tlb_write;
    TLBEntry *tlb_code;

    /* used to fetch instructions from TinyEMU memory map */
    uint8_t *code_ptr, *code_end;
    target_ulong code_to_pc_addend;

    /* These are the physical addresses for current instruction and data memory
     * from TinyEMU physical memory map defined in virt_machine.c. They are used
     * for look-up into memory hierarchy by the simulated core. */
    target_ulong code_guest_paddr;
    target_ulong data_guest_paddr;

    /* Track whether the current data memory access was a device IO or RAM IO */
    int is_device_io;
    int hw_pg_tb_wlk_stage_id;  /* id of the stage (FETCH, MEMORY) which initiated page table walk */
    int ins_tlb_lookup_accounted;
    int ins_tlb_hit_accounted;

    SimParams *sim_params;
    RTC *rtc;

    /* simulated RISC-V core*/
    RISCVSIMCPUState *simcpu;
  } RISCVCPUState;

/* Note: Below declared functions are accessed from both simulation and emulation side */
int get_insn_rm(RISCVCPUState *s, unsigned int rm);

no_inline __exception int
target_read_insn_slow(RISCVCPUState *s, uint8_t **pptr, target_ulong addr);

__exception int target_read_insn_u16(RISCVCPUState *s, uint16_t *pinsn,
                                       target_ulong addr);

uint32_t get_insn32(uint8_t *ptr);

#define target_read_slow glue(glue(riscv, MAX_XLEN), _read_slow)
#define target_write_slow glue(glue(riscv, MAX_XLEN), _write_slow)

DLL_PUBLIC int target_read_slow(RISCVCPUState *s, mem_uint_t *pval,
                                target_ulong addr, int size_log2);
DLL_PUBLIC int target_write_slow(RISCVCPUState *s, target_ulong addr,
                                 mem_uint_t val, int size_log2);

/* return 0 if OK, != 0 if exception */
#define TARGET_READ_WRITE(size, uint_type, size_log2)                          \
    static inline __exception int target_read_u##size(                         \
        RISCVCPUState *s, uint_type *pval, target_ulong addr)                  \
    {                                                                          \
        uint32_t tlb_idx;                                                      \
                                                                               \
        s->is_device_io = 0;                                                   \
        tlb_idx = (addr >> PG_SHIFT) & (TLB_SIZE - 1);                         \
        if (s->simcpu->simulation)                                             \
        {                                                                      \
            ++s->simcpu->stats[s->priv].load_tlb_lookups;                      \
        }                                                                      \
        if (likely(s->tlb_read[tlb_idx].vaddr                                  \
                   == (addr & ~(PG_MASK & ~((size / 8) - 1)))))                \
        {                                                                      \
            *pval = *(uint_type *)(s->tlb_read[tlb_idx].mem_addend             \
                                   + (uintptr_t)addr);                         \
            if (s->simcpu->simulation)                                         \
            {                                                                  \
                ++s->simcpu->stats[s->priv].load_tlb_hits;                     \
            }                                                                  \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            mem_uint_t val;                                                    \
            int ret;                                                           \
            ret = target_read_slow(s, &val, addr, size_log2);                  \
            if (ret)                                                           \
                return ret;                                                    \
            *pval = val;                                                       \
        }                                                                      \
                                                                               \
        if (!s->is_device_io)                                                  \
        {                                                                      \
            s->data_guest_paddr = s->tlb_read[tlb_idx].guest_paddr             \
                                  + (addr - s->tlb_read[tlb_idx].vaddr);       \
        }                                                                      \
        return 0;                                                              \
    }                                                                          \
                                                                               \
    static inline __exception int target_write_u##size(                        \
        RISCVCPUState *s, target_ulong addr, uint_type val)                    \
    {                                                                          \
        uint32_t tlb_idx;                                                      \
                                                                               \
        s->is_device_io = 0;                                                   \
        tlb_idx = (addr >> PG_SHIFT) & (TLB_SIZE - 1);                         \
        if (s->simcpu->simulation)                                             \
        {                                                                      \
            ++s->simcpu->stats[s->priv].store_tlb_lookups;                     \
        }                                                                      \
        if (likely(s->tlb_write[tlb_idx].vaddr                                 \
                   == (addr & ~(PG_MASK & ~((size / 8) - 1)))))                \
        {                                                                      \
            *(uint_type *)(s->tlb_write[tlb_idx].mem_addend + (uintptr_t)addr) \
                = val;                                                         \
            if (s->simcpu->simulation)                                         \
            {                                                                  \
                ++s->simcpu->stats[s->priv].store_tlb_hits;                    \
            }                                                                  \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            int ret;                                                           \
            ret = target_write_slow(s, addr, val, size_log2);                  \
            if (ret)                                                           \
                return ret;                                                    \
        }                                                                      \
                                                                               \
        if (!s->is_device_io)                                                  \
        {                                                                      \
            s->data_guest_paddr = s->tlb_write[tlb_idx].guest_paddr            \
                                  + (addr - s->tlb_write[tlb_idx].vaddr);      \
        }                                                                      \
        return 0;                                                              \
    }

TARGET_READ_WRITE(8, uint8_t, 0)
TARGET_READ_WRITE(16, uint16_t, 1)
TARGET_READ_WRITE(32, uint32_t, 2)
#if MLEN >= 64
TARGET_READ_WRITE(64, uint64_t, 3)
#endif
#if MLEN >= 128
TARGET_READ_WRITE(128, uint128_t, 4)
#endif

#endif /* RISCV_CPU_PRIV_H */
