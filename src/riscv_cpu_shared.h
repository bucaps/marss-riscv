/*
 * RISCV CPU emulator
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
#ifndef _RISCV_CPU_SHARED_H_
#define _RISCV_CPU_SHARED_H_

#include <sys/time.h>
#include <limits.h>
#include <errno.h>
#include <stdio.h>
#include <time.h>
#include <semaphore.h>

#include "sim_params_stats.h"
#include "cutils.h"
#include "iomem.h"
#include "riscvsim/riscv_sim_cpu.h"

#if defined(RV32)
#define MAX_XLEN 32
#define BIT_SIZE 32
#elif defined(RV64)
#define MAX_XLEN 64
#define BIT_SIZE 64
#endif

#ifndef FLEN
#if MAX_XLEN == 128
#define FLEN 128
#else
#define FLEN 64
#endif
#endif /* !FLEN */

#define CONFIG_EXT_C /* compressed instructions */

//#define DUMP_INVALID_MEM_ACCESS
//#define DUMP_MMU_EXCEPTIONS
//#define DUMP_INTERRUPTS
//#define DUMP_INVALID_CSR
//#define DUMP_EXCEPTIONS
//#define DUMP_CSR
//#define CONFIG_LOGFILE

#if defined(EMSCRIPTEN)
#define USE_GLOBAL_STATE
/* use local variables slows down the generated JS code */
#define USE_GLOBAL_VARIABLES
#endif

#if FLEN > 0
#include "softfp.h"
#endif

#define __exception __attribute__((warn_unused_result))

#if MAX_XLEN == 32
typedef uint32_t target_ulong;
typedef int32_t target_long;
#define PR_target_ulong "08x"
#elif MAX_XLEN == 64
typedef uint64_t target_ulong;
typedef int64_t target_long;
#define PR_target_ulong "016" PRIx64
#elif MAX_XLEN == 128
typedef uint128_t target_ulong;
typedef int128_t target_long;
#define PR_target_ulong "016" PRIx64 /* XXX */
#else
#error unsupported MAX_XLEN
#endif

/* FLEN is the floating point register width */
#if FLEN > 0
#if FLEN == 32
typedef uint32_t fp_uint;
#define F32_HIGH 0
#elif FLEN == 64
typedef uint64_t fp_uint;
#define F32_HIGH ((fp_uint)-1 << 32)
#define F64_HIGH 0
#elif FLEN == 128
typedef uint128_t fp_uint;
#define F32_HIGH ((fp_uint)-1 << 32)
#define F64_HIGH ((fp_uint)-1 << 64)
#else
#error unsupported FLEN
#endif
#endif

/* MLEN is the maximum memory access width */
#if MAX_XLEN <= 32 && FLEN <= 32
#define MLEN 32
#elif MAX_XLEN <= 64 && FLEN <= 64
#define MLEN 64
#else
#define MLEN 128
#endif

#if MLEN == 32
typedef uint32_t mem_uint_t;
#elif MLEN == 64
typedef uint64_t mem_uint_t;
#elif MLEN == 128
typedef uint128_t mem_uint_t;
#else
#unsupported MLEN
#endif

#define CAUSE_MISALIGNED_FETCH 0x0
#define CAUSE_FAULT_FETCH 0x1
#define CAUSE_ILLEGAL_INSTRUCTION 0x2
#define CAUSE_BREAKPOINT 0x3
#define CAUSE_MISALIGNED_LOAD 0x4
#define CAUSE_FAULT_LOAD 0x5
#define CAUSE_MISALIGNED_STORE 0x6
#define CAUSE_FAULT_STORE 0x7
#define CAUSE_USER_ECALL 0x8
#define CAUSE_SUPERVISOR_ECALL 0x9
#define CAUSE_HYPERVISOR_ECALL 0xa
#define CAUSE_MACHINE_ECALL 0xb
#define CAUSE_FETCH_PAGE_FAULT 0xc
#define CAUSE_LOAD_PAGE_FAULT 0xd
#define CAUSE_STORE_PAGE_FAULT 0xf

/* Note: converted to correct bit position at runtime */
#define CAUSE_INTERRUPT ((uint32_t)1 << 31)

#define PRV_U 0
#define PRV_S 1
#define PRV_H 2
#define PRV_M 3

/* misa CSR */
#define MCPUID_SUPER (1 << ('S' - 'A'))
#define MCPUID_USER (1 << ('U' - 'A'))
#define MCPUID_I (1 << ('I' - 'A'))
#define MCPUID_M (1 << ('M' - 'A'))
#define MCPUID_A (1 << ('A' - 'A'))
#define MCPUID_F (1 << ('F' - 'A'))
#define MCPUID_D (1 << ('D' - 'A'))
#define MCPUID_Q (1 << ('Q' - 'A'))
#define MCPUID_C (1 << ('C' - 'A'))

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

#define START_SIM_TIMER(start) clock_gettime(CLOCK_MONOTONIC, &start)
#define STOP_SIM_TIMER(end) clock_gettime(CLOCK_MONOTONIC, &end)
#define GET_SIM_TIMER_DIFF(start, end)                                             \
  (1000000000L * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec)

#define REALTIME_STATS_CLOCK_CYCLES_INTERVAL 500000

/* see sim_exception_cause field inside RISCVCPUState */
#define SIM_EXCEPTION 1
#define SIM_COMPLEX_OPCODE 2
#define SIM_TIMEOUT_EXCEPTION 3

typedef struct
{
  target_ulong vaddr;
  uintptr_t mem_addend;
  target_ulong guest_paddr;
} TLBEntry;

typedef struct RISCVCPUState
{
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

  uint8_t cur_xlen; /* current XLEN value, <= MAX_XLEN */
  uint8_t priv;     /* see PRV_x */
  uint8_t fs;       /* MSTATUS_FS value */
  uint8_t mxl;      /* MXL field in MISA register */

  uint64_t insn_counter;
  BOOL power_down_flag;
  int pending_exception; /* used during MMU exception handling */
  target_ulong pending_tval;

  /* PRV_M CSRs */
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

  /* PRV_S CSRs */
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

  PhysMemoryMap* mem_map;

  TLBEntry *tlb_read;
  TLBEntry *tlb_write;
  TLBEntry *tlb_code;

  struct timespec sim_start, sim_end; /* to measure simulation time */

  /* used to fetch instructions from RISCVEMU memory map */
  uint8_t *code_ptr, *code_end;
  target_ulong code_to_pc_addend;

  /* Note: These physical addresses are from RISCVEMU physical memory map defined in virt_machine.c.
     They are used for look-up into cache hierarchy by the simulated core. */
  target_ulong code_guest_paddr; /* keep track of the physical address of the current instruction fetched */
  target_ulong data_guest_paddr; /* Keep track of the physical address of the current load-store memory access */
  int is_device_io;              /* keep track whether the current data memory access was a device IO or RAM IO */


  int hw_pg_tb_wlk_latency_accounted;
  int hw_pg_tb_wlk_latency;   /* latency for reading/writing page table entries during page walk */
  int hw_pg_tb_wlk_stage_id;  /* id of the stage (FETCH, MEMORY) which initiated page table walk */
  int ins_page_faults_accounted;
  int load_page_faults_accounted;
  int store_page_faults_accounted;

  /* simulated RISC-V core*/
  RISCVSIMCPUState *simcpu;

  int n_cycles;

  FILE* sim_trace;

  /* used for setting up shared memory area to dump stats into, which can be
   read by stats-display tool */
  int stats_shm_fd;
  char stats_shm_name[256];
  SimStats *stats_shm_ptr;

  /* set when an exception is encountered in simulation mode */
  int sim_exception;
  target_ulong sim_epc;
  int sim_exception_cause;
  int sim_exception_stage;
  uint32_t sim_exception_ins;
  char sim_epc_str[RISCV_INS_STR_MAX_LENGTH];

  /* simulation control */
  int start_simulation;
  int stop_simulation;
  int simulation;
  int return_to_sim;

  SimParams *sim_params;
} RISCVCPUState;

/* Note: Below declared functions are accessed from both simulation and emulation side */

int get_insn_rm(RISCVCPUState* s, unsigned int rm);

/* used to read instruction from RISCVEMU memory map */

/* unaligned access at an address known to be a multiple of 2 */
uint32_t get_insn32(uint8_t* ptr);

/* return 0 if OK, != 0 if exception */
__exception int target_read_insn_slow(RISCVCPUState* s, uintptr_t* pmem_addend,
                                      target_ulong addr);

/* addr must be aligned */
__exception int target_read_insn_u16(RISCVCPUState* s, uint16_t* pinsn,
                                     target_ulong addr);

/* used by loads and stores to read/write data from RISCVEMU memory map */
__exception int target_read_u8(RISCVCPUState* s, uint8_t* pval,
                               target_ulong addr);

__exception int target_read_u16(RISCVCPUState* s, uint16_t* pval,
                                target_ulong addr);

__exception int target_read_u32(RISCVCPUState* s, uint32_t* pval,
                                target_ulong addr);

__exception int target_read_u64(RISCVCPUState* s, uint64_t* pval,
                                target_ulong addr);

__exception int target_write_u8(RISCVCPUState* s, target_ulong addr,
                                uint8_t val);

__exception int target_write_u16(RISCVCPUState* s, target_ulong addr,
                                 uint16_t val);

__exception int target_write_u32(RISCVCPUState* s, target_ulong addr,
                                 uint32_t val);

__exception int target_write_u64(RISCVCPUState* s, target_ulong addr,
                                 uint64_t val);

#endif
