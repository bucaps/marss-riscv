/**
 * Global typedefs and macros used by MARSS-RISCV
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
#ifndef _RISCV_SIM_MACROS_H_
#define _RISCV_SIM_MACROS_H_

/* Type of Functional Units */
#define FU_ALU 0x0
#define FU_MUL 0x1
#define FU_DIV 0x2
#define FU_FPU_ALU 0x3
#define FU_FPU_FMA 0x4
#define NUM_MAX_FU 5

/* Type of Branch instructions */
#define BRANCH_UNCOND 0x0
#define BRANCH_COND 0x1
#define BRANCH_FUNC_CALL 0x2
#define BRANCH_FUNC_RET 0x3

/* Extension C Quadrants */
#define C_QUADRANT0 0
#define C_QUADRANT1 1
#define C_QUADRANT2 2

/* Major Opcodes */
#define OP_IMM_MASK 0x13
#define OP_IMM_32_MASK 0x1b
#define OP_MASK 0x33
#define OP_MASK_32 0x3b
#define LUI_MASK 0x37
#define AUIPC_MASK 0x17
#define JAL_MASK 0x6f
#define JALR_MASK 0x67
#define BRANCH_MASK 0x63
#define LOAD_MASK 0x3
#define STORE_MASK 0x23
#define FENCE_MASK 0xf
#define CSR_MASK 0x73
#define ATOMIC_MASK 0x2F

/* Floating Point Instructions */
#define FLOAD_MASK 0x07
#define FSTORE_MASK 0x27
#define FMADD_MASK 0x43
#define FMSUB_MASK 0x47
#define FNMSUB_MASK 0x4B
#define FNMADD_MASK 0x4F
#define F_ARITHMETIC_MASK 0x53

/* Used as stage IDs for in-order pipeline */
#define PCGEN 0x0
#define FETCH 0x1
#define DECODE 0x2
#define MEMORY 0x3
#define COMMIT 0x4

#define NUM_CPU_STAGES 5 /* Excluding the functional units which are allocated separately */
#define NUM_INT_REG 32
#define NUM_FP_REG 32
#define NUM_FU 5 /* ALU, MUL, DIV, FP ALU, FP FMA */
#define NUM_FWD_BUS 6 /* Total 6 forwarding buses, 5 for functional units  and 1 for memory stage */
#define INCORE_EX_TO_MEM_QUEUE_SIZE 16
#define SPEC_REG_STATE_ENTRY 128

#define RISCV_INS_STR_MAX_LENGTH 64

/* NOTE: Max size of instruction latch pool, must always be greater than ROB
 * size */
#define INSN_LATCH_POOL_SIZE 128
#define INSN_LATCH_FREE 0x0
#define INSN_LATCH_ALLOCATED 0x1

/* Used to check pipeline drain status in case of exception inside simulator */
#define PIPELINE_NOT_DRAINED 0
#define PIPELINE_DRAINED 1

#define NUM_MAX_PRV_LEVELS 4

/* Used for updating performance counters */

#define NUM_MAX_INS_TYPES 17
#define INS_TYPE_LOAD 0x0
#define INS_TYPE_STORE 0x1
#define INS_TYPE_ATOMIC 0x2
#define INS_TYPE_SYSTEM 0x3
#define INS_TYPE_ARITMETIC 0x4
#define INS_TYPE_COND_BRANCH 0x5
#define INS_TYPE_JAL 0x6
#define INS_TYPE_JALR 0x7
#define INS_TYPE_INT_MUL 0x8
#define INS_TYPE_INT_DIV 0x9
#define INS_TYPE_FP_LOAD 0xa
#define INS_TYPE_FP_STORE 0xb
#define INS_TYPE_FP_ADD 0xc
#define INS_TYPE_FP_MUL 0xd
#define INS_TYPE_FP_FMA 0xe
#define INS_TYPE_FP_DIV_SQRT 0xf
#define INS_TYPE_FP_MISC 0x10

#define INS_CLASS_INT 0x11
#define INS_CLASS_FP 0x12

/* For Branch prediction unit */
#define BPU_MISS 0x0
#define BPU_HIT 0x1

#define GET_NUM_BITS(x) ceil(log2((x)))
#define GET_INDEX(x, bits) ((x) & ((1 << (bits)) - 1))
#define GET_SET_ADDR(pc, bits) (GET_INDEX((pc), (bits)))
#define PRINT_INIT_MSG(str) fprintf(stderr, " \x1B[32m*\x1B[0m " str "...\n")
#define PRINT_PROG_TITLE_MSG(str) fprintf(stderr, "\x1B[32m\x1B[0m " str "\n\n")

/* For exception handling during simulation */
#define SIM_ILLEGAL_OPCODE_EXCEPTION 0x1
#define SIM_COMPLEX_OPCODE_EXCEPTION 0x2
#define SIM_TEMU_TIMEOUT_EXCEPTION 0x3
#define SIM_MMU_EXCEPTION 0x4
#define SIM_ICOUNT_COMPLETE_EXCEPTION 0x5

/* Instruction types processed by FPU ALU. This is used to set the latencies. */
#define FU_FPU_ALU_FADD 0x0
#define FU_FPU_ALU_FSUB 0x1
#define FU_FPU_ALU_FMUL 0x2
#define FU_FPU_ALU_FDIV 0x3
#define FU_FPU_ALU_FSQRT 0x4
#define FU_FPU_ALU_FSGNJ 0x5
#define FU_FPU_ALU_FMIN 0x6
#define FU_FPU_ALU_FMAX 0x7
#define FU_FPU_ALU_FEQ 0x8
#define FU_FPU_ALU_FLT 0x9
#define FU_FPU_ALU_FLE 0xa
#define FU_FPU_ALU_FCVT 0xb
#define FU_FPU_ALU_CVT 0xc
#define FU_FPU_ALU_FMV 0xd
#define FU_FPU_ALU_FCLASS 0xe
#define MAX_FU_FPU_ALU_TYPES 15

/* Eviction policies for set associative data structures */
#define EVICT_POLICY_RANDOM 0x0
#define EVICT_POLICY_BIT_PLRU 0x1

#endif
