/**
 * RISC-V Instruction
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
#ifndef _RISCV_INSTRUCTION_H_
#define _RISCV_INSTRUCTION_H_

#include <inttypes.h>
#include "riscv_sim_typedefs.h"

typedef struct RVInstruction
{
    target_ulong pc;

    /* 32-bit or 16-bit instruction */
    uint32_t binary;

    /* Compressed instruction quadrant number */
    uint32_t quad;

    /* Instruction function selectors */
    int32_t major_opcode;
    int32_t funct3;
    int32_t funct4;
    int32_t funct5;
    int32_t funct7;

    /* Decoded register addresses */
    uint32_t rd;
    uint32_t rs1;
    uint32_t rs2;
    uint32_t rs3;

    /* Physical register addresses */
    uint32_t old_pdest;
    uint32_t pdest;
    uint32_t prs1;
    uint32_t prs2;
    uint32_t prs3;

    /* Decoded sign extended immediate values */
    int32_t imm;

    /* Rounding Mode, used by floating Point */
    uint32_t rm;
    
    /* Instruction string */
    char str[RISCV_INS_STR_MAX_LENGTH];
    int create_str;

    int has_src1;
    int has_src2;
    int has_dest;

    int has_fp_dest;
    int has_fp_src1;
    int has_fp_src2;
    int has_fp_src3;
    int f32_mask;
    int f64_mask;
    int set_fs;
    int current_fs;

    int is_atomic;
    int is_atomic_load;
    int is_atomic_store;
    int is_atomic_operate;

    int is_load;
    int is_store;
    int bytes_to_rw;
    int is_unsigned;
    target_ulong mem_addr;

    int is_branch;
    int branch_type;
    int32_t cond;
    target_ulong target;

    int fu_type;
    int is_system;
    int is_func_call;
    int is_func_ret;

    uint64_t buffer;
    uint64_t rs1_val;
    uint64_t rs2_val;
    uint64_t rs3_val;

    int exception;
    int exception_cause;

    /* for updating performance counters */
    int type;
    int data_class;
} RVInstruction;
#endif /* End RVInstruction */
