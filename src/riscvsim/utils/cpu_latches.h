/**
 * Set of data structures such as latches used by simulated core pipeline
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
#ifndef _CPU_LATCHES_H_
#define _CPU_LATCHES_H_

#include "../bpu/bpu.h"
#include "../decoder/riscv_instruction.h"

/* Instruction latch acts as a place holder for single instruction and keeps
 * complete information concerning it. This information is updated as the
 * instruction passes through the pipeline. This information includes status
 * flags, PC, decoded fields, latency in CPU cycles, operands values, the result
 * produced, memory address, branch status, exception status, etc. */
typedef struct InstructionLatch
{
    int status;
    int insn_latch_index;
    int is_decoded;
    struct RVInstruction ins;
    int max_clock_cycles;
    int elasped_clock_cycles;
    int cache_lookup_complete_signal_sent;

    int read_rs1;
    int read_rs2;
    int read_rs3;
    int keep_dest_busy;
    int result_ready;

    int renamed;
    int rob_idx;
    int iq_idx;
    int lsq_idx;

    int branch_processed;
    int mispredict;
    int is_branch_taken;
    int is_pred_correct;
    target_ulong branch_target;
    target_ulong predicted_target;
    BPUResponsePkt bpu_resp_pkt;

    uint64_t ins_dispatch_id;
} InstructionLatch;

typedef struct CPUStage
{
    int has_data;
    int stage_exec_done;
    int insn_latch_index;
} CPUStage;

void cpu_stage_flush(CPUStage *stage);
void cpu_stage_flush_pipe(CPUStage *stage, int num_stages);
void cpu_stage_flush_free_insn_latch(CPUStage *stage,
                                     InstructionLatch *insn_latch_pool);

InstructionLatch *insn_latch_allocate(InstructionLatch *insn_latch_pool);
InstructionLatch *get_insn_latch(InstructionLatch *insn_latch_pool,
                                 int index);
void reset_insn_latch_pool(InstructionLatch *e);
#endif
