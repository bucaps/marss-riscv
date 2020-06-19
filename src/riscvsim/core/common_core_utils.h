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
#ifndef _COMMON_CORE_UTILS_H_
#define _COMMON_CORE_UTILS_H_

#include "../bpu/bpu.h"
#include "../decoder/riscv_instruction.h"
#include "../riscv_sim_typedefs.h"

struct RISCVCPUState;
 
typedef struct InstructionLatch
{
    int status;
    int insn_latch_index;
    int is_decoded;
    struct RVInstruction ins;
    int max_clock_cycles;
    int elasped_clock_cycles;

    int data_fwd_done;
    int read_rs1;
    int read_rs2;
    int read_rs3;
    int keep_dest_busy;

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
    uint32_t has_data;
    int insn_latch_index;
    uint32_t stage_exec_done;
} CPUStage;

typedef struct SimTracePacket
{
    uint32_t cycle;
    InstructionLatch *e;
} SimTracePacket;

void cpu_stage_flush(CPUStage *stage);
void exec_unit_flush(CPUStage *stage, int num_stages);
void speculative_cpu_stage_flush(CPUStage *stage, InstructionLatch *insn_latch_pool);
void speculative_exec_unit_flush(CPUStage *stage, int num_stages,
                                 InstructionLatch *insn_latch_pool);

InstructionLatch *insn_latch_allocate(InstructionLatch *insn_latch_pool);
void reset_insn_latch_pool(InstructionLatch *e);
InstructionLatch *get_insn_latch(InstructionLatch *insn_latch_pool, int index);

int code_tlb_access_and_ins_fetch(struct RISCVCPUState *s, InstructionLatch *e);
void do_fetch_stage_exec(struct RISCVCPUState *s, InstructionLatch *e);

void do_decode_stage_exec(struct RISCVCPUState *s, InstructionLatch *e);

void set_exception_state(struct RISCVCPUState *s, const InstructionLatch *e);
void set_timer_exception_state(struct RISCVCPUState *s, const InstructionLatch *e);
int execute_load_store(struct RISCVCPUState *s, InstructionLatch *e);
int get_data_mem_access_latency(struct RISCVCPUState *s, InstructionLatch *e);

void handle_bpu_frontend_probe(struct RISCVCPUState *s, InstructionLatch *e);
void handle_no_bpu_frontend_probe(struct RISCVCPUState *s, InstructionLatch *e);
int handle_branch_with_bpu(struct RISCVCPUState *s, InstructionLatch *e);
int handle_branch_no_bpu(struct RISCVCPUState *s, InstructionLatch *e);

int handle_branch_decode_with_bpu(struct RISCVCPUState *s, InstructionLatch *e);
int handle_branch_decode_no_bpu(struct RISCVCPUState *s, InstructionLatch *e);

void copy_cache_stats_to_global_stats(struct RISCVCPUState *s);
void sim_print_ins_trace(struct RISCVCPUState *s);
void sim_print_exp_trace(struct RISCVCPUState *s);

void update_arch_reg_int(struct RISCVCPUState *s, InstructionLatch *e);
void update_arch_reg_fp(struct RISCVCPUState *s, InstructionLatch *e);
void update_insn_commit_stats(struct RISCVCPUState *s, InstructionLatch *e);
void setup_sim_trace_pkt(struct RISCVCPUState *s, InstructionLatch *e);
void write_stats_to_stats_display_shm(struct RISCVCPUState *s);
int set_max_clock_cycles_for_non_pipe_fu(struct RISCVCPUState *s, int fu_type,
                                    InstructionLatch *e);
#endif
