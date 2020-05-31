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

#include "bpu.h"
#include "riscv_instruction.h"
#include "riscv_sim_typedefs.h"

struct RISCVCPUState;

typedef struct DataFWDLatch
{
    uint64_t buffer;
    int rd;
    int valid;
    int fp_dest;
    int int_dest;
} DataFWDLatch;

typedef struct InstructionMapEntry
{
    int status;
    uint64_t ins_dispatch_id;
    uint32_t is_decoded;
    uint32_t is_branch_taken;
    target_ulong branch_target;
    uint32_t bpu_probe;
    target_ulong predicted_target;
    uint32_t is_pred_correct;
    int max_latency;
    int current_latency;
    int data_fwd_done;
    int read_rs1;
    int read_rs2;
    int read_rs3;
    int renamed;
    int rob_idx;
    int iq_idx;
    int lsq_idx;
    int imap_index;
    int branch_processed;
    int stop_flush;
    int mispredict;
    int keep_dest_busy;
    struct RVInstruction ins;
    BPUResponsePkt bpu_resp_pkt;
} IMapEntry;

typedef struct CPUStage
{
    uint32_t has_data;
    int imap_index;
    uint32_t stage_exec_done;
} CPUStage;

typedef struct SimTracePacket
{
    uint32_t cycle;
    IMapEntry *e;
} SimTracePacket;

void cpu_stage_flush(CPUStage *stage);
void exec_unit_flush(CPUStage *stage, int num_stages);
void speculative_cpu_stage_flush(CPUStage *stage, IMapEntry *imap);
void speculative_exec_unit_flush(CPUStage *stage, int num_stages,
                                 IMapEntry *imap);

IMapEntry *allocate_imap_entry(IMapEntry *imap);
void reset_imap(IMapEntry *e);
IMapEntry *get_imap_entry(IMapEntry *imap, int index);

int code_tlb_access_and_ins_fetch(struct RISCVCPUState *s, IMapEntry *e);
void do_fetch_stage_exec(struct RISCVCPUState *s, IMapEntry *e);

void do_decode_stage_exec(struct RISCVCPUState *s, IMapEntry *e);

void set_exception_state(struct RISCVCPUState *s, const IMapEntry *e);
void set_timer_exception_state(struct RISCVCPUState *s, const IMapEntry *e);
int execute_load_store(struct RISCVCPUState *s, IMapEntry *e);
int get_data_mem_access_latency(struct RISCVCPUState *s, IMapEntry *e);

void handle_bpu_frontend_probe(struct RISCVCPUState *s, IMapEntry *e);
void handle_no_bpu_frontend_probe(struct RISCVCPUState *s, IMapEntry *e);
int handle_branch_with_bpu(struct RISCVCPUState *s, IMapEntry *e);
int handle_branch_no_bpu(struct RISCVCPUState *s, IMapEntry *e);

int handle_branch_decode_with_bpu(struct RISCVCPUState *s, IMapEntry *e);
int handle_branch_decode_no_bpu(struct RISCVCPUState *s, IMapEntry *e);

void copy_cache_stats_to_global_stats(struct RISCVCPUState *s);
void sim_print_ins_trace(struct RISCVCPUState *s);
void sim_print_exp_trace(struct RISCVCPUState *s);

void update_arch_reg_int(struct RISCVCPUState *s, IMapEntry *e);
void update_arch_reg_fp(struct RISCVCPUState *s, IMapEntry *e);
void update_insn_commit_stats(struct RISCVCPUState *s, IMapEntry *e);
void setup_sim_trace_pkt(struct RISCVCPUState *s, IMapEntry *e);
void write_stats_to_stats_display_shm(struct RISCVCPUState *s);
int set_max_latency_for_non_pipe_fu(struct RISCVCPUState *s, int fu_type,
                                    IMapEntry *e);
#endif
