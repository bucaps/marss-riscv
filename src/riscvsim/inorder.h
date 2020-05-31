/**
 * In-order core
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
#ifndef _INORDER_H_
#define _INORDER_H_

#include "sim_params_stats.h"
#include "bpu.h"
#include "circular_queue.h"
#include "common_core_utils.h"

/* Forward declare */
struct RISCVSIMCPUState;

typedef struct InsDispatchQueue
{
    CQ cq;
    uint64_t data[INCORE_NUM_INS_DISPATCH_QUEUE_ENTRY];
} InsDispatchQueue;

typedef struct INCore
{
    /*----------  In-order pipeline stages   ----------*/
    CPUStage pcgen;
    CPUStage fetch;
    CPUStage decode;
    CPUStage memory;
    CPUStage commit;

    /*----------  Register valid bits  ----------*/
    uint32_t int_reg_status[NUM_INT_REG];
    uint32_t fp_reg_status[NUM_FP_REG];

    /*----------  Forwarding buses  ----------*/
    DataFWDLatch fwd_latch[NUM_FWD_BUS];

    /*----------  EX to Mem Queue  ----------*/
    InsDispatchQueue ins_dispatch_queue;

    /*----------  Execution units  ----------*/
    CPUStage *ialu;
    CPUStage *imul;
    CPUStage *idiv;
    CPUStage *fpu_fma;
    CPUStage fpu_alu;

    /*----------  Pointer to 5 or 6 stage run() function  ----------*/
    int (*pfn_incore_run_internal)(struct INCore *core);

    /*----------  Unique ID for every instruction   ----------*/
    /* As every instruction is issued from decode stage, it is
     * assigned a unique ID. This ID is used by memory stage to select earliest
     * instruction in sequence, when multiple functional units try to send their
     * respective instructions to memory stage */
    uint64_t ins_dispatch_id;

    struct RISCVSIMCPUState *simcpu; /* Pointer to parent */
} INCore;

/*---------- In-order core top-level functions  ----------*/
INCore *in_core_init(const SimParams *p, struct RISCVSIMCPUState *simcpu);
void in_core_reset(void *core_type);
void in_core_free(void *core_type);
int in_core_run(void *core_type);

/*----------  In-order core stages  ----------*/
void in_core_pcgen(INCore *core);
void in_core_fetch(INCore *core);
void in_core_decode(INCore *core);
void in_core_execute_all(INCore *core);
void in_core_memory(INCore *core);
int in_core_commit(INCore *core);
int in_core_run_5_stage(INCore *core);
int in_core_run_6_stage(INCore *core);
#endif
