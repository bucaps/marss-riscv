/**
 * In-order core
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
#ifndef _INORDER_H_
#define _INORDER_H_

#include "../bpu/bpu.h"
#include "../utils/circular_queue.h"
#include "../utils/cpu_latches.h"
#include "../utils/sim_params.h"

/* Forward declare */
struct RISCVSIMCPUState;

/* Every instruction issued from the decode stage is assigned a unique
 * sequential ID known as ins_dispatch_id. This ID is appended to
 * ex_to_mem_queue. This ID (returned from the ex_to_mem_queue top) is used by
 * memory stage to select the earliest instruction in sequence when multiple
 * functional units (if parallel FUs enabled) try to send their respective
 * instructions to the memory stage in the same cycle  */
typedef struct ExToMemQueue
{
    CQ cq;
    uint64_t data[INCORE_EX_TO_MEM_QUEUE_SIZE];
} ExToMemQueue;

typedef struct DataFWDLatch
{
    uint64_t buffer;
    int rd;
    int valid;
    int fp_dest;
    int int_dest;
} DataFWDLatch;

typedef struct INCore
{
    /*----------  In-order pipeline stages   ----------*/
    CPUStage fetch;
    CPUStage decode;
    CPUStage memory1;
    CPUStage memory2;
    CPUStage commit;

    /*----------  Register valid bits  ----------*/
    uint32_t int_reg_status[NUM_INT_REG];
    uint32_t fp_reg_status[NUM_FP_REG];

    /*----------  Forwarding buses  ----------*/
    DataFWDLatch fwd_latch[NUM_FWD_BUS];

    /*----------  EX to Mem Queue  ----------*/
    ExToMemQueue ex_to_mem_queue;

    /*----------  Execution units/EX stage ----------*/
    CPUStage *ialu;
    CPUStage *imul;
    CPUStage *idiv;
    CPUStage *fpu_fma;
    CPUStage fpu_alu;

    /*----------  Pointer to 5 or 6 stage run() function  ----------*/
    int (*pfn_incore_run_internal)(struct INCore *core);

    uint64_t ins_dispatch_id;

    struct RISCVSIMCPUState *simcpu; /* Pointer to parent */
} INCore;

/*---------- In-order core top-level functions  ----------*/
INCore *in_core_init(const SimParams *p, struct RISCVSIMCPUState *simcpu);
void in_core_reset(void *core_type);
void in_core_free(void *core_type);
int in_core_run(void *core_type);

/*----------  In-order core stages  ----------*/
void in_core_fetch(INCore *core);
void in_core_decode(INCore *core);
void in_core_execute_all(INCore *core);
void in_core_memory1(INCore *core);
void in_core_memory2(INCore *core);
int in_core_commit(INCore *core);
int in_core_run_5_stage(INCore *core);
int in_core_run_6_stage(INCore *core);
#endif
