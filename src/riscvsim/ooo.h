/**
 * Out of order core
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2017-2019 Gaurav Kothari {gkothar1@binghamton.edu}
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
#ifndef _OOO_H_
#define _OOO_H_

#include "../sim_params_stats.h"
#include "common_core_utils.h"
#include "circular_queue.h"

/* Forward declare */
struct RISCVSIMCPUState;

typedef struct PhysicalRegFileEntry
{
    int valid;
    int busy;
    uint64_t val;
} PRFEntry;

typedef struct IssueQueueEntry
{
    int valid;
    int ready;
    IMapEntry *e;
} IssueQueueEntry;

typedef struct WbQueueEntry
{
    int valid;
    IMapEntry *e;
} WbQueueEntry;

typedef struct WbQueue
{
    CQ cq;
    WbQueueEntry *entries;
} WbQueue;

typedef struct ROBEntry
{
    int ready;
    IMapEntry *e;
} ROBEntry;

typedef struct ReOrderBuffer
{
    CQ cq;
    ROBEntry *entries;
} ROB;

typedef struct LSQEntry
{
    int ready;
    int mem_request_sent;
    int mem_request_complete;
    IMapEntry *e;
} LSQEntry;

typedef struct LSQ
{
    CQ cq;
    LSQEntry *entries;
} LSQ;

/* Simple int circular queue */
typedef struct CircularQueueInt
{
    CQ cq;
    int *entries;
} CQInt;

/**

    TODO:
    - Branch index stack (BIS) based speculative execution rollback will be
      implemented in future

 */
typedef struct BranchIndexStackEntry
{
    IMapEntry *e;
} BISEntry;

typedef struct BranchIndexStack
{
    CQ cq;
    BISEntry *entries;
} BIS;

typedef struct OOCore
{
    /*----------  Front-end stages  ----------*/
    CPUStage fetch;
    CPUStage decode;
    CPUStage dispatch;

    /*----------  Forwarding buses  ----------*/
    DataFWDLatch fwd_latch[NUM_FWD_BUS];

    /*----------  Rename Tables  ----------*/
    int spec_rat_int[NUM_INT_REG];      /* Speculative rename table for INT register file */
    int spec_rat_fp[NUM_FP_REG];        /* Speculative rename table for FP register file */
    int commit_rat_int[NUM_INT_REG];    /* Commit rename table for INT register file */
    int commit_rat_fp[NUM_FP_REG];      /* Commit rename table for FP register file */

    /*----------  Physical Register Files  ----------*/
    PRFEntry *prf_int;  /* Physical register file for INT */
    PRFEntry *prf_fp;   /* Physical register file for FP */
    CQInt free_pr_int;  /* Free list of INT physical registers */
    CQInt free_pr_fp;   /* Free list of FP physical registers */

    WbQueue prf_int_wb_queue; /* INT Physical register file write-back queue */
    WbQueue prf_fp_wb_queue;  /* FP Physical register file write-back queue */
    ROB rob;                  /* Reorder buffer */
    LSQ lsq;                  /* Load-Store Queue */
    BIS bis;                  /* Branch Index Stack, For future implementation  */

    /*----------  Issue Queues  ----------*/
    IssueQueueEntry *iq_int; /* Issue Queue for INT instructions */
    IssueQueueEntry *iq_fp;  /* Issue Queue for FP instructions */
    IssueQueueEntry *iq_mem; /* Issue Queue for Memory instructions */

    /*----------  Execution units  ----------*/
    CPUStage *ialu;    /* INT ALU */
    CPUStage *imul;    /* INT Multiplier */
    CPUStage *idiv;    /* INT Divider */
    CPUStage *fpu_alu; /* FP ALU */
    CPUStage *fpu_fma; /* FP Fused Multiply Add */

    /*----------  Memory Stage  ----------*/
    CPUStage lsu; /* Load-Store Unit unit, works with LSQ */

    struct RISCVSIMCPUState *simcpu; /* Pointer to parent */
} OOCore;

/*----------  Out of order core top-level functions  ----------*/
OOCore *oo_core_init(const SimParams *p, struct RISCVSIMCPUState *simcpu);
void oo_core_reset(void *core_type);
void oo_core_free(void *core_type);
int oo_core_run(void *core_type);

/*----------  Out of order stages  ----------*/
int oo_core_rob_commit(OOCore *core);
void oo_core_writeback(OOCore *core);
void oo_core_lsq(OOCore *core);
void oo_core_lsu(OOCore *core);
void oo_core_execute_all(OOCore *core);
void oo_core_issue(OOCore *core);
void oo_core_dispatch(OOCore *core);
void oo_core_decode(OOCore *core);
void oo_core_fetch(OOCore *core);

/*----------  Out of order core utility functions  ----------*/
void oo_process_branch(OOCore *core, IMapEntry *e);

void iq_reset(IssueQueueEntry *iq_entry, int size);
int iq_full(IssueQueueEntry *iq, int size);
int iq_get_free_entry(IssueQueueEntry *iq, int size);

int send_phy_reg_write_request(WbQueue *q, IMapEntry *e);
#endif
