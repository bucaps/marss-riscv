/**
 * Out of order core
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2017-2020 Gaurav Kothari {gkothar1@binghamton.edu}
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

#include "../utils/circular_queue.h"
#include "../utils/cpu_latches.h"
#include "../utils/sim_params.h"

/* Forward declare */
struct RISCVSIMCPUState;

typedef struct IssueQueueEntry
{
    int valid;
    int ready;
    InstructionLatch *e;
} IssueQueueEntry;

typedef struct ROBEntry
{
    int ready;
    InstructionLatch *e;
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
    InstructionLatch *e;
} LSQEntry;

typedef struct LSQ
{
    CQ cq;
    LSQEntry *entries;
} LSQ;

typedef struct RenameTableEntry
{
    int read_from_rob;
    int rob_idx;
} RenameTableEntry;

typedef struct OOCore
{
    /*----------  Front-end stages  ----------*/
    CPUStage fetch;
    CPUStage decode;
    CPUStage dispatch;

    /*----------  Rename Tables  ----------*/
    RenameTableEntry *int_rat;
    RenameTableEntry *fp_rat;

    ROB rob; /* Reorder buffer */
    LSQ lsq; /* Load-Store Queue */

    /*----------  Issue Queues  ----------*/
    IssueQueueEntry *iq;

    /*----------  Execution units  ----------*/
    CPUStage *ialu;    /* INT ALU */
    CPUStage *imul;    /* INT Multiplier */
    CPUStage *idiv;    /* INT Divider */
    CPUStage *fpu_fma; /* FP Fused Multiply Add */

    CPUStage fpu_alu; /* FP ALU */

    /*----------  Memory Stage  ----------*/
    CPUStage lsu; /* Load-Store Unit unit, works with LSQ */

    /* Dispatch ID for instruction */
    uint64_t ins_dispatch_id; /* Support for speculative execution */

    struct RISCVSIMCPUState *simcpu; /* Pointer to parent */
} OOCore;

/*----------  Out of order core top-level functions  ----------*/
OOCore *oo_core_init(const SimParams *p, struct RISCVSIMCPUState *simcpu);
void oo_core_reset(void *core_type);
void oo_core_free(void *core_type);
int oo_core_run(void *core_type);

/*----------  Out of order stages  ----------*/
int oo_core_rob_commit(OOCore *core);
void oo_core_lsq(OOCore *core);
void oo_core_lsu(OOCore *core);
void oo_core_execute_all(OOCore *core);
void oo_core_issue(OOCore *core);
void oo_core_dispatch(OOCore *core);
void oo_core_decode(OOCore *core);
void oo_core_fetch(OOCore *core);

/*----------  Out of order core utility functions  ----------*/
void oo_process_branch(OOCore *core, InstructionLatch *e);

void iq_reset(IssueQueueEntry *iq_entry, int size);
int iq_full(const IssueQueueEntry *iq, int size);
int iq_get_free_entry(const IssueQueueEntry *iq, int size);
void read_int_operand_from_rob_slot(const OOCore *core, int asrc, int psrc,
                                    int current_rob_idx, uint64_t *buffer,
                                    int *read_flag);
void read_fp_operand_from_rob_slot(const OOCore *core, int asrc, int psrc,
                                   int current_rob_idx, uint64_t *buffer,
                                   int *read_flag);
int rob_entry_committed(const ROB *rob, int src_idx, int current_idx);
#endif
