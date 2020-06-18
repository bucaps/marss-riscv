/**
 * Branch Target Buffer
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
#ifndef _BRANCH_TARGET_BUFFER_H_
#define _BRANCH_TARGET_BUFFER_H_

#include "../riscv_sim_typedefs.h"
#include "../utils/evict_policy.h"
#include "../utils/sim_params_stats.h"

typedef struct BtbEntry
{
    target_ulong pc;     /* Virtual address of this branch */
    target_ulong target; /* Target of this branch */
    int type;            /* Type of branch, BRANCH_COND or BRANCH_UNCOND */
} BtbEntry;

typedef struct BranchTargetBuffer
{
    BtbEntry **data;
    int rand_evict;    /* If non-zero, uses random eviction policy for BTB, instead of LRU */
    int size;          /* Number of entries in BTB */
    int sets;          /* Number of BTB sets */
    uint32_t set_bits; /* Number of lowest bits of PC required to index into a set */
    int ways;          /* Number of ways in each set */
    EvictPolicy *evict_policy;

    int (*probe)(struct BranchTargetBuffer *b, target_ulong pc,
                 BtbEntry **btb_entry);
    void (*add)(struct BranchTargetBuffer *b, target_ulong pc, int type);
    void (*update)(struct BtbEntry *btb_entry, target_ulong target, int type);
    void (*flush)(struct BranchTargetBuffer *b);
} BranchTargetBuffer;

BranchTargetBuffer *btb_init(const SimParams *p);
void btb_free(BranchTargetBuffer **b);
#endif
