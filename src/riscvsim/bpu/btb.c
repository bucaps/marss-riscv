/**
 * Branch Target Buffer
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
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "btb.h"

void
btb_flush(BranchTargetBuffer *b)
{
    int i;

    b->evict_policy->reset(b->evict_policy);
    for (i = 0; i < b->sets; ++i)
    {
        memset((void *)b->data[i], 0, b->ways * sizeof(BtbEntry));
    }
}

/**
 * Returns BPU_HIT if the given pc is present in the BTB and assign address of
 * the BTB entry containing this PC to the out parameter (btb_entry), else
 * return BPU_MISS
 */
int
btb_probe(BranchTargetBuffer *b, target_ulong pc, BtbEntry **btb_entry)
{
    int j;
    int set_addr = GET_SET_ADDR(pc >> 1, b->set_bits);

    *btb_entry = NULL;

    for (j = 0; j < b->ways; ++j)
    {
        if (b->data[set_addr][j].pc == pc)
        {
            /* BTB Hit */
            b->evict_policy->use(b->evict_policy, set_addr, j);
            *btb_entry = &(b->data[set_addr][j]);
            return BPU_HIT;
        }
    }

    /* BTB Miss */
    return BPU_MISS;
}

/**
 * Allocates an entry for the given pc in BTB, after evicting
 * the entry in it's place. This is done from the decode stage
 * of the pipeline.
 */
void
btb_add(BranchTargetBuffer *b, target_ulong pc, int type)
{
    int set_addr = GET_SET_ADDR(pc >> 1, b->set_bits);
    int pos = b->evict_policy->evict(b->evict_policy, set_addr);

    // assert(pos >= 0 && pos < b->ways);
    b->data[set_addr][pos].pc = pc;
    b->data[set_addr][pos].target = 0;
    b->data[set_addr][pos].type = type;
}

/**
 * Updates entry for the given pc in BTB with the target address and
 * the branch outcome. This is done from the memory stage of the pipeline,
 * where the branches are resolved.
 */
void
btb_update(BtbEntry *btb_entry, target_ulong target, int type)
{
    btb_entry->target = target;
    btb_entry->type = type;
}

BranchTargetBuffer *
btb_init(const SimParams *p)
{
    int i;
    BranchTargetBuffer *b;

    b = (BranchTargetBuffer *)calloc(1, sizeof(BranchTargetBuffer));
    assert(b);
    b->size = p->btb_size;
    b->sets = b->size / p->btb_ways;
    b->ways = p->btb_ways;
    b->set_bits = GET_NUM_BITS(b->sets);
    b->evict_policy
        = evict_policy_create(b->sets, b->ways, p->btb_eviction_policy);
    b->data = (BtbEntry **)calloc(b->sets, sizeof(BtbEntry *));
    assert(b->data);
    for (i = 0; i < b->sets; ++i)
    {
        b->data[i] = (BtbEntry *)calloc(b->ways, sizeof(BtbEntry));
        assert(b->data[i]);
    }
    return b;
}

void
btb_free(BranchTargetBuffer **b)
{
    int i;

    for (i = 0; i < (*b)->sets; ++i)
    {
        free((*b)->data[i]);
        (*b)->data[i] = NULL;
    }
    free((*b)->data);
    (*b)->data = NULL;
    evict_policy_free(&(*b)->evict_policy);
    free(*b);
    *b = NULL;
}