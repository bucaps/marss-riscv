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
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "btb.h"

#define GET_SET_ADDR(pc, bits) (GET_INDEX((pc), (bits)))

/**
 * Returns any random way index between 0 to (b->ways - 1)
 * inclusive, for the given set.
 */
static int
btb_evict_policy_random(BranchTargetBuffer *b, int set)
{
    return rand() % b->ways;
}

/**
 * Returns least used way index between 0 to (b->ways - 1)
 * inclusive, for the given set.
 */
static int
btb_evict_policy_lru(BranchTargetBuffer *b, int set)
{
    int check_bit = 0;
    int lower_bound = 0;
    int upper_bound = b->ways - 1;

    while (check_bit < (b->ways - 1))
    {
        if (b->status_bits[set][check_bit])
        {
            check_bit = ((check_bit + 1) * 2) - 1;
            upper_bound = floor((lower_bound + upper_bound) / 2);
        }
        else
        {
            check_bit = ((check_bit + 1) * 2);
            lower_bound = floor((lower_bound + upper_bound) / 2) + 1;
        }
    }

    /* NOTE: This assert can be removed */
    assert(lower_bound == upper_bound);
    return lower_bound;
}

/**
 * Update the status bits for the given set and ways,
 * based on btb_probe. Based on status bits, least used way
 * in a given set is selected for eviction later on.
 */
static void
btb_update_lru_status_bits(BranchTargetBuffer *b, int set, int way)
{
    int update_bit = 0;
    int comp = floor(b->ways / 2);

    while (update_bit < (b->ways - 1))
    {
        if (way <= comp)
        {
            /* status bit is in the first half */
            b->status_bits[set][update_bit] = 0;
            update_bit = ((update_bit + 1) * 2) - 1;
            comp = floor(comp / 2);
        }
        else
        {
            /* status bit is in the second half */
            b->status_bits[set][update_bit] = 1;
            update_bit = ((update_bit + 1) * 2);
            comp = floor((comp + b->ways - 1) / 2);
        }
    }
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
    b->pfn_btb_evict_handler = NULL;
    if (p->btb_eviction_policy == BTB_RANDOM_EVICT)
    {
        b->rand_evict = 1;
        b->pfn_btb_evict_handler = &btb_evict_policy_random;
    }
    else
    {
        b->rand_evict = 0;
        b->pfn_btb_evict_handler = &btb_evict_policy_lru;
    }

    b->data = (BtbEntry **)calloc(b->sets, sizeof(BtbEntry *));
    b->status_bits = (int **)calloc(b->sets, sizeof(int *));
    assert(b->data);
    assert(b->status_bits);
    for (i = 0; i < b->sets; ++i)
    {
        b->data[i] = (BtbEntry *)calloc(b->ways, sizeof(BtbEntry));
        assert(b->data[i]);
        b->status_bits[i] = (int *)calloc(b->ways, sizeof(int));
        assert(b->status_bits[i]);
    }

    return b;
}

void
btb_free(BranchTargetBuffer **b)
{
    int i;

    for (i = 0; i < (*b)->sets; ++i)
    {
        free((*b)->status_bits[i]);
        (*b)->status_bits[i] = NULL;
        free((*b)->data[i]);
        (*b)->data[i] = NULL;
    }
    free((*b)->status_bits);
    (*b)->status_bits = NULL;
    free((*b)->data);
    (*b)->data = NULL;
    free(*b);
    *b = NULL;
}

void
btb_flush(BranchTargetBuffer *b)
{
    int i;

    for (i = 0; i < b->sets; ++i)
    {
        memset((void *)b->data[i], 0, b->ways * sizeof(BtbEntry));
        memset((void *)b->status_bits[i], 0, b->ways * sizeof(int));
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
            /* If LRU eviction policy is used, update status bits for this entry */
            if (!b->rand_evict)
            {
                btb_update_lru_status_bits(b, set_addr, j);
            }
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
    int pos = b->pfn_btb_evict_handler(b, set_addr);

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
btb_update(BtbEntry *btb_entry, target_ulong target,int type)
{
    btb_entry->target = target;
    btb_entry->type = type;
}