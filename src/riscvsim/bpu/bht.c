/**
 * Branch History Table
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2020 Gaurav Kothari {gkothar1@binghamton.edu}
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

#include "../riscv_sim_macros.h"
#include "bht.h"

static void
update_two_bit_counter(int *ctr, int pred)
{
    if (pred)
    {
        if ((*ctr >= 0) && (*ctr < 3))
        {
            (*ctr)++;
        }
    }
    else
    {
        if ((*ctr >= 1) && (*ctr <= 3))
        {
            (*ctr)--;
        }
    }
}

int
bht_get_prediction(const Bht *b, target_ulong pc)
{
    int idx;

    idx = GET_INDEX(pc, b->bht_index_bits);
    return b->bht_entry[idx].pred;
}

void
bht_add(Bht *b, target_ulong pc)
{
    int idx;

    /* In case of bimodal predictor, set the 2-bit saturating counter to 1
     * (Not taken) */
    idx = GET_INDEX(pc, b->bht_index_bits);
    b->bht_entry[idx].pred = 1;
}

void
bht_update(Bht *b, target_ulong pc, int pred)
{
    int idx;

    idx = GET_INDEX(pc, b->bht_index_bits);
    update_two_bit_counter(&b->bht_entry[idx].pred, pred);
}

void
bht_flush(Bht *b)
{
    int i;

    for (i = 0; i < b->bht_size; ++i)
    {
        b->bht_entry[i].pred = 1;
    }
}

Bht *
bht_init(const SimParams *p)
{
    Bht *b;

    b = calloc(1, sizeof(Bht));
    assert(b);

    b->bht_size = p->bht_size;
    b->bht_entry = calloc(b->bht_size, sizeof(BhtEntry));
    assert(b->bht_entry);

    b->bht_index_bits = GET_NUM_BITS(b->bht_size);
    return b;
}

void
bht_free(Bht **b)
{
    free((*b)->bht_entry);
    (*b)->bht_entry = NULL;
}
