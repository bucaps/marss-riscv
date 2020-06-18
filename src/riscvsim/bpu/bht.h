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
#ifndef _BHT_H_
#define _BHT_H_

#include "../riscv_sim_typedefs.h"
#include "../utils/sim_params_stats.h"

typedef struct BhtEntry
{
    int pred;
} BhtEntry;

typedef struct Bht
{
    BhtEntry *bht_entry;
    int bht_index_bits;
    int bht_size;

    void (*flush)(struct Bht *b);
    void (*add)(struct Bht *b, target_ulong pc);
    void (*update)(struct Bht *b, target_ulong pc, int pred);
    int (*get_prediction)(struct Bht *b, target_ulong pc);
} Bht;

Bht *bht_init(const SimParams *p);
void bht_free(Bht **b);
#endif
