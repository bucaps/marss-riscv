/**
 * Return address stack
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
#ifndef _RAS_H_
#define _RAS_H_

#include "../riscv_sim_macros.h"
#include "../riscv_sim_typedefs.h"
#include "../utils/sim_params_stats.h"

typedef struct Ras
{
    int sptop;
    int spfill;
    int cur_size;
    int max_size;
    target_ulong empty_reg;
    target_ulong *entry;

    void (*push)(struct Ras *ras, target_ulong pc);
    void (*flush)(struct Ras *ras);
    int (*empty)(struct Ras *ras);
    target_ulong (*pop)(struct Ras *ras);
} Ras;

Ras *ras_init(const SimParams *p);
void ras_free(Ras **ras);
#endif
