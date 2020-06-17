/**
 * Adaptive Predictor
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
#ifndef _ADAPTIVE_PREDICTOR_H_
#define _ADAPTIVE_PREDICTOR_H_

#include "sim_params_stats.h"
#include "riscv_sim_typedefs.h"

/* Adaptive Predictor Level 1: Global History Table (GHT) Entry */
typedef struct GHTEntry
{
    target_ulong pc; /* Virtual address of the branch */
    uint32_t ghr;    /* History register */
} GHTEntry;

/* Adaptive Predictor Level 2: Pattern History Table (PHT) Entry */
typedef struct PHTEntry
{
    target_ulong pc; /* Virtual address of the branch */
    int *ctr;        /* Array of 2-bit saturating counters */
} PHTEntry;

typedef struct AdaptivePredictor
{
    GHTEntry *ght;           /* GHT */
    PHTEntry *pht;           /* PHT */
    int ght_size;            /* Number of entries in GHT */
    int pht_size;            /* Number of entries in PHT */
    uint32_t ght_index_bits; /* Number of lowest bits of PC required to index into GHT */
    uint32_t pht_index_bits; /* Number of lowest bits of PC required to index into PHT */
    uint32_t hreg_bits;      /* Number of bits in history register present in GHT entry */
    int type;                /* Type of adaptive predictor scheme used (GAg, GAp, PAg, PAp),
                                based on ght_size and pht_size*/
    uint32_t (*pfn_ap_aliasing_func)(struct AdaptivePredictor *a, uint32_t hr,
                                     target_ulong pc); /* Type of aliasing function (and, xor or none) to apply for GAg based schemes */

    int (*probe)(struct AdaptivePredictor *a, target_ulong pc);
    int (*get_prediction)(struct AdaptivePredictor *a, target_ulong pc);
    void (*add)(struct AdaptivePredictor *a, target_ulong pc);
    void (*update)(struct AdaptivePredictor *a, target_ulong pc, int pred);
    void (*flush)(struct AdaptivePredictor *a);
} AdaptivePredictor;

AdaptivePredictor * adaptive_predictor_init(const SimParams *p);
void adaptive_predictor_free(AdaptivePredictor **a);
#endif
