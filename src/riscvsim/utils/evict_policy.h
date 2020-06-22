/**
 * Eviction policy wrapper for set associative data structures
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
#ifndef _EVICT_POLICY_H_
#define _EVICT_POLICY_H_

#include <inttypes.h>

typedef struct EvictPolicy
{
    int type;
    int num_sets;
    int num_ways;

    /* MRU bit map for all the ways in a set */
    uint64_t *sets;

    /* This pointers are set according to eviction policy used */
    void (*reset)(struct EvictPolicy *p);
    void (*use)(struct EvictPolicy *p, int set, int way);
    int (*evict)(struct EvictPolicy *p, int set);
} EvictPolicy;

EvictPolicy *evict_policy_create(int sets, int ways, int policy_type);
void evict_policy_free(EvictPolicy **);
#endif /* _EVICT_POLICY_H_ */
