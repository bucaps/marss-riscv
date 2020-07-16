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
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "../riscv_sim_macros.h"
#include "evict_policy.h"

static void
bit_plru_use(EvictPolicy *p, int set, int way)
{
    if (p->num_ways == 1)
    {
        return;
    }

    /* Set MRU bit for this way in this set to 1 */
    p->sets[set] |= (1 << way);

    /* Whenever the last remaining 0 bit of a set's status bits is set to 1, all
     * other bits are reset to 0. */
    if (p->sets[set] == ((1 << p->num_ways) - 1))
    {
        p->sets[set] = 0;
        p->sets[set] |= (1 << way);
    }
}

static int
bit_plru_evict(EvictPolicy *p, int set)
{
    int i = 0;
    uint64_t current_set = p->sets[set];

    if (p->num_ways == 1)
    {
        return 0;
    }

    /* Return the first way address whose MRU bit is set to 0 */
    for (i = 0; i < p->num_ways; i++)
    {
        if ((current_set & 1) == 0)
        {
            return i;
        }
        current_set >>= 1;
    }

    assert(0);
}

static void
random_use(EvictPolicy *p, int set, int way)
{
    return;
}

static int
random_evict(EvictPolicy *p, int set)
{
    return rand() % p->num_ways;
}

static void
evict_policy_reset(EvictPolicy *p)
{
    memset((void *)p->sets, 0, p->num_sets * sizeof(uint64_t));
}

EvictPolicy *
evict_policy_create(int sets, int ways, int policy_type)
{
    EvictPolicy *p;

    p = calloc(1, sizeof(EvictPolicy));
    assert(p);

    p->num_sets = sets;
    p->num_ways = ways;
    p->type = policy_type;

    p->sets = calloc(p->num_sets, sizeof(uint64_t));
    assert(p->sets);

    switch (p->type)
    {
        case EVICT_POLICY_RANDOM:
        {
            p->use = &random_use;
            p->evict = &random_evict;
            break;
        }
        case EVICT_POLICY_BIT_PLRU:
        {
            p->use = &bit_plru_use;
            p->evict = &bit_plru_evict;
            break;
        }
        default:
        {
            break;
        }
    }

    p->reset = &evict_policy_reset;
    return p;
}

void
evict_policy_free(EvictPolicy **p)
{
    free((*p)->sets);
    free(*p);
}
