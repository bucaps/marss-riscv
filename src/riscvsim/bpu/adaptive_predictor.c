/**
 * Adaptive Predictor
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

#include "adaptive_predictor.h"

#define GAG 0x0
#define GAP 0x1
#define PAG 0x2
#define PAP 0x3

#define UPDATE_GHR(ghr, bits, pred) (GET_INDEX((((ghr) << 1) | (pred)), (bits)))

#define PRED_NOT_TAKEN 0x0
#define PRED_TAKEN 0x1

static uint32_t
xor_aliasing_func(const AdaptivePredictor *a, uint32_t hr, target_ulong pc)
{
    return GET_INDEX(hr ^ pc, a->hreg_bits);
}

static uint32_t
and_aliasing_func(const AdaptivePredictor *a, uint32_t hr, target_ulong pc)
{
    return GET_INDEX(hr & pc, a->hreg_bits);
}

static uint32_t
none_aliasing_func(const AdaptivePredictor *a, uint32_t hr, target_ulong pc)
{
    return GET_INDEX(hr, a->hreg_bits);
}

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

/* Returns BPU_HIT if given pc is present in GHT */
static int
adaptive_predictor_ght_probe(const AdaptivePredictor *a, target_ulong pc)
{
    int index = GET_INDEX(pc >> 1, a->ght_index_bits);

    if (a->ght[index].pc == pc)
    {
        return BPU_HIT;
    }

    return BPU_MISS;
}

/* Returns BPU_HIT if given pc is present in PHT */
static int
adaptive_predictor_pht_probe(const AdaptivePredictor *a, target_ulong pc)
{
    int index = GET_INDEX(pc >> 1, a->pht_index_bits);

    if (a->pht[index].pc == pc)
    {
        return BPU_HIT;
    }

    return BPU_MISS;
}

/**
 * Based on the adaptive predictor scheme, probe either GHT
 * or PHT or both.
 */
int
adaptive_predictor_probe(const AdaptivePredictor *a, target_ulong pc)
{
    switch (a->type)
    {
        case GAG:
        {
            return BPU_HIT;
        }
        case GAP:
        {
            return adaptive_predictor_pht_probe(a, pc);
        }
        case PAG:
        {
            return adaptive_predictor_ght_probe(a, pc);
        }
        case PAP:
        {
            return (adaptive_predictor_ght_probe(a, pc)
                    && adaptive_predictor_pht_probe(a, pc));
        }
    }

    assert(0);
    return 0;
}

/**
 * Based on the adaptive predictor scheme, return the
 * prediction.
 */
int
adaptive_predictor_get_prediction(const AdaptivePredictor *a, target_ulong pc)
{
    int l1_index;
    int l2_index;
    uint32_t ghr;

    switch (a->type)
    {
        case GAG:
        {
            ghr = GET_INDEX(a->ght[0].ghr, a->hreg_bits);

            /* Apply aliasing function for Gshare and Gselect */
            ghr = a->pfn_ap_aliasing_func(a, ghr, pc);
            if (a->pht[0].ctr[ghr] > 1)
            {
                return 1;
            }
            return 0;
        }
        case GAP:
        {
            ghr = GET_INDEX(a->ght[0].ghr, a->hreg_bits);
            l2_index = GET_INDEX(pc >> 1, a->pht_index_bits);
            if (a->pht[l2_index].ctr[ghr] > 1)
            {
                return 1;
            }
            return 0;
        }
        case PAG:
        {
            l1_index = GET_INDEX(pc >> 1, a->ght_index_bits);
            ghr = GET_INDEX(a->ght[l1_index].ghr, a->hreg_bits);
            if (a->pht[0].ctr[ghr] > 1)
            {
                return 1;
            }
            return 0;
        }
        case PAP:
        {
            l1_index = GET_INDEX(pc >> 1, a->ght_index_bits);
            l2_index = GET_INDEX(pc >> 1, a->pht_index_bits);
            ghr = GET_INDEX(a->ght[l1_index].ghr, a->hreg_bits);
            if (a->pht[l2_index].ctr[ghr] > 1)
            {
                return 1;
            }
            return 0;
        }
    }

    assert(0);
    return 0;
}

/**
 * Based on the adaptive predictor scheme, allocate the entries in GHT or
 * PHT or both.
 *
 * History register in the allocated GHT entry is set to 0.
 * 2-bit saturating counter array in the allocated PHT entry get default values
 * of 0 (strongly not taken).
 */
void
adaptive_predictor_add(AdaptivePredictor *a, target_ulong pc)
{
    int l1_index;
    int l2_index;

    switch (a->type)
    {
        case GAG:
        {
            break;
        }
        case GAP:
        {
            l2_index = GET_INDEX(pc >> 1, a->pht_index_bits);
            a->pht[l2_index].pc = pc;
            memset(a->pht[l2_index].ctr, 0, sizeof(int) * (1 << a->hreg_bits));
            break;
        }
        case PAG:
        {
            l1_index = GET_INDEX(pc >> 1, a->ght_index_bits);
            a->ght[l1_index].pc = pc;
            a->ght[l1_index].ghr = 0;
            break;
        }
        case PAP:
        {
            l1_index = GET_INDEX(pc >> 1, a->ght_index_bits);
            l2_index = GET_INDEX(pc >> 1, a->pht_index_bits);
            a->ght[l1_index].pc = pc;
            a->ght[l1_index].ghr = 0;
            a->pht[l2_index].pc = pc;
            memset(a->pht[l2_index].ctr, 0, sizeof(int) * (1 << a->hreg_bits));
            break;
        }
    }
}

/**
 * Based on the adaptive predictor scheme, update the entries in GHT or
 * PHT or both. First update the 2-bit saturating counter in PHT and then
 * update history register in GHT. Update to history register is performed
 * by left-shifting the current branch outcome into its previous value.
 */
void
adaptive_predictor_update(AdaptivePredictor *a, target_ulong pc, int pred)
{
    int l1_index;
    int l2_index;
    uint32_t ghr;

    switch (a->type)
    {
        case GAG:
        {
            ghr = GET_INDEX(a->ght[0].ghr, a->hreg_bits);

            /* Apply aliasing function for Gshare and Gselect */
            ghr = a->pfn_ap_aliasing_func(a, ghr, pc);
            update_two_bit_counter(&a->pht[0].ctr[ghr], pred);
            a->ght[0].ghr = UPDATE_GHR(a->ght[0].ghr, a->hreg_bits, pred);
            break;
        }
        case GAP:
        {
            ghr = GET_INDEX(a->ght[0].ghr, a->hreg_bits);
            l2_index = GET_INDEX(pc >> 1, a->pht_index_bits);
            update_two_bit_counter(&a->pht[l2_index].ctr[ghr], pred);
            a->ght[0].ghr = UPDATE_GHR(a->ght[0].ghr, a->hreg_bits, pred);
            break;
        }
        case PAG:
        {
            l1_index = GET_INDEX(pc >> 1, a->ght_index_bits);
            ghr = GET_INDEX(a->ght[l1_index].ghr, a->hreg_bits);
            update_two_bit_counter(&a->pht[0].ctr[ghr], pred);
            a->ght[l1_index].ghr
                = UPDATE_GHR(a->ght[l1_index].ghr, a->hreg_bits, pred);
            break;
        }
        case PAP:
        {
            l1_index = GET_INDEX(pc >> 1, a->ght_index_bits);
            ghr = GET_INDEX(a->ght[l1_index].ghr, a->hreg_bits);
            l2_index = GET_INDEX(pc >> 1, a->pht_index_bits);
            update_two_bit_counter(&a->pht[l2_index].ctr[ghr], pred);
            a->ght[l1_index].ghr
                = UPDATE_GHR(a->ght[l1_index].ghr, a->hreg_bits, pred);
            break;
        }
    }
}

void
adaptive_predictor_flush(AdaptivePredictor *a)
{
    int i;

    memset(a->ght, 0, a->ght_size * sizeof(GHTEntry));
    for (i = 0; i < a->pht_size; ++i)
    {
        a->pht[i].pc = 0;
        memset(a->pht[i].ctr, 0, sizeof(int) * (1 << a->hreg_bits));
    }
}

AdaptivePredictor *
adaptive_predictor_init(const SimParams *p)
{
    int i;
    AdaptivePredictor *a;

    a = (AdaptivePredictor *)calloc(1, sizeof(AdaptivePredictor));
    assert(a);

    /* Create GHT */
    a->ght = (GHTEntry *)malloc(sizeof(GHTEntry) * p->bpu_ght_size);
    assert(a->ght);
    a->ght_size = p->bpu_ght_size;
    a->ght_index_bits = GET_NUM_BITS(p->bpu_ght_size);
    a->hreg_bits = p->bpu_history_bits;
    memset(a->ght, 0, a->ght_size * sizeof(GHTEntry));

    /* Create PHT */
    a->pht = (PHTEntry *)malloc(sizeof(PHTEntry) * p->bpu_pht_size);
    assert(a->pht);
    a->pht_size = p->bpu_pht_size;
    a->pht_index_bits = GET_NUM_BITS(p->bpu_pht_size);
    memset(a->pht, 0, a->pht_size * sizeof(PHTEntry));
    for (i = 0; i < a->pht_size; ++i)
    {
        a->pht[i].ctr = (int *)malloc(sizeof(int) * (1 << a->hreg_bits));
        assert(a->pht[i].ctr);
        memset(a->pht[i].ctr, 0, sizeof(int) * (1 << a->hreg_bits));
    }
    a->pfn_ap_aliasing_func = NULL;
    /* Set prediction scheme */
    if ((a->ght_size == 1) && (a->pht_size == 1))
    {
        a->type = GAG;

        /* For GAg based scheme, setup aliasing function to be used (For Gshare
         * and Gselect predictors) */
        switch (p->bpu_aliasing_func_type)
        {
            case BPU_ALIAS_FUNC_XOR:
            {
                a->pfn_ap_aliasing_func = &xor_aliasing_func;
                break;
            }
            case BPU_ALIAS_FUNC_AND:
            {
                a->pfn_ap_aliasing_func = &and_aliasing_func;
                break;
            }
            case BPU_ALIAS_FUNC_NONE:
            {
                a->pfn_ap_aliasing_func = &none_aliasing_func;
                break;
            }
        }
    }

    if ((a->ght_size == 1) && (a->pht_size > 1))
    {
        a->type = GAP;
    }
    if ((a->ght_size > 1) && (a->pht_size == 1))
    {
        a->type = PAG;
    }
    if ((a->ght_size > 1) && (a->pht_size > 1))
    {
        a->type = PAP;
    }

    return a;
}

void
adaptive_predictor_free(AdaptivePredictor **a)
{
    int i;

    for (i = 0; i < (*a)->pht_size; ++i)
    {
        free((*a)->pht[i].ctr);
        (*a)->pht[i].ctr = NULL;
    }
    free((*a)->pht);
    (*a)->pht = NULL;
    free((*a)->ght);
    (*a)->ght = NULL;
    free(*a);
    *a = NULL;
}