/**
 * Branch Prediction Unit
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
#include "bpu.h"

void
bpu_flush(BranchPredUnit *u)
{
    btb_flush(u->btb);

    switch (u->bpu_type)
    {
        case BPU_TYPE_BIMODAL:
        {
            bht_flush(u->bht);
            break;
        }

        case BPU_TYPE_ADAPTIVE:
        {
            adaptive_predictor_flush(u->ap);
            break;
        }
    }

    if (u->ras)
    {
        ras_flush(u->ras);
    }
}

/* Probes the BPU for given pc. */
void
bpu_probe(BranchPredUnit *u, target_ulong pc, BPUResponsePkt *p, int priv)
{
    /* Check if the PC is present in BTB */
    p->btb_probe_status = btb_probe(u->btb, pc, &p->btb_entry);
    ++(u->stats[priv].btb_probes);

    if (p->btb_probe_status == BPU_HIT)
    {
        ++(u->stats[priv].btb_hits);
    }

    switch (u->bpu_type)
    {
        case BPU_TYPE_BIMODAL:
        {
            p->ap_probe_status = BPU_HIT;
            break;
        }

        case BPU_TYPE_ADAPTIVE:
        {
            p->ap_probe_status = adaptive_predictor_probe(u->ap, pc);

            if (p->btb_probe_status == BPU_HIT)
            {
                /* If the PC present in BTB is a unconditional branch, mark
                 * ap_probe_status as HIT */
                if (p->btb_entry->type == BRANCH_UNCOND)
                {
                    p->ap_probe_status = BPU_HIT;
                }
            }
            break;
        }
    }

    p->bpu_probe_status = p->btb_probe_status & p->ap_probe_status;
}

/**
 * Returns the target address for this pc. Predictions for conditional branches
 * are checked before returning the target address. If prediction is
 * taken,target address is returned, else 0 is returned.
 */
target_ulong
bpu_get_target(BranchPredUnit *u, target_ulong pc, BtbEntry *btb_entry)
{
    switch (btb_entry->type)
    {
        case BRANCH_UNCOND:
        {
            /* No need to check prediction for unconditional branches, so
               directly return target address. */
            return btb_entry->target;
        }

        case BRANCH_COND:
        {
            /* Must check prediction for conditional branches, so if prediction
               is taken return the target address, else return 0. */
            switch (u->bpu_type)
            {
                case BPU_TYPE_BIMODAL:
                {
                    if (bht_get_prediction(u->bht, pc) > 1)
                    {
                        return btb_entry->target;
                    }
                    break;
                }

                case BPU_TYPE_ADAPTIVE:
                {
                    if (adaptive_predictor_get_prediction(u->ap, pc))
                    {
                        return btb_entry->target;
                    }
                    break;
                }
            }
            break;
        }
    }

    /* BPU Hit, but prediction is not-taken */
    return 0;
}

void
bpu_add(BranchPredUnit *u, target_ulong pc, int type, BPUResponsePkt *p,
        int priv, int fret)
{
    /* All the branches are allocated BTB entry */
    if (!p->btb_probe_status)
    {
        /* If using return address stack, don't add function returns to BTB */
        if (u->ras && fret)
        {
            return;
        }

        btb_add(u->btb, pc, type);
        ++(u->stats[priv].btb_inserts);
    }

    switch (u->bpu_type)
    {
        case BPU_TYPE_BIMODAL:
        {
            if (type == BRANCH_COND)
            {
                bht_add(u->bht, pc);
            }
            break;
        }

        case BPU_TYPE_ADAPTIVE:
        {
            /* If BPU is using adaptive predictor, then PC must also be added in
             * adaptive predictor structures, but only for conditional branches
             */
            if ((type == BRANCH_COND) && !p->ap_probe_status)
            {
                adaptive_predictor_add(u->ap, pc);
            }
            break;
        }
    }
}

void
bpu_update(BranchPredUnit *u, target_ulong pc, target_ulong target, int pred,
           int type, BPUResponsePkt *p, int priv)
{
    if (p->btb_probe_status)
    {
        btb_update(p->btb_entry, target, type);
        ++(u->stats[priv].btb_updates);
    }

    switch (u->bpu_type)
    {
        case BPU_TYPE_BIMODAL:
        {
            if (type == BRANCH_COND)
            {
                bht_update(u->bht, pc, pred);
            }
            break;
        }

        case BPU_TYPE_ADAPTIVE:
        {
            /* If BPU is using adaptive predictor, adaptive predictor structures
             * must be also be updated, but only for conditional branches */
            if ((type == BRANCH_COND) && p->ap_probe_status)
            {
                adaptive_predictor_update(u->ap, pc, pred);
            }
            break;
        }
    }
}

BranchPredUnit *
bpu_init(const SimParams *p, SimStats *s)
{
    BranchPredUnit *u;

    u = (BranchPredUnit *)calloc(1, sizeof(BranchPredUnit));
    assert(u);
    u->btb = NULL;
    u->bht = NULL;
    u->ap = NULL;
    u->ras = NULL;
    u->stats = s;
    u->btb = btb_init(p);
    u->bpu_type = p->bpu_type;

    switch (u->bpu_type)
    {
        case BPU_TYPE_BIMODAL:
        {
            u->bht = bht_init(p);
            break;
        }

        case BPU_TYPE_ADAPTIVE:
        {
            u->ap = adaptive_predictor_init(p);
            break;
        }
    }

    if (p->ras_size)
    {
        u->ras = ras_init(p);
    }

    return u;
}

void
bpu_free(BranchPredUnit **u)
{
    btb_free(&(*u)->btb);

    switch ((*u)->bpu_type)
    {
        case BPU_TYPE_BIMODAL:
        {
            bht_free(&(*u)->bht);
            break;
        }

        case BPU_TYPE_ADAPTIVE:
        {
            adaptive_predictor_free(&(*u)->ap);
            break;
        }
    }

    if ((*u)->ras)
    {
        ras_free(&(*u)->ras);
    }

    free(*u);
    *u = NULL;
}