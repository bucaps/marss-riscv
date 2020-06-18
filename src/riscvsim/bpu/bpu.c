/**
 * Branch Prediction Unit
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
#include "bpu.h"

static void
bpu_flush(BranchPredUnit *u)
{
    u->btb->flush(u->btb);

    switch (u->bpu_type)
    {
        case BPU_TYPE_BIMODAL:
        {
            u->bht->flush(u->bht);
            break;
        }

        case BPU_TYPE_ADAPTIVE:
        {
            u->ap->flush(u->ap);
            break;
        }
    }

    if (u->ras)
    {
        u->ras->flush(u->ras);
    }
}

/* Probes the BPU for given pc. */
static void
bpu_probe(BranchPredUnit *u, target_ulong pc, BPUResponsePkt *p, int priv)
{
    p->ap_probe_status = BPU_HIT;
    p->btb_probe_status = u->btb->probe(u->btb, pc, &p->btb_entry);
    if (p->btb_probe_status == BPU_HIT)
    {
        ++(u->stats[priv].btb_hits);
    }
    ++(u->stats[priv].btb_probes);

    /* TODO: verify */
    if (u->ap && ((p->btb_probe_status == BPU_MISS)
                  || (p->btb_entry->type == BRANCH_COND)))
    {
        p->ap_probe_status = u->ap->probe(u->ap, pc);
    }

    p->bpu_probe_status = p->btb_probe_status && p->ap_probe_status;
}

/**
 * Returns the target address for this pc. Predictions for conditional branches
 * are checked before returning the target address. If prediction is
 * taken,target address is returned, else 0 is returned.
 */
static target_ulong
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
                    if (u->bht->get_prediction(u->bht, pc) > 1)
                    {
                        return btb_entry->target;
                    }
                    break;
                }

                case BPU_TYPE_ADAPTIVE:
                {
                    if (u->ap->get_prediction(u->ap, pc))
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

static void
bpu_add(BranchPredUnit *u, target_ulong pc, int type, BPUResponsePkt *p,
        int priv, int fret)
{
    /* All the branches are allocated BTB entry */
    if (!p->btb_probe_status)
    {
        /* If using return address stack, don't add function returns to BTB */
        if (!(u->ras && fret))
        {
            u->btb->add(u->btb, pc, type);
            ++(u->stats[priv].btb_inserts);
        }
    }

    switch (u->bpu_type)
    {
        case BPU_TYPE_BIMODAL:
        {
            if (type == BRANCH_COND)
            {
                u->bht->add(u->bht, pc);
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
                u->ap->add(u->ap, pc);
            }
            break;
        }
    }
}

static void
bpu_update(BranchPredUnit *u, target_ulong pc, target_ulong target, int pred,
           int type, BPUResponsePkt *p, int priv)
{
    if (p->btb_probe_status)
    {
        u->btb->update(p->btb_entry, target, type);
        ++(u->stats[priv].btb_updates);
    }

    switch (u->bpu_type)
    {
        case BPU_TYPE_BIMODAL:
        {
            if (type == BRANCH_COND)
            {
                u->bht->update(u->bht, pc, pred);
            }
            break;
        }

        case BPU_TYPE_ADAPTIVE:
        {
            /* If BPU is using adaptive predictor, adaptive predictor structures
             * must be also be updated, but only for conditional branches */
            if ((type == BRANCH_COND) && p->ap_probe_status)
            {
                u->ap->update(u->ap, pc, pred);
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

    u->probe = &bpu_probe;
    u->add = &bpu_add;
    u->update = &bpu_update;
    u->flush = &bpu_flush;
    u->get_target = &bpu_get_target;
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