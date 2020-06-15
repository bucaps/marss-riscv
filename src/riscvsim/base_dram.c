/**
 * Base DRAM model
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "base_dram.h"

static void
read_complete_callback(target_ulong addr, StageMemAccessQueue *f,
                       StageMemAccessQueue *b)
{
    int i;

    for (i = 0; i < f->cur_idx; ++i)
    {
        if ((f->entry[i].valid) && (f->entry[i].addr == addr)
            && (f->entry[i].type == Read))
        {
            f->entry[i].valid = FALSE;
            --f->cur_size;
            return;
        }
    }

    for (i = 0; i < b->cur_idx; ++i)
    {
        if ((b->entry[i].valid) && (b->entry[i].addr == addr)
            && (b->entry[i].type == Read))
        {
            b->entry[i].valid = FALSE;
            --b->cur_size;
            return;
        }
    }
}

static void
write_complete_callback(target_ulong addr, StageMemAccessQueue *f,
                        StageMemAccessQueue *b)
{
    int i;

    for (i = 0; i < f->cur_idx; ++i)
    {
        if ((f->entry[i].valid) && (f->entry[i].addr == addr)
            && (f->entry[i].type == Write))
        {
            f->entry[i].valid = FALSE;
            --f->cur_size;
            return;
        }
    }

    for (i = 0; i < b->cur_idx; ++i)
    {
        if ((b->entry[i].valid) && (b->entry[i].addr == addr)
            && (b->entry[i].type == Write))
        {
            b->entry[i].valid = FALSE;
            --b->cur_size;
            return;
        }
    }
}

static int
base_dram_can_accept_request(BaseDram *d)
{
    return !d->mem_access_active;
}

static void
base_dram_send_request(BaseDram *d, PendingMemAccessEntry *e)
{
    uint64_t current_page_num;

    if (e->req_pte)
    {
        /* This access is related to reading/writing page table entry */
        d->max_clock_cycles = d->pte_rw_latency;
    }
    else
    {
        d->max_clock_cycles = d->mem_access_latency;
    }

    /* Remove page offset to get current page number, page size is always 4KB */
    current_page_num = e->addr >> 12;

    if (d->last_accessed_page_num == current_page_num)
    {
        /* Page hit */
        d->max_clock_cycles *= 0.6;
    }
    else
    {
        /* Page miss */
        d->last_accessed_page_num = current_page_num;
    }

    /* Send a write complete callback to the calling pipeline stage as
     * we don't want the pipeline stage to wait for write to complete.
     * But, simulate this write delay asynchronously via the memory
     * controller */
    if (e->type == Write)
    {
        write_complete_callback(e->addr, d->frontend_mem_access_queue,
                                d->backend_mem_access_queue);
    }

    d->active_mem_request = e;
    d->mem_access_active = TRUE;
    d->elasped_clock_cycles = 1;
}

static int
base_dram_clock(BaseDram *d)
{
    if (d->mem_access_active)
    {
        if (d->elasped_clock_cycles == d->max_clock_cycles)
        {
            d->mem_access_active = FALSE;
            d->max_clock_cycles = 0;
            d->elasped_clock_cycles = 0;

            if (d->active_mem_request->type == Read)
            {
                read_complete_callback(d->active_mem_request->addr,
                                       d->frontend_mem_access_queue,
                                       d->backend_mem_access_queue);
            }

            d->active_mem_request->valid = FALSE;
            return TRUE;
        }
        else
        {
            d->elasped_clock_cycles++;
        }
    }

    return FALSE;
}

static void
base_dram_reset(BaseDram *d)
{
    d->elasped_clock_cycles = 0;
    d->max_clock_cycles = 0;
    d->mem_access_active = FALSE;
    d->active_mem_request = NULL;
    d->last_accessed_page_num = 0;
}

BaseDram *
base_dram_create(const SimParams *p, StageMemAccessQueue *f,
                 StageMemAccessQueue *b)
{
    BaseDram *d;

    d = calloc(1, sizeof(BaseDram));
    assert(d);

    d->pte_rw_latency = p->pte_rw_latency;
    d->mem_access_latency = p->mem_access_latency;
    d->frontend_mem_access_queue = f;
    d->backend_mem_access_queue = b;

    d->can_accept_request = &base_dram_can_accept_request;
    d->send_request = &base_dram_send_request;
    d->clock = &base_dram_clock;
    d->reset = &base_dram_reset;

    d->reset(d);
    return d;
}

void
base_dram_free(BaseDram **d)
{
    free(*d);
}
