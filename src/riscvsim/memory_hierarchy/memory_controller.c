/**
 * Memory Controller
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../utils/circular_queue.h"
#include "../utils/sim_log.h"
#include "dramsim_wrapper_c_connector.h"
#include "memory_controller.h"

static void
mem_controller_log_config(const MemoryController *m)
{
    sim_log_event_to_file(sim_log, "%s", "Setting up memory controller");
    sim_log_param_to_file(sim_log, "%s: %d", "burst_length", m->burst_length);
    sim_log_param_to_file(sim_log, "%s: %d", "fetch_cpu_stage_queue_size", FRONTEND_MEM_ACCESS_QUEUE_SIZE);
    sim_log_param_to_file(sim_log, "%s: %d", "mem_cpu_stage_queue_size", BACKEND_MEM_ACCESS_QUEUE_SIZE);
    sim_log_param_to_file(sim_log, "%s: %d", "global_mem_request_queue_size", MEM_REQUEST_QUEUE_SIZE);
    sim_log_param_to_file(sim_log, "%s: %s", "dram_model_type", dram_model_type_str[m->dram_model_type]);
}

void
mem_controller_reset(MemoryController *m)
{
    /* Invalidate the entries added to mem_request_queue on the speculated path */
    mem_controller_invalidate_mem_request_queue_entries(
        m, &m->frontend_mem_access_queue);
    mem_controller_invalidate_mem_request_queue_entries(
        m, &m->backend_mem_access_queue);
    mem_controller_reset_cpu_stage_queue(&m->frontend_mem_access_queue);
    mem_controller_reset_cpu_stage_queue(&m->backend_mem_access_queue);
    mem_controller_reset_mem_request_queue(m);
    dram_reset(m->dram);
}

void
mem_controller_set_burst_length(MemoryController *m, int burst_length)
{
    m->burst_length = burst_length;
}

void
mem_controller_reset_cpu_stage_queue(StageMemAccessQueue *q)
{
    q->cur_idx = 0;
    q->cur_size = 0;
}

void
mem_controller_reset_mem_request_queue(MemoryController *m)
{
    cq_reset(&m->mem_request_queue.cq);
}

static void
fill_memory_request_entry(PendingMemAccessEntry *e, target_ulong paddr,
                          MemAccessType type, int is_pte)
{
    e->addr = paddr;
    e->type = type;
    e->req_pte = is_pte;
    e->valid = TRUE;

    /* Don't start simulating DRAM access delay until cache lookup delay is
     * simulated */
    e->start_access = FALSE;
}

int
mem_controller_create_mem_request(MemoryController *m, target_ulong paddr,
                                  int bytes_to_access, MemAccessType type,
                                  void *p_mem_access_info)
{
    target_ulong start_offset;
    int index, source_cpu_stage_id;

    source_cpu_stage_id = *(int *)p_mem_access_info;

    /*  Align the address for this access to the burst_length */
    start_offset = paddr % m->burst_length;
    if (0 != start_offset)
    {
        bytes_to_access += start_offset;
        paddr -= start_offset;
    }

    while (bytes_to_access > 0)
    {
        switch (source_cpu_stage_id)
        {
            case FETCH:
            {
                fill_memory_request_entry(
                    &m->frontend_mem_access_queue
                         .entry[m->frontend_mem_access_queue.cur_idx],
                    paddr, type, FALSE);
                ++m->frontend_mem_access_queue.cur_idx;
                ++m->frontend_mem_access_queue.cur_size;
                break;
            }
            case MEMORY:
            {
                fill_memory_request_entry(
                    &m->backend_mem_access_queue
                         .entry[m->backend_mem_access_queue.cur_idx],
                    paddr, type, FALSE);
                ++m->backend_mem_access_queue.cur_idx;
                ++m->backend_mem_access_queue.cur_size;
                break;
            }
            default:
            {
                sim_assert(
                    (0), "error: %s at line %d in %s(): %s", __FILE__, __LINE__,
                    __func__,
                    "memory access generated by incorrect pipeline stage");
            }
        }

        /* Add requests to the mem_request_queue */
        index = cq_enqueue(&m->mem_request_queue.cq);

        sim_assert((index != -1), "error: %s at line %d in %s(): %s", __FILE__,
                   __LINE__, __func__, "memory request queue is full");

        fill_memory_request_entry(&m->mem_request_queue.entry[index], paddr,
                                  type, FALSE);

        /* Calculate remaining transactions for this access */
        bytes_to_access -= m->burst_length;
        paddr += m->burst_length;
    }

    return 0;
}

void
mem_controller_clock(MemoryController *m)
{
    PendingMemAccessEntry *e;

    if (dram_can_accept_request(m->dram))
    {
        if (!cq_empty(&m->mem_request_queue.cq))
        {
            e = &m->mem_request_queue.entry[cq_front(&m->mem_request_queue.cq)];
            if (e->valid)
            {
                if (e->start_access)
                {
                    dram_send_request(m->dram, e);
                }
            }
            else
            {
                /* This entry was on miss speculated path, so it was flushed by
                 * the CPU pipeline stage */
                cq_dequeue(&m->mem_request_queue.cq);
            }
        }
    }

    if (dram_clock(m->dram))
    {
        cq_dequeue(&m->mem_request_queue.cq);
    }
}

MemoryController *
mem_controller_init(const SimParams *p)
{
    MemoryController *m;

    m = (MemoryController *)calloc(1, sizeof(MemoryController));
    assert(m);
    m->burst_length = p->burst_length;
    m->dram_model_type = p->dram_model_type;

    m->frontend_mem_access_queue.max_size = FRONTEND_MEM_ACCESS_QUEUE_SIZE;
    m->frontend_mem_access_queue.entry = (PendingMemAccessEntry *)calloc(
        m->frontend_mem_access_queue.max_size, sizeof(PendingMemAccessEntry));
    assert(m->frontend_mem_access_queue.entry);

    m->backend_mem_access_queue.max_size = BACKEND_MEM_ACCESS_QUEUE_SIZE;
    m->backend_mem_access_queue.entry = (PendingMemAccessEntry *)calloc(
        m->backend_mem_access_queue.max_size, sizeof(PendingMemAccessEntry));
    assert(m->backend_mem_access_queue.entry);

    cq_init(&m->mem_request_queue.cq, MEM_REQUEST_QUEUE_SIZE);
    memset((void *)m->mem_request_queue.entry, 0,
           sizeof(PendingMemAccessEntry) * MEM_REQUEST_QUEUE_SIZE);

    m->dram = dram_create(p, &m->frontend_mem_access_queue,
                          &m->backend_mem_access_queue);

    switch (m->dram_model_type)
    {
        case MEM_MODEL_BASE:
        {
            mem_controller_set_burst_length(m, p->burst_length);
            break;
        }
        case MEM_MODEL_DRAMSIM:
        {
            mem_controller_set_burst_length(m, dramsim_get_burst_size());
            break;
        }
        case MEM_MODEL_RAMULATOR:
        {
            mem_controller_set_burst_length(m, p->burst_length);
            break;
        }
        default:
        {
            sim_assert((0), "error: %s at line %d in %s(): %s", __FILE__,
                       __LINE__, __func__, "invalid memory model");
        }
    }
    mem_controller_log_config(m);
    return m;
}

void
mem_controller_free(MemoryController **m)
{
    dram_free(&(*m)->dram);
    free((*m)->backend_mem_access_queue.entry);
    (*m)->backend_mem_access_queue.entry = NULL;
    free((*m)->frontend_mem_access_queue.entry);
    (*m)->frontend_mem_access_queue.entry = NULL;
    free(*m);
    *m = NULL;
}

void
mem_controller_cache_lookup_complete_signal(MemoryController *m,
                                            StageMemAccessQueue *stage_queue)
{
    int i, j;
    target_ulong addr;

    for (j = 0; j < stage_queue->cur_idx; ++j)
    {
        addr = stage_queue->entry[j].addr;

        if (!cq_empty(&m->mem_request_queue.cq))
        {
            if (m->mem_request_queue.cq.rear >= m->mem_request_queue.cq.front)
            {
                for (i = m->mem_request_queue.cq.front;
                     i <= m->mem_request_queue.cq.rear; i++)
                {
                    if (m->mem_request_queue.entry[i].addr == addr)
                    {
                        m->mem_request_queue.entry[i].start_access = TRUE;
                    }
                }
            }
            else
            {
                for (i = m->mem_request_queue.cq.front;
                     i < m->mem_request_queue.cq.max_size; i++)
                {
                    if (m->mem_request_queue.entry[i].addr == addr)
                    {
                        m->mem_request_queue.entry[i].start_access = TRUE;
                    }
                }

                for (i = 0; i <= m->mem_request_queue.cq.rear; i++)
                {
                    if (m->mem_request_queue.entry[i].addr == addr)
                    {
                        m->mem_request_queue.entry[i].start_access = TRUE;
                    }
                }
            }
        }
    }
}

void
mem_controller_invalidate_mem_request_queue_entries(
    MemoryController *m, StageMemAccessQueue *stage_queue)
{
    int i, j;
    target_ulong addr;

    for (j = 0; j < stage_queue->cur_idx; ++j)
    {
        addr = stage_queue->entry[j].addr;

        if (!cq_empty(&m->mem_request_queue.cq))
        {
            if (m->mem_request_queue.cq.rear >= m->mem_request_queue.cq.front)
            {
                for (i = m->mem_request_queue.cq.front;
                     i <= m->mem_request_queue.cq.rear; i++)
                {
                    if (m->mem_request_queue.entry[i].addr == addr)
                    {
                        m->mem_request_queue.entry[i].valid = FALSE;
                    }
                }
            }
            else
            {
                for (i = m->mem_request_queue.cq.front;
                     i < m->mem_request_queue.cq.max_size; i++)
                {
                    if (m->mem_request_queue.entry[i].addr == addr)
                    {
                        m->mem_request_queue.entry[i].valid = FALSE;
                    }
                }

                for (i = 0; i <= m->mem_request_queue.cq.rear; i++)
                {
                    if (m->mem_request_queue.entry[i].addr == addr)
                    {
                        m->mem_request_queue.entry[i].valid = FALSE;
                    }
                }
            }
        }
    }
}