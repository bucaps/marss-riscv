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
#include "dramsim_wrapper_c_connector.h"
#include "memory_controller.h"

static void
mem_controller_reset(MemoryController *m)
{
    m->flush_cpu_stage_queue(&m->frontend_mem_access_queue);
    m->flush_cpu_stage_queue(&m->backend_mem_access_queue);
    m->flush_mem_request_queue(m);

    switch (m->dram_model_type)
    {
        case MEM_MODEL_BASE:
        {
            m->base_dram->reset(m->base_dram);
            break;
        }
        case MEM_MODEL_DRAMSIM:
        {
            dramsim_wrapper_destroy();
            break;
        }
        default:
        {
            fprintf(stderr, "error: invalid memory model\n");
            exit(1);
        }
    }
}

static void
mem_controller_set_burst_length(MemoryController *m, int burst_length)
{
    m->burst_length = burst_length;
}

static void
mem_controller_flush_cpu_stage_queue(StageMemAccessQueue *q)
{
    q->cur_idx = 0;
    q->cur_size = 0;
}

static void
mem_controller_flush_mem_request_queue(MemoryController *m)
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
}

static int
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
                assert(0);
            }
        }

        /* Add transaction for this access to mem_request_queue */
        index = cq_enqueue(&m->mem_request_queue.cq);
        assert(index != -1);
        fill_memory_request_entry(&m->mem_request_queue.entry[index], paddr,
                                  type, FALSE);

        /* Calculate remaining transactions for this access */
        bytes_to_access -= m->burst_length;
        paddr += m->burst_length;
    }

    return 0;
}

static int
mem_controller_create_mem_request_pte(MemoryController *m, target_ulong paddr,
                                      int bytes_to_access, MemAccessType type,
                                      void *p_mem_access_info)
{
    int index, source_cpu_stage_id;

    source_cpu_stage_id = *(int *)p_mem_access_info;

    switch (source_cpu_stage_id)
    {
        case FETCH:
        {
            fill_memory_request_entry(
                &m->frontend_mem_access_queue
                     .entry[m->frontend_mem_access_queue.cur_idx],
                paddr, type, TRUE);
            ++m->frontend_mem_access_queue.cur_idx;
            ++m->frontend_mem_access_queue.cur_size;
            break;
        }
        case MEMORY:
        {
            fill_memory_request_entry(
                &m->backend_mem_access_queue
                     .entry[m->backend_mem_access_queue.cur_idx],
                paddr, type, TRUE);
            ++m->backend_mem_access_queue.cur_idx;
            ++m->backend_mem_access_queue.cur_size;
            break;
        }
        default:
        {
            assert(0);
        }
    }

    /* Add transaction for this access to mem_request_queue */
    index = cq_enqueue(&m->mem_request_queue.cq);
    assert(index != -1);
    fill_memory_request_entry(&m->mem_request_queue.entry[index], paddr, type,
                              TRUE);

    return 0;
}

static void
mem_controller_clock_base(MemoryController *m)
{
    PendingMemAccessEntry *e;

    if (m->base_dram->can_accept_request(m->base_dram))
    {
        if (!cq_empty(&m->mem_request_queue.cq))
        {
            e = &m->mem_request_queue.entry[cq_front(&m->mem_request_queue.cq)];
            m->base_dram->send_request(m->base_dram, e);
        }
    }

    if (m->base_dram->clock(m->base_dram))
    {
        cq_dequeue(&m->mem_request_queue.cq);
    }
}

static void
mem_controller_clock_dramsim(MemoryController *m)
{
    PendingMemAccessEntry *e;

    while (!cq_empty(&m->mem_request_queue.cq))
    {
        e = &m->mem_request_queue.entry[cq_front(&m->mem_request_queue.cq)];
        if (dramsim_wrapper_can_add_transaction(e->addr))
        {
            dramsim_wrapper_add_transaction(e->addr, e->type);
            cq_dequeue(&m->mem_request_queue.cq);
        }
        else
        {
            break;
        }
    }

    dramsim_wrapper_update();
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

    m->reset = &mem_controller_reset;
    m->flush_cpu_stage_queue = &mem_controller_flush_cpu_stage_queue;
    m->flush_mem_request_queue = &mem_controller_flush_mem_request_queue;
    m->set_burst_length = &mem_controller_set_burst_length;
    m->create_mem_request = &mem_controller_create_mem_request;
    m->create_mem_request_pte = &mem_controller_create_mem_request_pte;

    PRINT_INIT_MSG("Setting up dram");

    switch (m->dram_model_type)
    {
        case MEM_MODEL_BASE:
        {
            PRINT_INIT_MSG("Setting up base dram model");
            m->clock = &mem_controller_clock_base;
            m->set_burst_length(m, p->burst_length);
            m->base_dram = base_dram_create(p, &m->frontend_mem_access_queue,
                                            &m->backend_mem_access_queue);
            break;
        }
        case MEM_MODEL_DRAMSIM:
        {
            PRINT_INIT_MSG("Setting up DRAMSim2");
            dramsim_wrapper_init(
                p->dramsim_ini_file, p->dramsim_system_ini_file,
                p->dramsim_stats_dir, p->core_name, p->guest_ram_size,
                &m->frontend_mem_access_queue, &m->backend_mem_access_queue);

            m->clock = &mem_controller_clock_dramsim;
            m->set_burst_length(m, dramsim_get_burst_size());
            break;
        }
        default:
        {
            fprintf(stderr, "error: invalid memory model\n");
            exit(1);
        }
    }

    return m;
}

void
mem_controller_free(MemoryController **m)
{
    switch ((*m)->dram_model_type)
    {
        case MEM_MODEL_BASE:
        {
            base_dram_free(&(*m)->base_dram);
            break;
        }
        case MEM_MODEL_DRAMSIM:
        {
            dramsim_wrapper_destroy();
            break;
        }
        default:
        {
            fprintf(stderr, "error: invalid memory model\n");
            exit(1);
        }
    }
    free((*m)->backend_mem_access_queue.entry);
    (*m)->backend_mem_access_queue.entry = NULL;
    free((*m)->frontend_mem_access_queue.entry);
    (*m)->frontend_mem_access_queue.entry = NULL;
    free(*m);
    *m = NULL;
}