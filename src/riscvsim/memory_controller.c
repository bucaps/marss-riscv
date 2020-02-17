/**
 * Memory Controller
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
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "circular_queue.h"
#include "memory_controller.h"
#include "dramsim_wrapper_c_connector.h"

MemoryController *
mem_controller_init(const SimParams *p)
{
    MemoryController *m;

    m = (MemoryController *)calloc(1, sizeof(MemoryController));
    assert(m);
    m->dram_burst_size = p->dram_burst_size;
    m->mem_model_type = p->mem_model_type;

    m->frontend_mem_access_queue.max_size = FRONTEND_MEM_ACCESS_QUEUE_SIZE;
    m->frontend_mem_access_queue.entry = (PendingMemAccessEntry *)calloc(
        m->frontend_mem_access_queue.max_size, sizeof(PendingMemAccessEntry));
    assert(m->frontend_mem_access_queue.entry);

    m->backend_mem_access_queue.max_size = BACKEND_MEM_ACCESS_QUEUE_SIZE;
    m->backend_mem_access_queue.entry = (PendingMemAccessEntry *)calloc(
        m->backend_mem_access_queue.max_size, sizeof(PendingMemAccessEntry));
    assert(m->backend_mem_access_queue.entry);

    cq_init(&m->dram_dispatch_queue.cq, DRAM_DISPATCH_QUEUE_SIZE);
    memset((void *)m->dram_dispatch_queue.entry, 0,
           sizeof(PendingMemAccessEntry) * DRAM_DISPATCH_QUEUE_SIZE);

    PRINT_INIT_MSG("Setting up dram");

    switch (m->mem_model_type)
    {
        case MEM_MODEL_BASE:
        {
            PRINT_INIT_MSG("Setting up base dram model");
            m->dram = dram_init(p, p->guest_ram_size, DRAM_NUM_DIMMS, DRAM_NUM_BANKS,
                                DRAM_MEM_BUS_WIDTH, DRAM_BANK_COL_SIZE);
            m->mem_controller_update_internal = &mem_controller_update_base;
            mem_controller_set_dram_burst_size(m, p->dram_burst_size);
            break;
        }
        case MEM_MODEL_DRAMSIM:
        {
            PRINT_INIT_MSG("Setting up DRAMSim2");
            dramsim_wrapper_init(
                p->dramsim_ini_file, p->dramsim_system_ini_file,
                p->dramsim_stats_dir, p->core_name, p->guest_ram_size,
                &m->frontend_mem_access_queue, &m->backend_mem_access_queue);

            m->mem_controller_update_internal = &mem_controller_update_dramsim;
            mem_controller_set_dram_burst_size(m, dramsim_get_burst_size());

            /* Check if the cache line size equals DRAMSim burst size */
            if ((p->words_per_cache_line * sizeof(target_ulong))
                != dramsim_get_burst_size())
            {
                fprintf(stderr, "error: cache line size and dramsim burst size "
                                "not equal\n");
                exit(1);
            }
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
    switch ((*m)->mem_model_type)
    {
        case MEM_MODEL_BASE:
        {
            dram_free(&(*m)->dram);
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
    free((*m)->latency_array);
    (*m)->latency_array = NULL;
    free((*m)->latency_status_bv);
    (*m)->latency_status_bv = NULL;
    free(*m);
    *m = NULL;
}

void
mem_controller_reset(MemoryController *m)
{
    m->current_latency = 0;
    m->mem_access_active = 0;

    mem_controller_flush_stage_mem_access_queue(&m->frontend_mem_access_queue);
    mem_controller_flush_stage_mem_access_queue(&m->backend_mem_access_queue);

    cq_reset(&m->dram_dispatch_queue.cq);
}


void
mem_controller_set_dram_burst_size(MemoryController *m, int dram_burst_size)
{
    m->dram_burst_size = dram_burst_size;
}

int
mem_controller_access_dram(MemoryController *m, target_ulong paddr, int bytes_to_access,
           MemAccessType type, void *p_mem_access_info)
{
    int index, stage;
    int stage_queue_index;
    target_ulong start_offset;
    int max_bytes_to_access = bytes_to_access;

    /* Get stage id which has generated this access, 0 for fetch, 1 for memory */
    stage = *(int *)p_mem_access_info;

    /*  Align the address for this access to the dram_burst_size */
    start_offset = paddr % m->dram_burst_size;
    if (0 != start_offset)
    {
        bytes_to_access += start_offset;
        paddr -= start_offset;
    }

    /* Read/Write data dram_burst_size bytes at a time */
    while (bytes_to_access > 0)
    {
        /* Add transaction for this access to either of the front-end or back-end
         * queue */
        if (stage == FETCH)
        {
            m->frontend_mem_access_queue.entry[m->frontend_mem_access_queue.cur_idx].addr = paddr;
            m->frontend_mem_access_queue.entry[m->frontend_mem_access_queue.cur_idx].type = type;
            m->frontend_mem_access_queue.entry[m->frontend_mem_access_queue.cur_idx].valid = 1;
            m->frontend_mem_access_queue
                .entry[m->frontend_mem_access_queue.cur_idx]
                .max_bytes_to_access
                = max_bytes_to_access;
            m->frontend_mem_access_queue.entry[m->frontend_mem_access_queue.cur_idx].req_addr = 0;
            stage_queue_index = m->frontend_mem_access_queue.cur_idx;
            ++m->frontend_mem_access_queue.cur_idx;
            ++m->frontend_mem_access_queue.cur_size;
        }
        else if (stage == MEMORY)
        {
            m->backend_mem_access_queue.entry[m->backend_mem_access_queue.cur_idx].addr = paddr;
            m->backend_mem_access_queue.entry[m->backend_mem_access_queue.cur_idx].type = type;
            m->backend_mem_access_queue.entry[m->backend_mem_access_queue.cur_idx].valid = 1;
            m->backend_mem_access_queue
                .entry[m->backend_mem_access_queue.cur_idx]
                .max_bytes_to_access
                = max_bytes_to_access;
            m->backend_mem_access_queue.entry[m->backend_mem_access_queue.cur_idx].req_addr = 0;
            stage_queue_index = m->backend_mem_access_queue.cur_idx;
            ++m->backend_mem_access_queue.cur_idx;
            ++m->backend_mem_access_queue.cur_size;
        }
        else
        {
            /* Only fetch and memory stage generate memory access */
            assert(0);
        }

        /* Add transaction for this access to dram dispatch queue */
        index = cq_enqueue(&m->dram_dispatch_queue.cq);

        assert(index != -1);
        m->dram_dispatch_queue.entry[index].addr = paddr;
        m->dram_dispatch_queue.entry[index].type = type;
        m->dram_dispatch_queue.entry[index].bytes_to_access = m->dram_burst_size;
        m->dram_dispatch_queue.entry[index].valid = 1;
        m->dram_dispatch_queue.entry[index].stage_queue_type = stage;
        m->dram_dispatch_queue.entry[index].stage_queue_index = stage_queue_index;

        /* Calculate remaining transactions for this access */
        bytes_to_access -= m->dram_burst_size;
        paddr += m->dram_burst_size;
    }

    return 0;
}

/* Read callback used by base DRAM model */
static void
read_complete(MemoryController *m, target_ulong addr)
{
    int i;

    for (i = 0; i < m->frontend_mem_access_queue.cur_idx; ++i)
    {
        if ((m->frontend_mem_access_queue.entry[i].valid)
            && (m->frontend_mem_access_queue.entry[i].addr == addr)
            && (m->frontend_mem_access_queue.entry[i].type == Read))
        {
            m->frontend_mem_access_queue.entry[i].valid = 0;
            --m->frontend_mem_access_queue.cur_size;
            return;
        }
    }

    for (i = 0; i < m->backend_mem_access_queue.cur_idx; ++i)
    {
        if ((m->backend_mem_access_queue.entry[i].valid)
            && (m->backend_mem_access_queue.entry[i].addr == addr)
            && (m->backend_mem_access_queue.entry[i].type == Read))
        {
            m->backend_mem_access_queue.entry[i].valid = 0;
            --m->backend_mem_access_queue.cur_size;
            return;
        }
    }
}

/* Write callback used by base DRAM model */
static void
write_complete(MemoryController *m, target_ulong addr)
{
    int i;

    for (i = 0; i < m->frontend_mem_access_queue.cur_idx; ++i)
    {
        if ((m->frontend_mem_access_queue.entry[i].valid)
            && (m->frontend_mem_access_queue.entry[i].addr == addr)
            && (m->frontend_mem_access_queue.entry[i].type == Write))
        {
            m->frontend_mem_access_queue.entry[i].valid = 0;
            --m->frontend_mem_access_queue.cur_size;
            return;
        }
    }

    for (i = 0; i < m->backend_mem_access_queue.cur_idx; ++i)
    {
        if ((m->backend_mem_access_queue.entry[i].valid)
            && (m->backend_mem_access_queue.entry[i].addr == addr)
            && (m->backend_mem_access_queue.entry[i].type == Write))
        {
            m->backend_mem_access_queue.entry[i].valid = 0;
            --m->backend_mem_access_queue.cur_size;
            return;
        }
    }
}

/**
 * Entire cache line is read mem_bus_width_bytes at a time.
 * This creates a bit-vector to indicate latency and read status for each
 * of the mem_bus_width_bytes size chunk of cache line.
 */
void
mem_controller_create_wrap_around_bit_vec(MemoryController *m)
{
    m->max_bus_blks = (int )(m->dram_burst_size / m->dram->mem_bus_width_bytes);
    m->latency_array = calloc(m->max_bus_blks, sizeof(int));
    assert(m->latency_array);
    m->latency_status_bv = calloc(m->max_bus_blks, sizeof(int));
    assert(m->latency_status_bv);
}

/* Determine if wrap-around access is needed to read this cache-line. NOTE: In
 * case of cache eviction, wrap-around access is not needed. Entire line has to
 * be fetched. */
static int
wraparound_access_required(MemoryController *m, PendingMemAccessEntry *e,
                           target_ulong *req_addr)
{
    *req_addr = 0;

    switch (e->stage_queue_type)
    {
        case FETCH:
        {

            *req_addr = m->frontend_mem_access_queue.entry[e->stage_queue_index]
                            .req_addr;
            break;
        }
        case MEMORY:
        {
            *req_addr = m->backend_mem_access_queue.entry[e->stage_queue_index]
                            .req_addr;
            break;
        }
    }

    if (*req_addr)
    {
        return 1;
    }

    return 0;
}

/* Determine the mem_bus_width_bytes chunk address (also the location in the
 * bit-vector) from the required physical address */
static int
get_bit_vec_id_from_phy_addr(MemoryController *m, target_ulong cur_addr,
                             target_ulong start_addr)
{
    return (int)((cur_addr % start_addr) / m->dram->mem_bus_width_bytes);
}

static void
reset_bit_vec_for_cache_line_read(MemoryController *m)
{
    memset((void *)m->latency_status_bv, 0, sizeof(int) * m->max_bus_blks);
    memset((void *)m->latency_array, 0, sizeof(int) * m->max_bus_blks);
}

static void
calculate_wraparound_delays(MemoryController *m, PendingMemAccessEntry *e,
                     target_ulong req_addr)
{
    int bit_vec_id, cur_latency;
    target_ulong wpr_start_addr, next_paddr, max_addr;

    /* Reset bit vector and latency for all the chunks of cache line to be read
     * at the granularity of bus width. NOTE: can be removed */
    reset_bit_vec_for_cache_line_read(m);

    /* Address of the last bit plus 1 in the cache line */
    max_addr = e->addr + e->bytes_to_access;

    /* Calculate wrap around read start address and align the address to the
     * nearest low order mem_bus_width_byte */
    wpr_start_addr = req_addr - (req_addr % m->dram->mem_bus_width_bytes);

    /* Query DRAM latency for wrap around start address */
    bit_vec_id = get_bit_vec_id_from_phy_addr(m, wpr_start_addr, e->addr);
    m->latency_array[bit_vec_id]
        = dram_get_latency(m->dram, wpr_start_addr, e->type);

    /* Read starts from here */
    m->cur_bit_vec_id = bit_vec_id;
    m->wpr_bit_vec_start_id = bit_vec_id;

    /* Calculate delays for the second half of the cache line */
    next_paddr = wpr_start_addr + m->dram->mem_bus_width_bytes;

    while (next_paddr < max_addr)
    {
        cur_latency = dram_get_latency(m->dram, next_paddr, e->type);
        bit_vec_id = get_bit_vec_id_from_phy_addr(m, next_paddr, e->addr);
        m->latency_array[bit_vec_id] = cur_latency;
        next_paddr += m->dram->mem_bus_width_bytes;
    }

    /* Calculate delays for the first half of the cache line */
    next_paddr = e->addr;
    while (next_paddr < wpr_start_addr)
    {
        cur_latency = dram_get_latency(m->dram, next_paddr, e->type);
        bit_vec_id = get_bit_vec_id_from_phy_addr(m, next_paddr, e->addr);
        m->latency_array[bit_vec_id] = cur_latency;
        next_paddr += m->dram->mem_bus_width_bytes;
    }
}

static void
calculate_seq_access_delays(MemoryController *m, PendingMemAccessEntry *e)
{
    int bit_vec_id, cur_latency;
    target_ulong next_paddr, max_addr;

    /* Reset bit vector and latency for all the chunks of cache line to be read
     * at the granularity of bus width. NOTE: can be removed */
    reset_bit_vec_for_cache_line_read(m);

    /* Address of the last bit plus 1 in cache line */
    max_addr = e->addr + e->bytes_to_access;

    next_paddr = e->addr;
    while (next_paddr < max_addr)
    {
        cur_latency = dram_get_latency(m->dram, next_paddr, e->type);
        bit_vec_id = get_bit_vec_id_from_phy_addr(m, next_paddr, e->addr);
        assert((bit_vec_id >= 0) && (bit_vec_id < m->max_bus_blks));
        m->latency_array[bit_vec_id] = cur_latency;
        next_paddr += m->dram->mem_bus_width_bytes;
    }
}

static void
do_wrap_around_read(MemoryController *m, PendingMemAccessEntry *e)
{
    if (m->cur_bit_vec_id <= m->last_bit_vec_id)
    {
        if (m->current_latency == m->latency_array[m->cur_bit_vec_id])
        {
            /* Latency for current blk is simulated */
            m->latency_status_bv[m->cur_bit_vec_id] = 1;

            /* Check if we read wrap-around start address and fire the read
             * callback to the waiting stage */
            if (m->cur_bit_vec_id == m->wpr_bit_vec_start_id)
            {
                if (e->type == Read)
                {
                    read_complete(m, e->addr);
                }
            }

            /* For next blk */
            m->cur_bit_vec_id++;
            m->current_latency = 0;
            m->num_bit_vec_read++;
        }

        if (m->cur_bit_vec_id > m->last_bit_vec_id)
        {
            /* All the blks in cache line are fetched */
            if (m->num_bit_vec_read >= m->max_bus_blks)
            {
                m->mem_access_active = 0;
                m->wrap_around_mode = 0;

                /* Remove the entry */
                e->valid = 0;
                cq_dequeue(&m->dram_dispatch_queue.cq);
            }
            else
            {
                /* Read the first half now */
                m->current_latency = 0;
                m->cur_bit_vec_id = 0;
                m->last_bit_vec_id = m->wpr_bit_vec_start_id - 1;
            }
        }
        else
        {
            m->current_latency++;
        }
    }
}

static void
do_seq_read(MemoryController *m, PendingMemAccessEntry *e)
{
    if (m->cur_bit_vec_id <= m->last_bit_vec_id)
    {
        if (m->current_latency == m->latency_array[m->cur_bit_vec_id])
        {
            /* Latency for current blk is simulated */
            m->latency_status_bv[m->cur_bit_vec_id] = 1;

            /* For next blk */
            m->cur_bit_vec_id++;
            m->current_latency = 0;
        }

        if (m->cur_bit_vec_id > m->last_bit_vec_id)
        {
            /* All the blks in cache line are fetched */
            m->mem_access_active = 0;

            /* Fire read callback */
            if (e->type == Read)
            {
                read_complete(m, e->addr);
            }

            /* Remove the entry */
            e->valid = 0;
            cq_dequeue(&m->dram_dispatch_queue.cq);
        }
        else
        {
            m->current_latency++;
        }
    }
}

int
mem_controller_wrap_around_read_pending(MemoryController *m, target_ulong addr)
{
    int bit_vec_id;
    PendingMemAccessEntry *e;
    target_ulong start_byte_addr;

    if (m->mem_access_active)
    {
        e = &m->dram_dispatch_queue.entry[cq_front(&m->dram_dispatch_queue.cq)];
        if ((addr >= e->addr) && (addr < (e->addr + e->bytes_to_access)))
        {
            /* Cache line containing the word is being fetched */
            start_byte_addr = addr - (addr % m->dram->mem_bus_width_bytes);
            bit_vec_id
                = get_bit_vec_id_from_phy_addr(m, start_byte_addr, e->addr);
            if (!m->latency_status_bv[bit_vec_id])
            {
                /* bit vector for word is not received */
                return 1;
            }
        }
    }

    return 0;
}

void
mem_controller_req_fast_read_for_addr(StageMemAccessQueue *q, target_ulong addr)
{
    int i;

    for (i = 0; i < q->cur_size; ++i)
    {
        PendingMemAccessEntry *e = &q->entry[i];
        if (e->valid)
        {
            if ((addr >= e->addr)
                && (addr < (e->addr + e->max_bytes_to_access)))
            {
                e->req_addr = addr;
                break;
            }
        }
    }
}

void
mem_controller_update_base(MemoryController *m)
{
    PendingMemAccessEntry *e;
    target_ulong req_addr = 0;

    if (!m->mem_access_active)
    {
        if (!cq_empty(&m->dram_dispatch_queue.cq))
        {
            e = &m->dram_dispatch_queue
                     .entry[cq_front(&m->dram_dispatch_queue.cq)];

            m->wrap_around_mode = 0;

            switch (e->type)
            {
                case Write:
                {
                    /* Immediately fire write callback to the waiting pipeline
                     * stage, don't stall the requesting pipeline stage for
                     * writes, but simulate the delay asynchronously */

                    /* Since the pipeline stage never stalls on writes, no need
                     * for wrap around accesses for writes */
                    write_complete(m, e->addr);
                    calculate_seq_access_delays(m, e);
                    break;
                }
                case Read:
                {
                    if (wraparound_access_required(m, e, &req_addr))
                    {
                        m->wrap_around_mode = 1;
                        calculate_wraparound_delays(m, e, req_addr);
                    }
                    else
                    {
                        calculate_seq_access_delays(m, e);
                    }
                    break;
                }
            }

            if (m->wrap_around_mode)
            {
                /* Since we don't read in sequence, this becomes our terminating
                 * condition */
                m->num_bit_vec_read = 0;
            }
            else
            {
                m->cur_bit_vec_id = 0;
            }

            m->last_bit_vec_id = m->max_bus_blks - 1;
            m->current_latency = 1;
            m->mem_access_active = 1;
        }
    }

    if (m->mem_access_active)
    {
        e = &m->dram_dispatch_queue.entry[cq_front(&m->dram_dispatch_queue.cq)];

        if (m->wrap_around_mode)
        {
            do_wrap_around_read(m,e);
        }
        else
        {
            do_seq_read(m,e);
        }
    }
}

void
mem_controller_update_dramsim(MemoryController *m)
{
    PendingMemAccessEntry *e;

    while (!cq_empty(&m->dram_dispatch_queue.cq))
    {
        e = &m->dram_dispatch_queue.entry[cq_front(&m->dram_dispatch_queue.cq)];
        if (dramsim_wrapper_can_add_transaction(e->addr))
        {
            dramsim_wrapper_add_transaction(e->addr, e->type);
            cq_dequeue(&m->dram_dispatch_queue.cq);
        }
        else
        {
            break;
        }
    }

    dramsim_wrapper_update();
}

void
mem_controller_flush_stage_mem_access_queue(StageMemAccessQueue *q)
{
    q->cur_idx = 0;
    q->cur_size = 0;
}

void
mem_controller_flush_dram_queue(MemoryController *m)
{
    cq_reset(&m->dram_dispatch_queue.cq);
}

void
mem_controller_flush_stage_queue_entry_from_dram_queue(
    DRAMDispatchQueue *dram_queue, StageMemAccessQueue *stage_queue)
{
    int i;
    target_ulong first_addr = stage_queue->entry[0].addr;

    if (!cq_empty(&dram_queue->cq))
    {
        if (dram_queue->cq.rear >= dram_queue->cq.front)
        {
            for (i = dram_queue->cq.front; i <= dram_queue->cq.rear; i++)
            {
                if (dram_queue->entry[i].addr == first_addr)
                {
                    dram_queue->cq.rear = i;
                    return;
                }
            }
        }
        else
        {
            for (i = dram_queue->cq.front; i < dram_queue->cq.max_size; i++)
            {
                if (dram_queue->entry[i].addr == first_addr)
                {
                    dram_queue->cq.rear = i;
                    return;
                }
            }

            for (i = 0; i <= dram_queue->cq.rear; i++)
            {
                if (dram_queue->entry[i].addr == first_addr)
                {
                    dram_queue->cq.rear = i;
                    return;
                }
            }
        }
    }
}