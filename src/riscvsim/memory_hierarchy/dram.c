/**
 * DRAM model
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

#include "../utils/sim_log.h"
#include "dram.h"
#include "dramsim_wrapper_c_connector.h"
#include "ramulator_wrapper_c_connector.h"

static void
dram_log_config(const Dram *d, const SimParams *p)
{
    sim_log_event_to_file(sim_log, "%s", "Setting up base dram");
    sim_log_param_to_file(sim_log, "%s: %s", "dram_model_type",
                          dram_model_type_str[p->dram_model_type]);
    switch (p->dram_model_type)
    {
        case MEM_MODEL_BASE:
        {
            sim_log_param_to_file(sim_log, "%s: %d cycle(s)",
                                  "mem_access_latency", d->mem_access_latency);
            break;
        }
        case MEM_MODEL_DRAMSIM:
        {
            sim_log_param_to_file(sim_log, "%s: %s)", "config_file",
                                  p->dramsim_config_file);
            sim_log_param_to_file(sim_log, "%s: %s", "output-directory",
                                  p->sim_file_path);
            break;
        }
        case MEM_MODEL_RAMULATOR:
        {
            sim_log_param_to_file(sim_log, "%s: %s)", "config_file",
                                  p->ramulator_config_file);
            sim_log_param_to_file(sim_log, "%s: %s", "output-directory",
                                  p->sim_file_path);
            break;
        }
    }
}

/* Base DRAM model keeps track of the physical page number of the latest
 * request processed. Any subsequent accesses to the same physical page
 * occupies a lower delay, which is roughly 60 percent of the fixed
 * mem_access_latency. */
static int
base_dram_model_get_max_clock_cycles(Dram *d, PendingMemAccessEntry *e)
{
    uint64_t current_page_num;
    int max_clock_cycles = d->mem_access_latency;

    /* Remove page offset to get current page number, page size is always 4KB */
    current_page_num = e->addr >> 12;

    if (d->last_accessed_page_num == current_page_num)
    {
        /* Page hit */
        max_clock_cycles *= 0.6;
    }
    else
    {
        /* Page miss */
        d->last_accessed_page_num = current_page_num;
    }

    return max_clock_cycles;
}

#define DRAMSIM3_RAM_BASE_ADDR 0x0
#define TINYEMU_RAM_BASE_ADDR 0x80000000

/* In TinyEMU, physical memory from 0x0 to 0x80000000 is allocated for the
 * TinyEMU devices. Guest RAM starts from address  0x80000000. Hence all the
 * physical addresses obtained after TLB lookup for guest RAM are above
 * 0x80000000. So before sending the address to DRAMsim3/ramulator (which starts
 * from 0x0), convert the given address. */
static target_ulong
get_tinyemu_ram_addr_from_zero(target_ulong tinyemu_ram_addr)
{
    target_ulong offset;

    if (tinyemu_ram_addr >= TINYEMU_RAM_BASE_ADDR)
    {
        offset = tinyemu_ram_addr - TINYEMU_RAM_BASE_ADDR;
        tinyemu_ram_addr = DRAMSIM3_RAM_BASE_ADDR + offset;
    }

    return tinyemu_ram_addr;
}

static int
dramsim_get_max_clock_cycles(Dram *d, PendingMemAccessEntry *e)
{
    uint64_t max_clock_cycles;
    target_ulong ram_addr = get_tinyemu_ram_addr_from_zero(e->addr);
    assert(dramsim_wrapper_can_add_transaction(ram_addr, e->type));
    dramsim_wrapper_add_transaction(ram_addr, e->type);
    max_clock_cycles = dramsim_wrapper_get_max_clock_cycles();

    if (max_clock_cycles >= 1000)
    {
        sim_log_event(sim_log,
                      "possible dramsim3 block detected: %lu cycle(s) reported",
                      max_clock_cycles);
    }

    return max_clock_cycles;
}

static int
ramulator_get_max_clock_cycles(Dram *d, PendingMemAccessEntry *e)
{
    int max_clock_cycles;
    target_ulong ram_addr = get_tinyemu_ram_addr_from_zero(e->addr);
    assert(ramulator_wrapper_add_transaction(ram_addr, e->type));
    max_clock_cycles = ramulator_wrapper_get_max_clock_cycles();

    if (max_clock_cycles >= 1000)
    {
        sim_log_event(sim_log,
                      "possible ramulator block detected: %d cycle(s) reported",
                      max_clock_cycles);
    }

    return max_clock_cycles;
}

static void
read_complete_callback(target_ulong addr, StageMemAccessQueue *f,
                       StageMemAccessQueue *b)
{
    int i;

    for (i = 0; i < f->cur_idx; ++i)
    {
        if ((f->entry[i].valid) && (f->entry[i].addr == addr)
            && (f->entry[i].type == MEM_ACCESS_READ))
        {
            f->entry[i].valid = FALSE;
            --f->cur_size;
            return;
        }
    }

    for (i = 0; i < b->cur_idx; ++i)
    {
        if ((b->entry[i].valid) && (b->entry[i].addr == addr)
            && (b->entry[i].type == MEM_ACCESS_READ))
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
            && (f->entry[i].type == MEM_ACCESS_WRITE))
        {
            f->entry[i].valid = FALSE;
            --f->cur_size;
            return;
        }
    }

    for (i = 0; i < b->cur_idx; ++i)
    {
        if ((b->entry[i].valid) && (b->entry[i].addr == addr)
            && (b->entry[i].type == MEM_ACCESS_WRITE))
        {
            b->entry[i].valid = FALSE;
            --b->cur_size;
            return;
        }
    }
}

int
dram_can_accept_request(const Dram *d)
{
    return !d->mem_access_active;
}

void
dram_send_request(Dram *d, PendingMemAccessEntry *e)
{
    d->max_clock_cycles = d->get_max_clock_cycles_for_request(d, e);
    assert(d->max_clock_cycles);

    /* Send a write complete callback to the calling pipeline stage as
     * we don't want the pipeline stage to wait for write to complete.
     * But, simulate this write delay asynchronously via the memory
     * controller */
    if (e->type == MEM_ACCESS_WRITE)
    {
        write_complete_callback(e->addr, d->frontend_mem_access_queue,
                                d->backend_mem_access_queue);
    }

    d->active_mem_request = e;
    d->mem_access_active = TRUE;
    d->elasped_clock_cycles = 1;
}

int
dram_clock(Dram *d)
{
    if (d->mem_access_active)
    {
        if (d->elasped_clock_cycles == d->max_clock_cycles)
        {
            d->mem_access_active = FALSE;
            d->max_clock_cycles = 0;
            d->elasped_clock_cycles = 0;

            if (d->active_mem_request->type == MEM_ACCESS_READ)
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

void
dram_reset(Dram *d)
{
    d->elasped_clock_cycles = 0;
    d->max_clock_cycles = 0;
    d->mem_access_active = FALSE;
    d->active_mem_request = NULL;

    /* For base DRAM model */
    d->last_accessed_page_num = 0;
}

Dram *
dram_create(const SimParams *p, StageMemAccessQueue *f,
                 StageMemAccessQueue *b)
{
    Dram *d;

    d = calloc(1, sizeof(Dram));
    assert(d);

    d->mem_access_latency = p->mem_access_latency;
    d->frontend_mem_access_queue = f;
    d->backend_mem_access_queue = b;

    d->dram_model_type = p->dram_model_type;

    switch (d->dram_model_type)
    {
        case MEM_MODEL_BASE:
        {
            d->get_max_clock_cycles_for_request
                = &base_dram_model_get_max_clock_cycles;
            break;
        }
        case MEM_MODEL_DRAMSIM:
        {
            dramsim_wrapper_init(p->dramsim_config_file, p->sim_file_path);
            d->get_max_clock_cycles_for_request = &dramsim_get_max_clock_cycles;
            break;
        }
        case MEM_MODEL_RAMULATOR:
        {
            ramulator_wrapper_init(p->ramulator_config_file,
                                   p->cache_line_size);
            d->get_max_clock_cycles_for_request
                = &ramulator_get_max_clock_cycles;
            break;
        }
    }

    dram_reset(d);
    dram_log_config(d, p);
    return d;
}

void
dram_free(Dram **d)
{
    switch ((*d)->dram_model_type)
    {
        case MEM_MODEL_BASE:
        {
            break;
        }
        case MEM_MODEL_DRAMSIM:
        {
            dramsim_wrapper_destroy();
            break;
        }
        case MEM_MODEL_RAMULATOR:
        {
            ramulator_wrapper_destroy();
            break;
        }
    }
    free(*d);
}
