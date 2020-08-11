/**
 * Memory Hierarchy
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

#include "dramsim_wrapper_c_connector.h"
#include "memory_hierarchy.h"

static int
mem_hierarchy_cache_disabled_read(MemoryHierarchy *mem_hierarchy,
                                  target_ulong paddr, int bytes, int stage_id,
                                  int priv)
{
    mem_controller_create_mem_request(mem_hierarchy->mem_controller, paddr,
                                      bytes, MEM_ACCESS_READ,
                                      (void *)&stage_id);
    return 1;
}

static int
mem_hierarchy_cache_disabled_write(MemoryHierarchy *mem_hierarchy,
                                   target_ulong paddr, int bytes, int stage_id,
                                   int priv)
{
    mem_controller_create_mem_request(mem_hierarchy->mem_controller, paddr,
                                      bytes, MEM_ACCESS_WRITE,
                                      (void *)&stage_id);
    return 1;
}

static int
mem_hierarchy_icache_read(MemoryHierarchy *mem_hierarchy, target_ulong paddr,
                          int bytes, int stage_id, int priv)
{
    return cache_read(mem_hierarchy->icache, paddr, bytes, (void *)&stage_id,
                      priv);
}

static int
mem_hierarchy_dcache_read(MemoryHierarchy *mem_hierarchy, target_ulong paddr,
                          int bytes, int stage_id, int priv)
{
    return cache_read(mem_hierarchy->dcache, paddr, bytes, (void *)&stage_id,
                      priv);
}

static int
mem_hierarchy_dcache_write(MemoryHierarchy *mem_hierarchy, target_ulong paddr,
                           int bytes, int stage_id, int priv)
{
    return cache_write(mem_hierarchy->dcache, paddr, bytes, (void *)&stage_id,
                       priv);
}

static void
mem_hierarchy_pte_read(MemoryHierarchy *mem_hierarchy, target_ulong paddr,
                       int bytes, int stage_id, int priv)
{
    mem_controller_create_mem_request_pte(mem_hierarchy->mem_controller, paddr,
                                          bytes, MEM_ACCESS_READ,
                                          (void *)&stage_id);
}

static void
mem_hierarchy_pte_write(MemoryHierarchy *mem_hierarchy, target_ulong paddr,
                        int bytes, int stage_id, int priv)
{
    mem_controller_create_mem_request_pte(mem_hierarchy->mem_controller, paddr,
                                          bytes, MEM_ACCESS_WRITE,
                                          (void *)&stage_id);
}

MemoryHierarchy *
memory_hierarchy_init(const SimParams *p, SimLog *log)
{
    int words_per_cache_line;
    MemoryHierarchy *mem_hierarchy;

    mem_hierarchy = (MemoryHierarchy *)calloc(1, sizeof(MemoryHierarchy));
    assert(mem_hierarchy);
    mem_hierarchy->p = (SimParams *)p;

    /* Setup memory controller */
    mem_hierarchy->mem_controller = mem_controller_init(p);

    /* Setup caches */
    if (p->enable_l1_caches)
    {
        words_per_cache_line = p->cache_line_size /  sizeof(target_ulong);
        mem_hierarchy->cache_line_size = p->cache_line_size;

        /* If caches are enabled, set burst length to cache line size */
        mem_controller_set_burst_length(mem_hierarchy->mem_controller,
                                        mem_hierarchy->cache_line_size);

        /* If DRAMSim3 memory model is used, its burst length must be equal to
         * cache line size */
        if (mem_hierarchy->mem_controller->dram_model_type == MEM_MODEL_DRAMSIM)
        {
            sim_assert((mem_hierarchy->mem_controller->burst_length
                        == dramsim_get_burst_size()),
                       "error: %s at line %d in %s(): %s", __FILE__, __LINE__,
                       __func__, "DRAMSim3 burst length must be equal to "
                                 "MARSS-RISCV cache line size or CPU memory "
                                 "controller burst length");
        }

        if (p->enable_l2_cache)
        {
            sim_log_event_to_file(log, "%s", "Setting up L2-cache");
            mem_hierarchy->l2_cache = cache_init(
                SharedCache, L2, p->l2_shared_cache_size,
                mem_hierarchy->cache_line_size, p->l2_shared_cache_ways,
                p->l2_shared_cache_read_latency,
                p->l2_shared_cache_write_latency, NULL, words_per_cache_line,
                p->l2_shared_cache_evict,
                (CacheWritePolicy)p->cache_write_policy,
                (CacheReadAllocPolicy)p->cache_read_allocate_policy,
                (CacheWriteAllocPolicy)p->cache_write_allocate_policy,
                mem_hierarchy->mem_controller);
        }

        sim_log_event_to_file(log, "%s", "Setting up L1-instruction cache");
        mem_hierarchy->icache = cache_init(
            InstructionCache, L1, p->l1_code_cache_size,
            mem_hierarchy->cache_line_size, p->l1_code_cache_ways,
            p->l1_code_cache_read_latency, 1, mem_hierarchy->l2_cache,
            words_per_cache_line, p->l1_code_cache_evict,
            (CacheWritePolicy)p->cache_write_policy,
            (CacheReadAllocPolicy)p->cache_read_allocate_policy,
            (CacheWriteAllocPolicy)p->cache_write_allocate_policy,
            mem_hierarchy->mem_controller);

        sim_log_event_to_file(log, "%s", "Setting up L1-data cache");
        mem_hierarchy->dcache = cache_init(
            DataCache, L1, p->l1_data_cache_size,
            mem_hierarchy->cache_line_size, p->l1_data_cache_ways,
            p->l1_data_cache_read_latency, p->l1_data_cache_write_latency,
            mem_hierarchy->l2_cache, words_per_cache_line,
            p->l1_data_cache_evict, (CacheWritePolicy)p->cache_write_policy,
            (CacheReadAllocPolicy)p->cache_read_allocate_policy,
            (CacheWriteAllocPolicy)p->cache_write_allocate_policy,
            mem_hierarchy->mem_controller);
    }

    if (p->enable_l1_caches)
    {
        mem_hierarchy->insn_read_delay = &mem_hierarchy_icache_read;
        mem_hierarchy->data_read_delay = &mem_hierarchy_dcache_read;
        mem_hierarchy->data_write_delay = &mem_hierarchy_dcache_write;
    }
    else
    {
        mem_hierarchy->insn_read_delay = &mem_hierarchy_cache_disabled_read;
        mem_hierarchy->data_read_delay = &mem_hierarchy_cache_disabled_read;
        mem_hierarchy->data_write_delay = &mem_hierarchy_cache_disabled_write;
    }

    mem_hierarchy->pte_read_req_send = &mem_hierarchy_pte_read;
    mem_hierarchy->pte_write_req_send = &mem_hierarchy_pte_write;

    return mem_hierarchy;
}

void
memory_hierarchy_free(MemoryHierarchy **mem_hierarchy)
{
    if ((*mem_hierarchy)->p->enable_l1_caches)
    {
        if ((*mem_hierarchy)->l2_cache)
        {
            cache_free(&(*mem_hierarchy)->l2_cache);
        }
        cache_free(&(*mem_hierarchy)->dcache);
        cache_free(&(*mem_hierarchy)->icache);
    }

    mem_controller_free(&(*mem_hierarchy)->mem_controller);

    free(*mem_hierarchy);
    *mem_hierarchy = NULL;
}