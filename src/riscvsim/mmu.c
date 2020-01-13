/**
 * Memory Management Unit
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
#include <stdio.h>

#include "mmu.h"

/**
 * Returns number of cache lines to be allocated, based on total cache size in
 * KB and size of each line.
 */
static int
get_num_cache_blks_from_cache_size_kb(int size_kb, int num_words_per_line)
{
    return (int)((size_kb * 1024)
                 / (num_words_per_line * sizeof(target_ulong)));
}

MMU *
mmu_init(const SimParams *p)
{
    MMU *mmu;

    mmu = (MMU *)calloc(1, sizeof(MMU));
    assert(mmu);

    /* Setup memory controller */
    PRINT_INIT_MSG("Setting up memory controller");
    mmu->mem_controller = mem_controller_init(p);

    mmu->caches_enabled = p->enable_l1_caches;

    /* Setup caches */
    if (p->enable_l1_caches)
    {
        /* If caches are enabled, set dram burst size to cache line size. If
         * DRAMSim2 is used, its burst size must be equal to cache line size */
        mem_controller_set_dram_burst_size(mmu->mem_controller,
                                           p->words_per_cache_line
                                               * sizeof(target_ulong));

        if (p->enable_l2_cache)
        {
            PRINT_INIT_MSG("Setting up l2-shared cache");
            mmu->l2_cache = create_cache(
                SharedCache, L2,
                get_num_cache_blks_from_cache_size_kb(p->l2_shared_cache_size,
                                                      p->words_per_cache_line),
                p->l2_shared_cache_ways, p->l2_probe_latency, NULL,
                p->words_per_cache_line,
                (CacheEvictionPolicy)p->l2_shared_cache_evict,
                (CacheWritePolicy)p->cache_write_policy,
                (CacheReadAllocPolicy)p->cache_read_allocate_policy,
                (CacheWriteAllocPolicy)p->cache_write_allocate_policy,
                mmu->mem_controller);
        }

        PRINT_INIT_MSG("Setting up l1-instruction cache");
        mmu->icache = create_cache(
            InstructionCache, L1,
            get_num_cache_blks_from_cache_size_kb(p->l1_code_cache_size,
                                                  p->words_per_cache_line),
            p->l1_code_cache_ways, p->l1_code_cache_probe_latency,
            mmu->l2_cache, p->words_per_cache_line,
            (CacheEvictionPolicy)p->l1_code_cache_evict,
            (CacheWritePolicy)p->cache_write_policy,
            (CacheReadAllocPolicy)p->cache_read_allocate_policy,
            (CacheWriteAllocPolicy)p->cache_write_allocate_policy,
            mmu->mem_controller);

        PRINT_INIT_MSG("Setting up l1-data cache");
        mmu->dcache = create_cache(
            DataCache, L1, get_num_cache_blks_from_cache_size_kb(
                               p->l1_data_cache_size, p->words_per_cache_line),
            p->l1_data_cache_ways, p->l1_data_cache_probe_latency,
            mmu->l2_cache, p->words_per_cache_line,
            (CacheEvictionPolicy)p->l1_data_cache_evict,
            (CacheWritePolicy)p->cache_write_policy,
            (CacheReadAllocPolicy)p->cache_read_allocate_policy,
            (CacheWriteAllocPolicy)p->cache_write_allocate_policy,
            mmu->mem_controller);
    }

    return mmu;
}

/**
 * Below functions return latency required for accessing instruction/data
 * through cache hierarchy. If cache operations (cache miss, dirty victim write-back)
 * require memory accesses, these accesses are added to pending memory access queue of
 * the caller stage. The caller stage must wait till its pending memory access
 * queue becomes empty in addition to the latency returned by caches.
 *
 * If caches are disabled, memory operation is initiated directly and caller
 * stage must wait till its pending memory access queue becomes empty.
 */

int
mmu_insn_read(MMU *mmu, target_ulong paddr, int bytes_to_read, int stage_id, int priv)
{
    if (mmu->caches_enabled)
    {
        return cache_read(mmu->icache, paddr, bytes_to_read, (void *)&stage_id, priv);
    }

    mem_controller_access_dram(mmu->mem_controller, paddr, bytes_to_read,
                                      Read, (void *)&stage_id);
    return 1;
}

int
mmu_data_read(MMU *mmu, target_ulong paddr, int bytes_to_read, int stage_id, int priv)
{
    if (mmu->caches_enabled)
    {
        return cache_read(mmu->dcache, paddr, bytes_to_read, (void *)&stage_id, priv);
    }

    mem_controller_access_dram(mmu->mem_controller, paddr, bytes_to_read,
                                      Read, (void *)&stage_id);
    return 1;
}

int
mmu_data_write(MMU *mmu, target_ulong paddr, int bytes_to_write, int stage_id, int priv)
{
    if (mmu->caches_enabled)
    {
        return cache_write(mmu->dcache, paddr, bytes_to_write,
                           (void *)&stage_id, priv);
    }

    mem_controller_access_dram(mmu->mem_controller, paddr,
                                      bytes_to_write, Write, (void *)&stage_id);
    return 1;
}

/**
 * Only used to simulate latency for reading page table entry during hardware
 * page walk.
 */
int
mmu_pte_read(MMU *mmu, target_ulong paddr, int bytes_to_read, int stage_id, int priv)
{
    if (mmu->caches_enabled)
    {
        /* Page table entries are looked up in L1 Data Cache  */
        return cache_read(mmu->dcache, paddr, bytes_to_read,
                          (void *)&stage_id, priv);
    }

    mem_controller_access_dram(mmu->mem_controller, paddr, bytes_to_read,
                                      Read, (void *)&stage_id);
    return 1;
}

/**
 * Only used to simulate latency for writing page table entry during hardware
 * page walk.
 */
int
mmu_pte_write(MMU *mmu, target_ulong paddr, int bytes_to_write, int stage_id,
              int priv)
{
    if (mmu->caches_enabled)
    {
        /* Page table entries are looked up in L1 Data Cache  */
        return cache_write(mmu->dcache, paddr, bytes_to_write,
                           (void *)&stage_id, priv);
    }

    mem_controller_access_dram(mmu->mem_controller, paddr, bytes_to_write,
                               Write, (void *)&stage_id);
    return 1;
}

void
mmu_free(MMU **mmu)
{
    if ((*mmu)->caches_enabled)
    {
        if ((*mmu)->l2_cache)
        {
            delete_cache(&(*mmu)->l2_cache);
        }
        delete_cache(&(*mmu)->dcache);
        delete_cache(&(*mmu)->icache);
    }

    mem_controller_free(&(*mmu)->mem_controller);

    free(*mmu);
    *mmu = NULL;
}