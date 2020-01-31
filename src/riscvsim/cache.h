/**
 * Caches
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
#ifndef _CACHE_H_
#define _CACHE_H_

#include "memory_controller.h"
#include "riscv_sim_typedefs.h"
#include "sim_params_stats.h"

/* Word size in the target architecture */
#define WORD_SIZE (sizeof(target_ulong))

struct CacheBlk;
struct Cache;

typedef int (*PFN_GET_VICTIM_INDEX)(const struct Cache *c, int set);
typedef int (*PFN_READ_ALLOC_HANDLER)(const struct Cache *c, target_ulong paddr,
                                      int bytes_to_read, int set,
                                      void *p_mem_access_info, int priv);
typedef int (*PFN_WRITE_HANDLER)(const struct Cache *c, target_ulong paddr,
                                 int bytes_to_write, int set, int way,
                                 void *p_mem_access_info, int priv);
typedef int (*PFN_WRITE_ALLOC_HANDLER)(const struct Cache *c, target_ulong paddr,
                                       int bytes_to_write, int set,
                                       void *p_mem_access_info, int priv);
typedef int (*PFN_VICTIM_EVICTION_HANDLER)(const struct Cache *c, struct CacheBlk *pBlk,
                                           int set, int way,
                                           void *p_mem_access_info, int priv);

/* Cache types */
typedef enum CacheTypes
{
    InstructionCache = 0x1,
    DataCache = 0x2,
    SharedCache = 0x3,
} CacheTypes;

/* Cache Levels, add levels as required */
typedef enum CacheLevels
{
    L1 = 0x1,
    L2 = 0x2,
    L3 = 0x3,
} CacheLevels;

/* Cache victim selection policy */
typedef enum CacheEvictionPolicy
{
    Random, /* Replace a random block */
    LRU,    /* Replace least recently used block (perfect LRU) */
} CacheEvictionPolicy;

/* Status of the cache block */
typedef enum BlockStatus
{
    Unused = 0x0,
    Valid = 0x1,
} BlockStatus;

/* Block status, used if WriteBack policy is set */
typedef enum BlockDirtyStatus
{
    NonDirty = 0x0,
    Dirty = 0x1
} BlockDirtyStatus;

/* Cache line allocate policy on write-miss */
typedef enum CacheWriteAllocPolicy
{
    WriteAllocate = 0x0,
    WriteNoAllocate = 0x1,
} CacheWriteAllocPolicy;

/* Cache line allocate policy on read-miss */
typedef enum CacheReadAllocPolicy
{
    ReadAllocate = 0x0,
    ReadNoAllocate = 0x1,
} CacheReadAllocPolicy;

/* Cache write policy */
typedef enum CacheWritePolicy
{
    WriteBack = 0x0,
    WriteThrough = 0x1,
} CacheWritePolicy;

/* Statistical counters for cache */
typedef struct CacheStats
{
    uint64_t total_read_cnt;
    uint64_t total_write_cnt;
    uint64_t read_miss_cnt;
    uint64_t write_miss_cnt;
} CacheStats;

/* Single cache line */
typedef struct CacheBlk
{
    uint8_t status;
    uint8_t dirty;
    target_ulong tag;
} CacheBlk;

/* Cache object storing cache blocks, status bits and policies to be used */
typedef struct Cache
{
    int level;
    int type;

    int num_ways;
    int num_blks;
    int num_sets;
    int tag_bits;
    int set_bits;
    int word_bits;
    uint64_t size;

    int max_words_per_blk;
    target_ulong max_tag_val;

    target_ulong tag_bits_mask;
    int probe_latency;

    CacheEvictionPolicy cache_evict_policy;

    CacheWritePolicy cache_write_policy;
    CacheReadAllocPolicy cache_read_alloc_policy;
    CacheWriteAllocPolicy cache_write_alloc_policy;

    MemoryController *mem_controller;

    /* Function pointer respective to
       eviction policy used(LRU/Random) */
    PFN_GET_VICTIM_INDEX pfn_get_victim_index;
    /* Function pointer respective cache line allocation policy
       used on read-miss(Read-Allocate/Read-No-Allocate) */
    PFN_READ_ALLOC_HANDLER pfn_read_alloc_handler;
    /* Function pointer respective to write
       policy used(write-back/write-through) */
    PFN_WRITE_HANDLER pfn_write_handler;
    /* Function pointer respective cache line allocation policy
       used on write-miss(Read-Allocate/Read-No-Allocate) */
    PFN_WRITE_ALLOC_HANDLER pfn_write_alloc_handler;
    /* Function pointer respective to eviction policy
       used(Write-Allocate/Write-No-Allocate) */
    PFN_VICTIM_EVICTION_HANDLER pfn_victim_evict_handler;

    CacheBlk **blk;
    int **status_bits;
    /* Pointer to the next level cache, if NULL it means it is the last level
     * cache (LLC) */
    struct Cache *next_level_cache;
    CacheStats *stats;
} Cache;

int cache_read(const Cache *c, target_ulong paddr, int bytes_to_read,
               void *p_mem_access_info, int priv);
int cache_write(const Cache *c, target_ulong paddr, int bytes_to_read,
                void *p_mem_access_info, int priv);

Cache *create_cache(CacheTypes type, CacheLevels level, uint32_t blks,
                    uint32_t ways, int probe_latency, Cache *next_level_cache,
                    int words_per_blk, CacheEvictionPolicy evict_policy,
                    CacheWritePolicy write_policy,
                    CacheReadAllocPolicy read_alloc_policy,
                    CacheWriteAllocPolicy write_alloc_policy,
                    MemoryController *mem_controller);
void delete_cache(Cache **c);

const CacheStats *const get_cache_stats(Cache *c);
void reset_cache_stats(Cache *c);
void cache_flush(const Cache *c);
void print_cache_config(const Cache *const c);

#endif
