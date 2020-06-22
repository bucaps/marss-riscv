/**
 * Caches
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
#ifndef _CACHE_H_
#define _CACHE_H_

#include "../riscv_sim_typedefs.h"
#include "../utils/evict_policy.h"
#include "../utils/sim_params.h"
#include "memory_controller.h"

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
typedef int (*PFN_WRITE_ALLOC_HANDLER)(const struct Cache *c,
                                       target_ulong paddr, int bytes_to_write,
                                       int set, void *p_mem_access_info,
                                       int priv);
typedef int (*PFN_VICTIM_EVICTION_HANDLER)(const struct Cache *c,
                                           struct CacheBlk *pBlk, int set,
                                           int way, void *p_mem_access_info,
                                           int priv);

/* Cache types */
typedef enum CacheTypes {
    InstructionCache = 0x1,
    DataCache = 0x2,
    SharedCache = 0x3,
} CacheTypes;

/* Cache Levels, add levels as required */
typedef enum CacheLevels {
    L1 = 0x1,
    L2 = 0x2,
    L3 = 0x3,
} CacheLevels;

/* Status of the cache block */
typedef enum BlockStatus {
    Unused = 0x0,
    Valid = 0x1,
} BlockStatus;

/* Block status, used if WriteBack policy is set */
typedef enum BlockDirtyStatus { NonDirty = 0x0, Dirty = 0x1 } BlockDirtyStatus;

/* Cache line allocate policy on write-miss */
typedef enum CacheWriteAllocPolicy {
    WriteAllocate = 0x0,
    WriteNoAllocate = 0x1,
} CacheWriteAllocPolicy;

/* Cache line allocate policy on read-miss */
typedef enum CacheReadAllocPolicy {
    ReadAllocate = 0x0,
    ReadNoAllocate = 0x1,
} CacheReadAllocPolicy;

/* Cache write policy */
typedef enum CacheWritePolicy {
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

/* Cache hierarchy model consists of two levels of physically indexed,
 * physically tagged blocking caches. Level-1 caches include a separate
 * instruction and data cache, whereas Level-2 consists of an optional unified
 * cache. The cache accesses are non-pipelined and follow a non-inclusive
 * non-exclusive design, meaning the contents of the lower level cache are
 * neither strictly inclusive nor exclusive of the higher-level cache. L2 cache
 * can be accessed in parallel by split L1 caches.*/
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
    int read_latency;
    int write_latency;

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

    /* Pointer to the next level cache, if NULL it means it is the last level
     * cache (LLC) */
    struct Cache *next_level_cache;
    CacheStats *stats;
    EvictPolicy *evict_policy;
} Cache;

Cache *cache_init(CacheTypes type, CacheLevels level, int size_kb,
                  int cache_line_size, uint32_t ways, int read_latency,
                  int write_latency, Cache *next_level_cache, int words_per_blk,
                  int evict_policy, CacheWritePolicy write_policy,
                  CacheReadAllocPolicy read_alloc_policy,
                  CacheWriteAllocPolicy write_alloc_policy,
                  MemoryController *mem_controller);
void cache_flush(struct Cache *c);
void cache_reset_stats(struct Cache *c);
const CacheStats *cache_get_stats(const struct Cache *c);
int cache_read(const struct Cache *c, target_ulong paddr, int bytes_to_read,
               void *p_mem_access_info, int priv);
int cache_write(const struct Cache *c, target_ulong paddr, int bytes_to_read,
                void *p_mem_access_info, int priv);
void cache_free(Cache **c);
#endif
