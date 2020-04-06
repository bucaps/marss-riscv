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
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cache.h"

#define MAX_STR 32

// static char cache_evict_policy_str[][MAX_STR] = {"Random", "LRU"};
// static char cache_write_policy_str[][MAX_STR] = {"WriteBack", "WriteThrough"};
// static char cache_read_alloc_policy_str[][MAX_STR]
//     = {"ReadAllocate", "ReadNoAllocate"};
// static char cache_write_alloc_policy_str[][MAX_STR]
//     = {"WriteAllocate", "WriteNoAllocate"};

static void
update_tag_address(const Cache *c, uint32_t *pset, CacheBlk **pblk,
                   target_ulong *ptag, target_ulong *paddr,
                   int *pbytes_to_access, int available_bytes)
{
    /* Update pbytes_to_access to account for the available_bytes already
     * accessed */
    *pbytes_to_access = *pbytes_to_access - available_bytes;

    /* Calculate set, tag and physical address to access remaining bytes not
     * present in the current cache line */
    *pset = (*pset + 1) % c->num_sets;
    *pblk = c->blk[*pset];
    *ptag = (*ptag + 1) % c->max_tag_val;
    *paddr = *ptag << c->word_bits;
}

static void
update_status_bits(const Cache *c, int set, int way)
{
    /* Update status bit used by LRU algorithm (Hot & Cold bit)*/
    int update_bit = 0;
    int comp = floor(c->num_ways / 2);

    while (update_bit < (c->num_ways - 1))
    {
        if (way <= comp)
        {
            /* Status bit is in first half */
            c->status_bits[set][update_bit] = 0;
            update_bit = ((update_bit + 1) * 2) - 1;
            comp = floor(comp / 2);
        }
        else
        {
            /* Status bit is in second half */
            c->status_bits[set][update_bit] = 1;
            update_bit = ((update_bit + 1) * 2);
            comp = floor((comp + c->num_ways - 1) / 2);
        }
    }
}

static int
get_lru_victim_index(const Cache *c, int set)
{
    int check_bit = 0;
    int lower_bound = 0;
    int upper_bound = c->num_ways - 1;

    while (check_bit < (c->num_ways - 1))
    {
        if (c->status_bits[set][check_bit])
        {
            /* First half is cold */
            check_bit = ((check_bit + 1) * 2) - 1;
            upper_bound = floor((lower_bound + upper_bound) / 2);
        }
        else
        {
            /* Second half is cold */
            check_bit = ((check_bit + 1) * 2);
            lower_bound = floor((lower_bound + upper_bound) / 2) + 1;
        }
    }

    return lower_bound;
}

static int
get_random_victim_index(const Cache *c, int set)
{
    return rand() % c->num_ways;
}

static int
read_data_internal(const Cache *c, target_ulong paddr, int bytes_to_read,
                   void *p_mem_access_info, int priv)
{
    /* Read from next-level cache if present, otherwise from memory */
    if (NULL != c->next_level_cache)
    {
        return cache_read(c->next_level_cache, paddr, bytes_to_read,
                          p_mem_access_info, priv);
    }

    return mem_controller_access_dram(c->mem_controller, paddr, bytes_to_read, Read,
                      p_mem_access_info);
}

static int
read_allocate_handler(const Cache *c, target_ulong paddr, int bytes_to_read,
                      int set, void *p_mem_access_info, int priv)
{
    target_ulong tag;
    int latency = 0;
    CacheBlk *blk = c->blk[set];
    /* Select victim using the set policy */
    int victim = (*c->pfn_get_victim_index)(c, set);

    /* As we want to allocate complete line, bytes_to_read = cache line width in
     * bytes and paddr is the address of the zeroth byte in the cache line */
    paddr = paddr & c->tag_bits_mask;
    bytes_to_read = (WORD_SIZE * c->max_words_per_blk);
    tag = paddr >> (c->word_bits);

    /* Handle victim eviction according to set policy */
    latency += (*c->pfn_victim_evict_handler)(c, &(blk[victim]), set, victim,
                                              p_mem_access_info, priv);

    /* Read the line contents into the cache line selected as victim and adjust
     * the latency accordingly */
    latency += read_data_internal(c, paddr, bytes_to_read, p_mem_access_info, priv);

    blk[victim].tag = tag;
    blk[victim].status = Valid;

    /* Update the status bits used for LRU victim selection policy */
    update_status_bits(c, set, victim);

    return latency;
}

static int
read_no_allocate_handler(const Cache *c, target_ulong paddr, int bytes_to_read,
                         int set, void *p_mem_access_info, int priv)
{
    /* Read the line contents and adjust the latency accordingly */
    return read_data_internal(c, paddr, bytes_to_read, p_mem_access_info, priv);
}

int
cache_read(const Cache *c, target_ulong paddr, int bytes_to_read,
           void *p_mem_access_info, int priv)
{
    int i;
    uint32_t start_byte = (paddr & ((1 << c->word_bits) - 1));
    uint32_t set = (paddr >> c->word_bits) & ((1 << c->set_bits) - 1);
    target_ulong tag = paddr >> (c->word_bits);
    int latency = c->read_latency;
    int available_bytes = 0;
    int cache_miss_updated = 0;
    CacheBlk *blk = c->blk[set];

    c->stats[priv].total_read_cnt++;

    while (bytes_to_read > 0)
    {
        for (i = 0; i < c->num_ways; ++i)
        {
            if ((blk[i].tag == tag) && (blk[i].status == Valid))
            {
                /* Tag-match: Physical address is present in the cache */
                update_status_bits(c, set, i);
                if ((start_byte + bytes_to_read)
                    <= (c->max_words_per_blk * WORD_SIZE))
                {
                    /* All required bytes are present in the cache */
                    return latency;
                }

                /* Required bytes are possibly split across 2 cache lines */
                /* Calculate the bytes available to read in the current cache line */
                available_bytes
                    = ((c->max_words_per_blk * WORD_SIZE) - start_byte);

                /* Adjust the remaining bytes to read, tag, set and physical
                 * address */
                update_tag_address(c, &set, &blk, &tag, &paddr, &bytes_to_read,
                                   available_bytes);
                break;
            }
        }

        if (i < c->num_ways)
        {
            /* It means above loop was broken because required bytes were not
             * found in the same cache line. So, start looking again for new tag
             * and address */
            continue;
        }

        if (!cache_miss_updated)
        {
            c->stats[priv].read_miss_cnt++;
            cache_miss_updated = 1;
        }

        if ((start_byte + bytes_to_read) <= (c->max_words_per_blk * WORD_SIZE))
        {
            /* All remaining bytes can be allocated in the same line */
            latency += (*c->pfn_read_alloc_handler)(c, paddr, bytes_to_read,
                                                    set, p_mem_access_info, priv);
            break;
        }
        else
        {
            /* All remaining bytes can not be allocated in the same line */
            available_bytes = ((c->max_words_per_blk * WORD_SIZE) - start_byte);

            /* Read the required line contents and allocate the cache line using
             * set policy */
            latency += (*c->pfn_read_alloc_handler)(c, paddr, available_bytes,
                                                    set, p_mem_access_info, priv);

            /* Adjust the remaining bytes to read, tag, set and physical
             * address */
            update_tag_address(c, &set, &blk, &tag, &paddr, &bytes_to_read,
                               available_bytes);
            continue;
        }
    }

    return latency;
}

static int
writeback_handler(const Cache *c, target_ulong paddr, int bytes_to_write,
                  int set, int way, void *p_mem_access_info, int priv)
{
    /* Just update dirty bit and status bits, no need to write to next cache */
    CacheBlk *blk = c->blk[set];
    blk[way].dirty = Dirty;

    update_status_bits(c, set, way);
    return 0;
}

static int
writethrough_handler(const Cache *c, target_ulong paddr, int bytes_to_write,
                     int set, int way, void *p_mem_access_info, int priv)
{
    /* Update the dirty bit and status bits */
    CacheBlk *blk = c->blk[set];
    blk[way].dirty = Dirty;
    update_status_bits(c, set, way);

    /* Propagate write to next-level cache if available, memory otherwise */
    if (NULL != c->next_level_cache)
    {
        return cache_write(c->next_level_cache, paddr, bytes_to_write,
                           p_mem_access_info, priv);
    }

    return mem_controller_access_dram(c->mem_controller, paddr, bytes_to_write, Write,
                      p_mem_access_info);
}

static int
write_allocate_handler(const Cache *c, target_ulong paddr, int bytes_to_write,
                       int set, void *p_mem_access_info, int priv)
{
    int latency = 0;
    CacheBlk *blk = c->blk[set];

    /* As we want to allocate complete line, bytes_to_read = cache line width in
     * bytes and paddr is the address of the zeroth byte in the cache line */
    int bytes_to_read = (WORD_SIZE * c->max_words_per_blk);

    /* Zero out the word bits to create address of first byte in the cache line
     * as we have to read contents for the whole cache line */
    target_ulong new_paddr = paddr & c->tag_bits_mask;
    target_ulong tag = paddr >> (c->word_bits);

    /* Select victim using the set policy */
    int victim = (*c->pfn_get_victim_index)(c, set);
    /* Handle victim eviction according to set policy */
    latency += (*c->pfn_victim_evict_handler)(c, &(blk[victim]), set, victim,
                                              p_mem_access_info, priv);

    /* Read the line contents into the cache line selected as victim and adjust
     * the latency accordingly */
    latency
        += read_data_internal(c, new_paddr, bytes_to_read, p_mem_access_info, priv);

    blk[victim].tag = tag;
    blk[victim].status = Valid;

    /* Handle the cache write using underlying write policy */
    latency += (*c->pfn_write_handler)(c, paddr, bytes_to_write, set, victim,
                                       p_mem_access_info, priv);

    return latency;
}

static int
write_no_allocate_handler(const Cache *c, target_ulong paddr,
                          int bytes_to_write, int set, void *p_mem_access_info, int priv)
{
    /* Do not allocate cache line but propagate write to next-level cache if
     * available, memory otherwise */
    if (NULL != c->next_level_cache)
    {
        return cache_write(c->next_level_cache, paddr, bytes_to_write,
                           p_mem_access_info, priv);
    }

    return mem_controller_access_dram(c->mem_controller, paddr, bytes_to_write, Write,
                      p_mem_access_info);
}

static int
writeback_victim_evict_handler(const Cache *c, CacheBlk *pBlk, int set, int way,
                               void *p_mem_access_info, int priv)
{
    int latency = 0;

    if (NULL != pBlk)
    {
        /* If the cache line contents are dirty, write to next level cache if
         * present, otherwise write to memory */
        if (Valid == pBlk->status && Dirty == pBlk->dirty)
        {
            if (NULL != c->next_level_cache)
            {
                latency += cache_write(
                    c->next_level_cache, (pBlk->tag << c->word_bits),
                    (WORD_SIZE * c->max_words_per_blk), p_mem_access_info, priv);
            }
            else
            {
                latency
                    += mem_controller_access_dram(c->mem_controller, pBlk->tag << c->word_bits,
                                  (WORD_SIZE * c->max_words_per_blk), Write,
                                  p_mem_access_info);
            }
        }
        memset(pBlk, 0, sizeof(CacheBlk));
    }

    return latency;
}

static int
writethrough_victim_evict_handler(const Cache *c, CacheBlk *pBlk, int set,
                                  int way, void *p_mem_access_info, int priv)
{
    /* No need to write to next level cache or memory as we have already written
     * it */
    memset(pBlk, 0, sizeof(CacheBlk));
    return 0;
}

int
cache_write(const Cache *c, target_ulong paddr, int bytes_to_write,
            void *p_mem_access_info, int priv)
{
    int i;
    uint32_t start_byte = (paddr & ((1 << c->word_bits) - 1));
    uint32_t set = (paddr >> c->word_bits) & ((1 << c->set_bits) - 1);
    target_ulong tag = paddr >> (c->word_bits);
    int latency = c->write_latency;
    int available_bytes = 0;
    int cache_miss_updated = 0;
    CacheBlk *blk = c->blk[set];

    c->stats[priv].total_write_cnt++;

    while (bytes_to_write > 0)
    {
        for (i = 0; i < c->num_ways; ++i)
        {
            if ((blk[i].tag == tag) && (blk[i].status == Valid))
            {
                /* Tag-match: Physical address is present in the cache */
                if ((start_byte + bytes_to_write)
                    <= (c->max_words_per_blk * WORD_SIZE))
                {
                    /* All required bytes are present in the cache */
                    latency += (*c->pfn_write_handler)(
                        c, paddr, bytes_to_write, set, i, p_mem_access_info, priv);
                    return latency;
                }

                /* Required bytes are possibly split across 2 cache lines */
                available_bytes
                    = ((c->max_words_per_blk * WORD_SIZE) - start_byte);

                /* Write the cache line with the bytes_to_write set to actual
                 * bytes found in this cache line */
                latency += (*c->pfn_write_handler)(c, paddr, available_bytes,
                                                   set, i, p_mem_access_info, priv);

                /* Adjust the remaining bytes to write */
                update_tag_address(c, &set, &blk, &tag, &paddr, &bytes_to_write,
                                   available_bytes);
                break;
            }
        }

        if (i < c->num_ways)
        {
            /* It means above loop was broken because required bytes were not
             * found in the same cache line. So, start looking again for new tag
             * and address */
            continue;
        }

        /* Cache Write Miss */
        if (!cache_miss_updated)
        {
            c->stats[priv].write_miss_cnt++;
            cache_miss_updated = 1;
        }

        /* Call write allocator function */
        if ((start_byte + bytes_to_write) <= (c->max_words_per_blk * WORD_SIZE))
        {
            latency += (*c->pfn_write_alloc_handler)(c, paddr, bytes_to_write,
                                                     set, p_mem_access_info, priv);
            break;
        }
        else
        {
            available_bytes = ((c->max_words_per_blk * WORD_SIZE) - start_byte);

            /* Write the cache line with the bytes_to_write set to actual bytes
             * found in this cache line and adjust the latency */
            latency += (*c->pfn_write_alloc_handler)(
                c, paddr, ((c->max_words_per_blk * WORD_SIZE) - start_byte),
                set, p_mem_access_info, priv);

            /* Adjust the remaining bytes to write */
            update_tag_address(c, &set, &blk, &tag, &paddr, &bytes_to_write,
                               available_bytes);
            continue;
        }
    }

    return latency;
}

void
cache_flush(const Cache *c)
{
    int i;

    for (i = 0; i < c->num_sets; ++i)
    {
        memset((void *)c->blk[i], 0, sizeof(CacheBlk) * c->num_ways);
        memset((void *)c->status_bits[i], 0, sizeof(int) * (c->num_ways));
    }
}

const CacheStats *const
get_cache_stats(Cache *c)
{
    return (const CacheStats *const)(c->stats);
}

void
reset_cache_stats(Cache *c)
{
    memset((void *)c->stats, 0, NUM_MAX_PRV_LEVELS * sizeof(CacheStats));
}

Cache *
create_cache(CacheTypes type, CacheLevels level, uint32_t blks, uint32_t ways,
             int read_latency, int write_latency, Cache *next_level_cache, int words_per_blk,
             CacheEvictionPolicy evict_policy, CacheWritePolicy write_policy,
             CacheReadAllocPolicy read_alloc_policy,
             CacheWriteAllocPolicy write_alloc_policy,
             MemoryController *mem_controller)
{
    int i;
    Cache *c = (Cache *)malloc(sizeof(Cache));

    c->type = type;
    c->level = level;
    c->num_blks = blks;
    c->num_ways = ways;
    c->num_sets = (blks / ways);

    /* Allocate memory for all cache line pointers */
    c->blk = (CacheBlk **)calloc(c->num_sets, sizeof(CacheBlk *));
    assert(c->blk);

    /* Allocate memory to hold status bits */
    c->status_bits = (int **)calloc(c->num_sets, sizeof(int *));
    assert(c->status_bits);

    /* Allocate memory for cache blocks */
    for (i = 0; i < c->num_sets; ++i)
    {
        c->blk[i] = (CacheBlk *)calloc(c->num_ways, sizeof(CacheBlk));
        assert(c->blk[i]);

        c->status_bits[i] = (int *)calloc(c->num_ways, sizeof(int));
        assert(c->status_bits[i]);
    }

    c->max_words_per_blk = words_per_blk;

    /* No. of bits required to represent byte offset within the cache line */
    c->word_bits = GET_NUM_BITS(WORD_SIZE * c->max_words_per_blk);
    /* No. of bits required to represent target set in the physical address */
    c->set_bits = GET_NUM_BITS(c->num_sets);

    /* Size of cache in KBs */
    c->size = (c->num_blks * c->max_words_per_blk * WORD_SIZE) / 1000;

    /* Included set bits again in the tag bits */
    c->tag_bits = (sizeof(target_ulong) * 8) - c->word_bits;
    c->tag_bits_mask = (target_ulong)(pow(2, sizeof(target_ulong) * 8) - 1)
                       << c->word_bits;
    c->max_tag_val = (1 << c->tag_bits);

    c->read_latency = read_latency;
    c->write_latency = write_latency;
    c->mem_controller = mem_controller;

    c->next_level_cache = next_level_cache;
    if (NULL == c->next_level_cache)
    {
        assert(c->mem_controller);
    }

    c->stats = (CacheStats *)calloc(NUM_MAX_PRV_LEVELS, sizeof(CacheStats));
    assert(c->stats);

    c->cache_evict_policy = evict_policy;
    c->cache_write_policy = write_policy;
    c->cache_read_alloc_policy = read_alloc_policy;
    c->cache_write_alloc_policy = write_alloc_policy;

    /* Set eviction function pointer */
    switch (evict_policy)
    {
        case Random:
        {
            c->pfn_get_victim_index = &get_random_victim_index;
            break;
        }

        case LRU:
        {
            c->pfn_get_victim_index = &get_lru_victim_index;
            break;
        }
    }

    /* Set write policy handler function pointer */
    switch (write_policy)
    {
        case WriteBack:
        {
            c->pfn_write_handler = &writeback_handler;
            c->pfn_victim_evict_handler = &writeback_victim_evict_handler;
            break;
        }

        case WriteThrough:
        {
            c->pfn_write_handler = &writethrough_handler;
            c->pfn_victim_evict_handler = &writethrough_victim_evict_handler;
            break;
        }
    }

    /* Set cache line allocation handler function pointer on read-miss */
    switch (read_alloc_policy)
    {
        case ReadAllocate:
        {
            c->pfn_read_alloc_handler = &read_allocate_handler;
            break;
        }

        case ReadNoAllocate:
        {
            c->pfn_read_alloc_handler = &read_no_allocate_handler;
            break;
        }
    }

    /* Set cache line allocation handler function pointer on write-miss */
    switch (write_alloc_policy)
    {
        case WriteAllocate:
        {
            c->pfn_write_alloc_handler = &write_allocate_handler;
            break;
        }

        case WriteNoAllocate:
        {
            c->pfn_write_alloc_handler = &write_no_allocate_handler;
            break;
        }
    }

    return c;
}

void
delete_cache(Cache **c)
{
    int i;

    for (i = 0; i < (*c)->num_sets; ++i)
    {
        free((*c)->blk[i]);
        (*c)->blk[i] = NULL;
        free((*c)->status_bits[i]);
        (*c)->status_bits[i] = NULL;
    }
    free((*c)->blk);
    (*c)->blk = NULL;
    free((*c)->status_bits);
    (*c)->status_bits = NULL;
    free(*c);
    *c = NULL;
}

void
print_cache_config(const Cache *const c)
{
    fprintf(stderr, "\n");
    fprintf(stderr, "Cache Configurations\n");
    fprintf(stderr, "Cache Type: %u\n", c->type);
    fprintf(stderr, "Cache Level: %u\n", c->level);
    fprintf(stderr, "Total No. of Blocks: %u\n", c->num_blks);
    fprintf(stderr, "Total No. of Ways: %u\n", c->num_ways);
    fprintf(stderr, "Total No. of Sets: %u\n", c->num_sets);
    fprintf(stderr, "Words per Block: %u\n", c->max_words_per_blk);
    fprintf(stderr, "Word Size: %lu\n", WORD_SIZE);
    fprintf(stderr, "Total Cache Size: %lu KB\n", c->size);
    fprintf(stderr, "Offset(word) bits: %u\n", c->word_bits);
    fprintf(stderr, "Set bits: %u\n", c->set_bits);
    fprintf(stderr, "Tag bits: %u\n", c->tag_bits);
    fprintf(stderr, "Eviction Policy: [%d] %s\n", c->cache_evict_policy,
            cache_evict_str[c->cache_evict_policy]);
    fprintf(stderr, "Write Policy: [%d] %s\n", c->cache_write_policy,
            cache_wp_str[c->cache_write_policy]);
    fprintf(stderr, "Read Alloc Policy: [%d] %s\n", c->cache_read_alloc_policy,
            cache_ra_str[c->cache_read_alloc_policy]);
    fprintf(stderr, "Write Alloc Policy: [%d] %s\n",
            c->cache_write_alloc_policy,
            cache_wa_str[c->cache_write_alloc_policy]);
    fprintf(stderr, "\n");
    fprintf(stderr, "\n");
}
