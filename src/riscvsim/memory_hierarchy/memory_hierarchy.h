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
#ifndef _MemoryHierarchy_H_
#define _MemoryHierarchy_H_

#include "../riscv_sim_typedefs.h"
#include "../utils/sim_log.h"
#include "../utils/sim_params.h"
#include "cache.h"
#include "memory_controller.h"

/* Memory hierarchy to simulate the delays, We do not model the actual data
 * in the hierarchy for simplicity, but just the addresses for simulating
 * the delays.*/
typedef struct MemoryHierarchy
{
    MemoryController *mem_controller;
    Cache *icache;
    Cache *dcache;
    Cache *l2_cache;
    Cache *page_walk_cache;
    SimParams *p;

    /* If caches are enabled */
    int cache_line_size;

    /* Pointers are set based on whether caches are enabled or disabled */
    int (*insn_read_delay)(struct MemoryHierarchy *mmu, target_ulong paddr,
                           int bytes, int cpu_stage_id, int priv);
    int (*data_read_delay)(struct MemoryHierarchy *mmu, target_ulong paddr,
                           int bytes, int cpu_stage_id, int priv);
    int (*data_write_delay)(struct MemoryHierarchy *mmu, target_ulong paddr,
                            int bytes, int cpu_stage_id, int priv);

    /* Page table entries read/write delays are simulated via Data Cache, if
     * found, else directly sent to memory */
    int (*pte_read_delay)(struct MemoryHierarchy *mmu, target_ulong paddr,
                          int bytes, int cpu_stage_id, int priv);
    int (*pte_write_delay)(struct MemoryHierarchy *mmu, target_ulong paddr,
                               int bytes, int cpu_stage_id, int priv);
} MemoryHierarchy;

MemoryHierarchy *memory_hierarchy_init(const SimParams *p, SimLog *log);
void memory_hierarchy_free(MemoryHierarchy **mmu);
#endif
