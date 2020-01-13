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
#ifndef _MEMORY_CONTROLLER_H_
#define _MEMORY_CONTROLLER_H_

#include <math.h>

#include "../sim_params_stats.h"
#include "circular_queue.h"
#include "dram.h"
#include "memory_controller_utils.h"
#include "riscv_sim_macros.h"

#define FRONTEND_MEM_ACCESS_QUEUE_SIZE 64
#define BACKEND_MEM_ACCESS_QUEUE_SIZE 64
#define DRAM_DISPATCH_QUEUE_SIZE 64

typedef struct DRAMDispatchQueue
{
    CQ cq;
    PendingMemAccessEntry entry[DRAM_DISPATCH_QUEUE_SIZE];
} DRAMDispatchQueue;

typedef struct MemoryController
{
    int mem_model_type;
    int current_latency;
    int max_latency;
    int mem_access_active;
    uint32_t dram_burst_size;
    void (*mem_controller_update_internal)(struct MemoryController *);
    StageMemAccessQueue frontend_mem_access_queue;
    StageMemAccessQueue backend_mem_access_queue;
    DRAMDispatchQueue dram_dispatch_queue;
    Dram *dram;
} MemoryController;

MemoryController *mem_controller_init(const SimParams *p);
void mem_controller_free(MemoryController **m);
void mem_controller_reset(MemoryController *m);
void mem_controller_update_base(MemoryController *m);
void mem_controller_update_dramsim(MemoryController *m);
void mem_controller_update(MemoryController *m);
void mem_controller_set_dram_burst_size(MemoryController *m,
                                        int dram_burst_size);
void mem_controller_flush_stage_mem_access_queue(StageMemAccessQueue *q);
int mem_controller_access_dram(MemoryController *m, target_ulong paddr,
                               int bytes_to_access, MemAccessType op_type,
                               void *p_mem_access_info);
void mem_controller_flush_dram_queue(MemoryController *m);
#endif
