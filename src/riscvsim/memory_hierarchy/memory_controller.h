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
#ifndef _MEMORY_CONTROLLER_H_
#define _MEMORY_CONTROLLER_H_

#include <math.h>

#include "../../cutils.h"
#include "../riscv_sim_typedefs.h"
#include "../utils/circular_queue.h"
#include "../utils/sim_params.h"
#include "dram.h"
#include "memory_controller_utils.h"

#define FRONTEND_MEM_ACCESS_QUEUE_SIZE 64
#define BACKEND_MEM_ACCESS_QUEUE_SIZE 64
#define MEM_REQUEST_QUEUE_SIZE 64

typedef struct MemRequestQueue
{
    CQ cq;
    PendingMemAccessEntry entry[MEM_REQUEST_QUEUE_SIZE];
} MemRequestQueue;

typedef struct MemoryController
{
    /* Type of DRAM model: base or dramsim3 */
    int dram_model_type;

    /* Memory controller burst length in bytes (or cache line size) */
    int burst_length;

    /* These queues are used to control the stall on fetch and memory CPU
     * pipeline stages */
    StageMemAccessQueue frontend_mem_access_queue;
    StageMemAccessQueue backend_mem_access_queue;

    /* A single FIFO queue known as mem_request_queue comprising all the pending
     * memory access requests */
    MemRequestQueue mem_request_queue;

    Dram *dram;
} MemoryController;

MemoryController *mem_controller_init(const SimParams *p);
void mem_controller_free(MemoryController **m);
void mem_controller_reset(MemoryController *m);
void mem_controller_clock(MemoryController *m);
void mem_controller_reset_cpu_stage_queue(StageMemAccessQueue *q);
void mem_controller_reset_mem_request_queue(MemoryController *m);
void mem_controller_set_burst_length(MemoryController *m, int burst_length);
int mem_controller_create_mem_request(MemoryController *m, target_ulong paddr,
                                      int bytes_to_access,
                                      MemAccessType op_type,
                                      void *p_mem_access_info);
void mem_controller_create_mem_request_pte(MemoryController *m,
                                           target_ulong paddr,
                                           int bytes_to_access,
                                           MemAccessType op_type,
                                           void *p_mem_access_info);
#endif
