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
#ifndef _BASE_DRAM_H_
#define _BASE_DRAM_H_

#include "../../cutils.h"
#include "../riscv_sim_typedefs.h"
#include "../utils/sim_params.h"
#include "memory_controller_utils.h"

typedef struct Dram
{
    /* Type of DRAM model: base or dramsim3 */
    int dram_model_type;

    /* Requests are processed sequentially, one at a time, from the head of
     * mem_req_queue. Processing solely involves simulating a latency in CPU
     * cycles (known as max_clock_cycles). After simulating the latency, the
     * stall on the waiting CPU pipeline stage is removed, and the current entry
     * is dequeued from mem_req_queue. Below parameters keep track of the
     * current request in progress. */
    int mem_access_active;
    int elasped_clock_cycles;
    int max_clock_cycles;
    PendingMemAccessEntry *active_mem_request;

    /* These queues are used to control the stall on fetch and memory CPU
     * pipeline stages */
    StageMemAccessQueue *frontend_mem_access_queue;
    StageMemAccessQueue *backend_mem_access_queue;

    /* Set based on type of DRAM model used: base or dramsim */
    int (*get_max_clock_cycles_for_request)(struct Dram *d,
                                            PendingMemAccessEntry *e);

    /* Following parameters are used by base DRAM model */
    uint64_t last_accessed_page_num;

    /* Fixed configurable latency in CPU cycles used by the base DRAM model */
    int mem_access_latency;
} Dram;

Dram *dram_create(const SimParams *p, StageMemAccessQueue *f,
                  StageMemAccessQueue *b);
int dram_can_accept_request(const Dram *d);
int dram_clock(Dram *d);
void dram_reset(Dram *d);
void dram_send_request(Dram *d, PendingMemAccessEntry *e);
void dram_free(Dram **d);
#endif /* _BASE_DRAM_H_ */
