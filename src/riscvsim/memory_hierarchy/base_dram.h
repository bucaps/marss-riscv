/**
 * Base DRAM model
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
#include "../utils/sim_params_stats.h"
#include "memory_controller_utils.h"

typedef struct BaseDram
{
    /* In base model, requests are processed sequentially, one at a time, from the
     * head of mem_req_queue. Processing solely involves simulating a fixed
     * configurable latency in CPU cycles. After simulating the latency, the
     * stall on the waiting CPU pipeline stage is removed, and the current entry
     * is dequeued from mem_req_queue. Below parameter are current request in 
     * progress */
    int mem_access_active;
    int elasped_clock_cycles;
    int max_clock_cycles;
    PendingMemAccessEntry *active_mem_request;

    /* Base DRAM model keeps track of the physical page number of the latest
     * request processed. Any subsequent accesses to the same physical page
     * occupies a lower delay, which is roughly 60 percent of the fixed
     * mem_access_latency. */
    uint64_t last_accessed_page_num;

    /* Fixed configurable latency in CPU cycles for data and page walk */
    int mem_access_latency;
    int pte_rw_latency;

    /* These queues are used to control the stall on fetch and memory CPU
     * pipeline stages */
    StageMemAccessQueue *frontend_mem_access_queue;
    StageMemAccessQueue *backend_mem_access_queue;

    int (*clock)(struct BaseDram *d);
    int (*can_accept_request)(struct BaseDram *d);
    void (*send_request)(struct BaseDram *d, PendingMemAccessEntry *e);
    void (*reset)(struct BaseDram *d);
} BaseDram;

BaseDram *base_dram_create(const SimParams *p, StageMemAccessQueue *f,
                           StageMemAccessQueue *b);
void base_dram_free(BaseDram **d);

#endif /* _BASE_DRAM_H_ */
