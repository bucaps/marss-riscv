/**
 * Memory controller utilities
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
#ifndef _MEM_CONTROLLER_UTILS_H_
#define _MEM_CONTROLLER_UTILS_H_

#include "../riscv_sim_typedefs.h"

/* Memory operation type */
typedef enum MemAccessType {
    MEM_ACCESS_READ = 0x0,
    MEM_ACCESS_WRITE = 0x1,
} MemAccessType;

typedef struct PendingMemAccessEntry
{
    int valid;
    int bytes_to_access;
    int max_bytes_to_access;
    target_ulong addr;
    target_ulong req_addr;
    target_ulong req_pte;
    int stage_queue_index;
    int stage_queue_type;
    MemAccessType type;
} PendingMemAccessEntry;

typedef struct StageMemAccessQueue
{
    int cur_idx;
    int max_size;
    int cur_size;
    PendingMemAccessEntry *entry;
} StageMemAccessQueue;
#endif
