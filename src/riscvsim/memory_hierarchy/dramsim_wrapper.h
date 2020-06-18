/**
 * DRAMSim2 CPP wrapper
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
#ifndef _DRAMSIM_WRAPPER_H_
#define _DRAMSIM_WRAPPER_H_

#include <cstdint>

#include "../riscv_sim_typedefs.h"
#include "memory_controller_utils.h"
#include <DRAMSim.h>

using namespace DRAMSim;

class dramsim_wrapper
{
  public:
    dramsim_wrapper();
    dramsim_wrapper(const char *dram_ini_file, const char *system_ini_file,
                    const char *stats_dir, const char *app_name, int size_mb,
                    StageMemAccessQueue *frontend_mem_access_queue,
                    StageMemAccessQueue *backend_mem_access_queue);
    ~dramsim_wrapper();
    bool can_add_transaction(target_ulong addr);
    bool add_transaction(target_ulong addr, bool isWrite);
    void update();
    void print_stats();
    int get_burst_size();
    void read_complete(unsigned, uint64_t, uint64_t);
    void write_complete(unsigned, uint64_t, uint64_t);

    MultiChannelMemorySystem *dramsim;
    TransactionCompleteCB *read_cb;
    TransactionCompleteCB *write_cb;
    StageMemAccessQueue *frontend_mem_access_queue;
    StageMemAccessQueue *backend_mem_access_queue;
};
#endif