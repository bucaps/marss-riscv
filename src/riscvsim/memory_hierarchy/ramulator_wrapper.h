/**
 * Ramulator CPP wrapper
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
#ifndef _RAMULATOR_WRAPPER_H_
#define _RAMULATOR_WRAPPER_H_

#include "../riscv_sim_typedefs.h"
#include "memory_controller_utils.h"

#include <Gem5Wrapper.h>
#include <Request.h>
#include <map>

using namespace ramulator;

class ramulator_wrapper
{
  public:
    ramulator_wrapper();
    ramulator_wrapper(const char *config_file, int cache_line_size);
    ~ramulator_wrapper();
    bool add_transaction(target_ulong addr, bool isWrite);
    int get_max_clock_cycles(PendingMemAccessEntry *e);
    bool access_complete();
    void write_complete(ramulator::Request &req);
    void read_complete(ramulator::Request &req);
    void finish();
    void print_stats(const char *stats_dir, const char *timestamp);

    /* We split the cache-line address into MEM_BUS_WIDTH sized parts, so this map
     * keeps track of callbacks for each of this part */
    std::map<target_ulong, bool> mem_addr_cb_status;

    std::function<void(ramulator::Request &)> read_cb_func;
    std::function<void(ramulator::Request &)> write_cb_func;
    Gem5Wrapper *gem5_wrapper;
};
#endif