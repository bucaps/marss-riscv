/**
 * DRAMSim3 CPP wrapper
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
#include <cstdio>
#include <string>

#include "../../cutils.h"
#include "dramsim_wrapper.h"

dramsim_wrapper::dramsim_wrapper(const char *config_file,
                                 const char *output_dir)
    : read_cb(std::bind(&dramsim_wrapper::read_complete, this,
                        std::placeholders::_1)),
      write_cb(std::bind(&dramsim_wrapper::write_complete, this,
                         std::placeholders::_1))
{
    dramsim = GetMemorySystem(std::string(config_file), std::string(output_dir),
                              read_cb, write_cb);

    mem_access_active = FALSE;
}

dramsim_wrapper::~dramsim_wrapper()
{
    delete dramsim;
}

void
dramsim_wrapper::read_complete(uint64_t addr)
{
    mem_access_active = FALSE;
}

void
dramsim_wrapper::write_complete(uint64_t addr)
{
    mem_access_active = FALSE;
}

bool
dramsim_wrapper::can_add_transaction(target_ulong addr, bool isWrite)
{
    return dramsim->WillAcceptTransaction(addr, isWrite);
}

bool
dramsim_wrapper::add_transaction(target_ulong addr, bool isWrite)
{
    mem_access_active = TRUE;
    return dramsim->AddTransaction(addr, isWrite);
}

int
dramsim_wrapper::get_max_clock_cycles()
{
    int clock_cycles_elasped = 0;

    while (mem_access_active)
    {
        dramsim->ClockTick();
        clock_cycles_elasped++;
    }

    return clock_cycles_elasped;
}

void
dramsim_wrapper::print_stats(const char* timestamp)
{
    dramsim->PrintStats(timestamp);
}

void
dramsim_wrapper::reset_stats()
{
    dramsim->ResetStats();
}

int
dramsim_wrapper::get_burst_size()
{
    return (dramsim->GetBusBits() * (dramsim->GetBurstLength() / 8));
}