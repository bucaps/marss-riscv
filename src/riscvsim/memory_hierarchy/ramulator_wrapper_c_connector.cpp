/**
 * Ramulator wrapper C connector
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
#include "ramulator_wrapper.h"
#include "ramulator_wrapper_c_connector.h"

#ifdef __cplusplus
extern "C" {
#endif

static ramulator_wrapper *ramulator_wrapper_obj = NULL;

void
ramulator_wrapper_init(const char *config_file, int cache_line_size)
{
    if (ramulator_wrapper_obj == NULL)
    {
        ramulator_wrapper_obj
            = new ramulator_wrapper(config_file, cache_line_size);
    }
}

void
ramulator_wrapper_destroy()
{
    delete ramulator_wrapper_obj;
    ramulator_wrapper_obj = NULL;
}

void
ramulator_wrapper_finish()
{
    ramulator_wrapper_obj->finish();
}

int
ramulator_wrapper_add_transaction(target_ulong addr, int isWrite)
{
    return (int)ramulator_wrapper_obj->add_transaction(addr, (bool)isWrite);
}

int
ramulator_wrapper_get_max_clock_cycles()
{
    return ramulator_wrapper_obj->get_max_clock_cycles();
}

void
ramulator_wrapper_print_stats(const char *stats_dir, const char *timestamp)
{
    ramulator_wrapper_obj->print_stats(stats_dir, timestamp);
}

#ifdef __cplusplus
}
#endif