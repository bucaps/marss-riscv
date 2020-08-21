/**
 * DRAMSim3 wrapper C connector
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
#ifndef _DRAMSIM_WRAPPER_C_CONNECTOR_H_
#define _DRAMSIM_WRAPPER_C_CONNECTOR_H_

#include <inttypes.h>

#include "../riscv_sim_typedefs.h"

#ifdef __cplusplus
extern "C" {
#endif
void dramsim_wrapper_init(const char *config_file, const char *output_dir);
void dramsim_wrapper_destroy();
int dramsim_wrapper_can_add_transaction(target_ulong addr, int isWrite);
int dramsim_wrapper_add_transaction(target_ulong addr, int isWrite);
int dramsim_wrapper_get_max_clock_cycles();
void dramsim_wrapper_print_stats(const char* timestamp);
void dramsim_wrapper_reset_stats();
int dramsim_get_burst_size();

#ifdef __cplusplus
}
#endif
#endif