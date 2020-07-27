/**
 * Simulation Log Generator Utility
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
#ifndef _SIM_LOG_H_
#define _SIM_LOG_H_

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>

#define sim_assert(cond, fmt, ...)                                             \
    do                                                                         \
    {                                                                          \
        if (!(cond))                                                           \
        {                                                                      \
            sim_log_event(sim_log, fmt, __VA_ARGS__);                          \
            abort();                                                           \
        }                                                                      \
    } while (0)

typedef struct SimLog
{
    FILE *log_fp;
} SimLog;

extern SimLog *sim_log;

SimLog *sim_log_init(const char *filename);
void sim_log_event(SimLog *s, const char *fmt, ...);
void sim_log_param(SimLog *s, const char *fmt, ...);
void sim_log_event_to_file(SimLog *s, const char *fmt, ...);
void sim_log_param_to_file(SimLog *s, const char *fmt, ...);
void sim_log_free(SimLog **s);
#endif
