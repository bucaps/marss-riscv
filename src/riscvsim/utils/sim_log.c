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
#include <assert.h>
#include <stdarg.h>
#include <stdlib.h>

#include "sim_log.h"

SimLog *sim_log;

SimLog *
sim_log_init(const char *filename)
{
    SimLog *s;
    s = calloc(1, sizeof(SimLog));
    assert(s);
    s->log_fp = fopen(filename, "w");
    assert(s->log_fp);
    return s;
}

void
sim_log_event_to_file(SimLog *s, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    assert(fprintf(s->log_fp, "%s ", "* (marss-riscv):") != 0);
    assert(vfprintf(s->log_fp, fmt, args) != 0);
    assert(fprintf(s->log_fp, "%s\n", "") != 0);
    fflush(s->log_fp);
    va_end(args);
}

void
sim_log_param_to_file(SimLog *s, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    assert(fprintf(s->log_fp, "\t%s ", "*") != 0);
    assert(vfprintf(s->log_fp, fmt, args) != 0);
    assert(fprintf(s->log_fp, "%s\n", "") != 0);
    fflush(s->log_fp);
    va_end(args);
}

/* This logs event both to stderr and log file */
void
sim_log_event(SimLog *s, const char *fmt, ...)
{
    va_list args, args1;
    va_copy(args, args1);

    /* Add log to terminal */
    va_start(args, fmt);
    assert(fprintf(stderr, "%s ", "* (marss-riscv):") != 0);
    assert(vfprintf(stderr, fmt, args) != 0);
    assert(fprintf(stderr, "%s\n", "") != 0);
    fflush(stderr);
    va_end(args);

    /* Add log to file */
    va_start(args1, fmt);
    assert(fprintf(s->log_fp, "%s ", "* (marss-riscv):") != 0);
    assert(vfprintf(s->log_fp, fmt, args1) != 0);
    assert(fprintf(s->log_fp, "%s\n", "") != 0);
    fflush(s->log_fp);
    va_end(args1);
}

/* This logs parameter both to stderr and log file */
void
sim_log_param(SimLog *s, const char *fmt, ...)
{
    va_list args, args1;
    va_copy(args, args1);

    /* Add log to terminal */
    va_start(args, fmt);
    assert(fprintf(stderr, "\t%s ", "*") != 0);
    assert(vfprintf(stderr, fmt, args) != 0);
    assert(fprintf(stderr, "%s\n", "") != 0);
    fflush(stderr);
    va_end(args);

    va_start(args1, fmt);
    assert(fprintf(s->log_fp, "\t%s ", "*") != 0);
    assert(vfprintf(s->log_fp, fmt, args1) != 0);
    assert(fprintf(s->log_fp, "%s\n", "") != 0);
    fflush(s->log_fp);
    va_end(args1);
}

void
sim_log_free(SimLog **s)
{
    fclose((*s)->log_fp);
    free(*s);
    *s = NULL;
}