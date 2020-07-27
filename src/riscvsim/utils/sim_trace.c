/**
 * Simulation Trace Generator Utility
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
#include <stdlib.h>

#include "sim_trace.h"

void
sim_trace_start(SimTrace *s, const char *filename)
{
    s->trace_fp = fopen(filename, "w");
    assert(s->trace_fp);
}

void
sim_trace_stop(SimTrace *s)
{
    fclose(s->trace_fp);
}

void
sim_trace_commit(const SimTrace *s, uint64_t clock_cycle, int cpu_mode,
                 InstructionLatch *e)
{
    fprintf(s->trace_fp, "cycle=%" TARGET_ULONG_FMT, clock_cycle);
    fprintf(s->trace_fp, " pc=%" TARGET_ULONG_HEX, e->ins.pc);
    fprintf(s->trace_fp, " insn=%" PRIx32, e->ins.binary);
    fprintf(s->trace_fp, " %s", e->ins.str);
    fprintf(s->trace_fp, " mode=%s", cpu_mode_str[cpu_mode]);
    fprintf(s->trace_fp, "\n");
}

void
sim_trace_exception(const SimTrace *s, uint64_t clock_cycle, int cpu_mode,
                    SimException *e)
{
    fprintf(s->trace_fp, "cycle=%" TARGET_ULONG_FMT, clock_cycle);
    fprintf(s->trace_fp, " pc=%" TARGET_ULONG_HEX, e->pc);
    fprintf(s->trace_fp, " insn=%" PRIx32, e->insn);
    fprintf(s->trace_fp, " %s", e->insn_str);
    fprintf(s->trace_fp, " mode=%s", cpu_mode_str[cpu_mode]);
    fprintf(s->trace_fp, "\n");
}

SimTrace *
sim_trace_init()
{
    SimTrace *s;
    s = calloc(1, sizeof(SimTrace));
    assert(s);
    return s;
}

void
sim_trace_free(SimTrace **s)
{
    free(*s);
    *s = NULL;
}