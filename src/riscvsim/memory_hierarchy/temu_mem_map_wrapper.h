/**
 * TinyEMU memory map wrapper
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2017-2020 Gaurav Kothari {gkothar1@binghamton.edu}
 * State University of New York at Binghamton
 *
 * Copyright (c) 2018-2019 Parikshit Sarnaik {psarnai1@binghamton.edu}
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
#ifndef _TEMU_MEM_MAP_WRAPPER_H_
#define _TEMU_MEM_MAP_WRAPPER_H_

#include "../utils/cpu_latches.h"

struct RISCVCPUState;

/* Wrapper over TinyEMU memory map used by the simulator to fetch instructions
 * and read/write data (loads, stores and atomics) from the emulated guest
 * physical memory */
typedef struct TemuMemMapWrapper
{
    int (*read_insn)(struct RISCVCPUState *s, InstructionLatch *e);
    int (*exec_load_store_atomic)(struct RISCVCPUState *s, InstructionLatch *e);
} TemuMemMapWrapper;

TemuMemMapWrapper *temu_mem_map_wrapper_init();
void temu_mem_map_wrapper_free(TemuMemMapWrapper **t);
#endif
