/**
 * Global typedefs and macros used by MARSS-RISCV
 *
 * Copyright (c) 2016-2017 Fabrice Bellard
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2017-2019 Gaurav Kothari {gkothar1@binghamton.edu}
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
#ifndef _RISCV_SIM_TYPEDEFS_H_
#define _RISCV_SIM_TYPEDEFS_H_

#include <inttypes.h>

#if MAX_XLEN == 32
#define RV32
#define BIT_SIZE 32
#elif MAX_XLEN  == 64
#define RV64
#define BIT_SIZE 64
#endif

#if BIT_SIZE == 32
typedef int32_t target_long;
typedef uint32_t target_ulong;
#define TARGET_ULONG_FMT PRIu32
#define TARGET_LONG_FMT PRId32
#define TARGET_ULONG_SCN SCNu32
#define TARGET_LONG_SCN SCNd32
#define TARGET_ULONG_HEX PRIx32

#elif BIT_SIZE == 64
typedef int64_t target_long;
typedef uint64_t target_ulong;
#define TARGET_ULONG_FMT PRIu64
#define TARGET_LONG_FMT PRId64
#define TARGET_ULONG_SCN SCNu64
#define TARGET_LONG_SCN SCNd64
#define TARGET_ULONG_HEX PRIx64

#endif // BIT_SIZE == 64

#include "riscv_sim_macros.h"

#endif
