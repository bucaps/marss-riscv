/*
 * RISCV RTC Timer Device
 *
 * Copyright (c) 2016-2017 Fabrice Bellard
 * Copyright (c) 2020 Gaurav Kothari
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
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "rtc_timer.h"

uint64_t
rtc_get_host_wall_clock_time(RTC *rtc)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * rtc->freq
           + (ts.tv_nsec / (1000000000 / rtc->freq));
}

/* This simulates the RISC-V mtime CSR register */
uint64_t
rtc_get_elasped_time(RTC *rtc)
{
    return rtc_get_host_wall_clock_time(rtc) - rtc->start_time;
}

RTC *
rtc_init(uint64_t freq)
{
    RTC *rtc = calloc(1, sizeof(RTC));
    assert(rtc);

    rtc->freq = freq;
    rtc->start_time = rtc_get_host_wall_clock_time(rtc);

    return rtc;
}

void
rtc_free(RTC **rtc)
{
    free(*rtc);
}