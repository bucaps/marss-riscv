/**
 * Set of data structures such as latches used by simulated core pipeline
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
#include <stdlib.h>

#include "../../cutils.h"
#include "../riscv_sim_macros.h"
#include "cpu_latches.h"
#include "sim_log.h"

void
cpu_stage_flush(CPUStage *stage)
{
    stage->has_data = FALSE;
    stage->insn_latch_index = -1;
    stage->stage_exec_done = FALSE;
}

void
cpu_stage_flush_pipe(CPUStage *stage, int num_stages)
{
    int i;

    for (i = 0; i < num_stages; ++i)
    {
        cpu_stage_flush(&stage[i]);
    }
}

void
cpu_stage_flush_free_insn_latch(CPUStage *stage,
                                InstructionLatch *insn_latch_pool)
{
    if (stage->insn_latch_index != -1)
    {
        insn_latch_pool[stage->insn_latch_index].status = INSN_LATCH_FREE;
    }
    cpu_stage_flush(stage);
}

static int
get_free_insn_latch(const InstructionLatch *insn_latch_pool)
{
    int i;

    for (i = 0; i < INSN_LATCH_POOL_SIZE; ++i)
    {
        if (insn_latch_pool[i].status == INSN_LATCH_FREE)
        {
            return i;
        }
    }

    return -1;
}

void
reset_insn_latch_pool(InstructionLatch *insn_latch_pool)
{
    int i;

    /* Add all the latches back to latch pool */
    for (i = 0; i < INSN_LATCH_POOL_SIZE; ++i)
    {
        insn_latch_pool[i].status = INSN_LATCH_FREE;
    }
}

InstructionLatch *
insn_latch_allocate(InstructionLatch *insn_latch_pool)
{
    InstructionLatch *e;
    int insn_latch_index;

    insn_latch_index = get_free_insn_latch(insn_latch_pool);

    sim_assert(
        (insn_latch_index != -1), "error: %s at line %d in %s(): %s", __FILE__,
        __LINE__, __func__,
        "failed to allocate instruction latch from instruction latch pool");

    e = &insn_latch_pool[insn_latch_index];
    memset((void *)e, 0, sizeof(InstructionLatch));
    e->status = INSN_LATCH_ALLOCATED;
    e->insn_latch_index = insn_latch_index;
    return e;
}

InstructionLatch *
get_insn_latch(InstructionLatch *insn_latch_pool, int index)
{
    return (&insn_latch_pool[index]);
}