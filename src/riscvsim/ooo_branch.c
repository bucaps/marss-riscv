/**
 * Out of order core Back-end: Branch Handling
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2017-2019 Gaurav Kothari {gkothar1@binghamton.edu}
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
#include "ooo.h"
#include "../riscv_cpu_shared.h"
#include "circular_queue.h"
#include "riscv_ins_execute_lib.h"
#include "riscv_sim_cpu.h"

void
oo_process_branch(OOCore *core, IMapEntry *e)
{
    e->mispredict
        = core->simcpu->pfn_branch_handler(core->simcpu->emu_cpu_state, e);

    if (e->mispredict)
    {
        cpu_stage_flush(&core->fetch);
        cpu_stage_flush(&core->decode);
        cpu_stage_flush(&core->dispatch);
    }

    switch (e->ins.branch_type)
    {
        case BRANCH_COND:
        {
            core->rob.entries[e->rob_idx].ready = TRUE;
            break;
        }
        case BRANCH_UNCOND:
        {
            if (!e->ins.has_dest)
            {
                core->rob.entries[e->rob_idx].ready = TRUE;
            }
            break;
        }
    }

    e->branch_processed = TRUE;
}
