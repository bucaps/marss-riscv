/**
 * Exception context setup during simulation
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

#include "../../cutils.h"
#include "sim_exception.h"

void
sim_exception_set(SimException *s, const InstructionLatch *e)
{
    s->pending = TRUE;
    s->cause = e->ins.exception_cause;

    switch (s->cause)
    {
        case SIM_ILLEGAL_OPCODE_EXCEPTION:
        case SIM_COMPLEX_OPCODE_EXCEPTION:
        case SIM_MMU_EXCEPTION:
        {
            s->pc = e->ins.pc;
            s->insn = e->ins.binary;
            strncpy(s->insn_str, e->ins.str, RISCV_INS_STR_MAX_LENGTH);
            s->insn_str[RISCV_INS_STR_MAX_LENGTH - 1] = '\0';
            break;
        }

        case SIM_TEMU_TIMEOUT_EXCEPTION:
        case SIM_ICOUNT_COMPLETE_EXCEPTION:
        {
            /* In case if the committed instruction was a branch, after
             * processing the timer, we must resume at this branch's target if
             * it was taken */
            if (unlikely(e->ins.is_branch && e->is_branch_taken))
            {
                s->pc = e->branch_target;
            }
            else
            {
                /* Else resume at next PC in sequence */
                if ((e->ins.binary & 3) != 3)
                {
                    /* Current instruction is compressed */
                    s->pc = e->ins.pc + 2;
                }
                else
                {
                    s->pc = e->ins.pc + 4;
                }
            }
            break;
        }
    }
}

SimException *
sim_exception_init()
{
    SimException *s;

    s = calloc(1, sizeof(SimException));
    assert(s);
    return s;
}

void
sim_exception_free(SimException **s)
{
    free(*s);
    *s = NULL;
}