/**
 * RISCV Simulated CPU State
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
#include <time.h>

#include "riscv_sim_cpu.h"
#include "inorder.h"
#include "ooo.h"

RISCVSIMCPUState *
riscv_sim_cpu_init(const SimParams *p, struct RISCVCPUState *s)
{
    RISCVSIMCPUState *simcpu;

    simcpu = calloc(1, sizeof(RISCVSIMCPUState));
    assert(simcpu);

    simcpu->emu_cpu_state = s;
    simcpu->pc = 0x1000;
    simcpu->clock = 0;
    simcpu->params = (SimParams *)p;

    simcpu->stats = (SimStats *)calloc(NUM_MAX_PRV_LEVELS, sizeof(SimStats));
    assert(simcpu->stats != NULL);

    simcpu->imap = (IMapEntry *)calloc(NUM_IMAP_ENTRY, sizeof(IMapEntry));
    assert(simcpu->imap);

    PRINT_PROG_TITLE_MSG("MARSS-RISCV: Micro-Architectural System Simulator for RISC-V");
    simcpu->mmu = mmu_init(simcpu->params);

    /* Seed for random eviction, if used in BPU and caches */
    srand(time(NULL));

    simcpu->pfn_branch_handler = NULL;
    simcpu->pfn_branch_frontend_probe_handler = NULL;

    if (p->enable_bpu)
    {
        PRINT_INIT_MSG("Setting up branch prediction unit");
        simcpu->bpu = bpu_init(p, simcpu->stats);
        simcpu->pfn_branch_handler = &handle_branch_with_bpu;
        simcpu->pfn_branch_frontend_probe_handler = &handle_bpu_frontend_probe;
        simcpu->pfn_branch_frontend_decode_handler
            = &handle_branch_decode_with_bpu;
    }
    else
    {
        simcpu->pfn_branch_handler = &handle_branch_no_bpu;
        simcpu->pfn_branch_frontend_probe_handler
            = &handle_no_bpu_frontend_probe;
        simcpu->pfn_branch_frontend_decode_handler
            = &handle_branch_decode_no_bpu;
    }

    switch (p->core_type)
    {
        case CORE_TYPE_INCORE:
        {
            PRINT_INIT_MSG("Setting up in-order core");
            simcpu->core = (void *)in_core_init(simcpu->params, simcpu);
            simcpu->pfn_core_reset = in_core_reset;
            simcpu->pfn_core_run = in_core_run;
            simcpu->pfn_core_free = in_core_free;
            break;
        }
        case CORE_TYPE_OOCORE:
        {
            PRINT_INIT_MSG("Setting up out-of-order core");
            simcpu->core = (void *)oo_core_init(simcpu->params, simcpu);
            simcpu->pfn_core_reset = oo_core_reset;
            simcpu->pfn_core_run = oo_core_run;
            simcpu->pfn_core_free = oo_core_free;
            break;
        }
    }

    sim_params_print(p);
    return simcpu;
}

void
riscv_sim_cpu_reset(RISCVSIMCPUState *simcpu)
{
    reset_imap(simcpu->imap);
    mem_controller_reset(simcpu->mmu->mem_controller);
    simcpu->pfn_core_reset(simcpu->core);
}

int
riscv_sim_cpu_run(RISCVSIMCPUState *simcpu)
{
    return simcpu->pfn_core_run(simcpu->core);;
}

void
riscv_sim_cpu_free(RISCVSIMCPUState **simcpu)
{
    free((*simcpu)->stats);
    (*simcpu)->stats = NULL;

    free((*simcpu)->imap);
    (*simcpu)->imap = NULL;

    mmu_free(&((*simcpu)->mmu));

    if ((*simcpu)->params->enable_bpu)
    {
        bpu_free(&((*simcpu)->bpu));
    }

    (*simcpu)->pfn_core_free(&(*simcpu)->core);
}