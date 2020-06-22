/*
 * Simulation Statistics
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
#ifndef _SIM_STATS_H_
#define _SIM_STATS_H_

#include <inttypes.h>

#include "../riscv_sim_macros.h"

typedef struct SimStats
{
    /* General Stats */
    uint64_t cycles;
    uint64_t insn_mem_delay;
    uint64_t data_mem_delay;
    uint64_t exec_unit_delay;

    /* Instruction Stats */
    uint64_t ins_fetch;
    uint64_t ins_simulated;
    uint64_t ins_emulated;

    uint64_t ins_type[NUM_MAX_INS_TYPES];
    uint64_t ins_cond_branch_taken;

    /* Register Access */
    uint64_t csr_reads;
    uint64_t csr_writes;
    uint64_t fp_regfile_reads;
    uint64_t fp_regfile_writes;
    uint64_t int_regfile_reads;
    uint64_t int_regfile_writes;

    /* FU_Access */
    uint64_t fu_access[NUM_MAX_FU];

    /* BPU */
    uint64_t btb_probes;
    uint64_t btb_hits;
    uint64_t btb_updates;
    uint64_t btb_inserts;

    uint64_t bpu_cond_correct;
    uint64_t bpu_cond_incorrect;
    uint64_t bpu_uncond_correct;
    uint64_t bpu_uncond_incorrect;

    /* TLB stats */
    uint64_t code_tlb_lookups;
    uint64_t code_tlb_hits;

    uint64_t load_tlb_lookups;
    uint64_t load_tlb_hits;

    uint64_t store_tlb_lookups;
    uint64_t store_tlb_hits;

    /* Cache Stats */
    uint64_t icache_read;
    uint64_t icache_read_miss;
    uint64_t dcache_read;
    uint64_t dcache_write;
    uint64_t dcache_read_miss;
    uint64_t dcache_write_miss;
    uint64_t l2_cache_read;
    uint64_t l2_cache_write;
    uint64_t l2_cache_read_miss;
    uint64_t l2_cache_write_miss;

    /* Exceptions */
    uint64_t ecall;
    uint64_t interrupts;
    uint64_t ins_page_walks;
    uint64_t load_page_walks;
    uint64_t store_page_walks;
    uint64_t ins_page_faults;
    uint64_t load_page_faults;
    uint64_t store_page_faults;
} SimStats;

/* Performance counters are printed to file in CSV format when simulation
 * completes */
void sim_stats_print_to_file(const SimStats *s, const char *filename);

/* Performance counters are printed on stderr in a tabular format when
 * simulation completes */
void sim_stats_print_to_terminal(const SimStats *s);
void sim_stats_reset(SimStats *s);
#endif
