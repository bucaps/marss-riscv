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
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "../../riscv_cpu_priv.h"
#include "sim_log.h"
#include "sim_stats.h"

#define SIM_STAT_PRINT_TO_FILE_HEADER(fp)                                      \
    do                                                                         \
    {                                                                          \
        fprintf(fp, "%s,%s,%s,%s,%s,%s\n", "stat-name", "user", "supervisor",  \
                "hypervisor", "machine", "total");                             \
    } while (0)

#define SIM_STAT_PRINT_TO_FILE(fp, stats, name, attr)                          \
    do                                                                         \
    {                                                                          \
        fprintf(                                                               \
            fp, "%s,%lu,%lu,%lu,%lu,%lu\n", name, stats[0].attr,               \
            stats[1].attr, stats[2].attr, stats[3].attr,                       \
            (stats[0].attr + stats[1].attr + stats[2].attr + stats[3].attr));  \
    } while (0)

#define SIM_STAT_PRINT_HEADER_TO_TERMINAL(fp)                                  \
    do                                                                         \
    {                                                                          \
        fprintf(fp, "%-30s %-18s %-18s %-18s %-18s %-18s\n", "stat-name",      \
                "user", "supervisor", "hypervisor", "machine", "total");       \
    } while (0)

#define SIM_STAT_PRINT_TO_TERMINAL(fp, stats, name, attr)                      \
    do                                                                         \
    {                                                                          \
        fprintf(                                                               \
            fp, "%-30s %-18lu %-18lu %-18lu %-18lu %-18lu\n", name,            \
            stats[0].attr, stats[1].attr, stats[2].attr, stats[3].attr,        \
            (stats[0].attr + stats[1].attr + stats[2].attr + stats[3].attr));  \
    } while (0)

void
sim_stats_print_to_file(const SimStats *s, const char *pathname,
                        uint64_t sim_time_milli_sec, const char *timestamp)
{
    FILE *fp;
    char *filename;
    char buffer[1024];

    /* Generate stats filename with format: prefix_timestamp.csv */
    sprintf(buffer, "%s.csv", timestamp);

    filename = (char *)malloc(strlen(pathname) + strlen(buffer) + 2);
    assert(filename);

    strcpy(filename, pathname);
    strcat(filename, "/");
    strcat(filename, buffer);

    fp = fopen(filename, "w");
    assert(fp);

    SIM_STAT_PRINT_TO_FILE_HEADER(fp);

    SIM_STAT_PRINT_TO_FILE(fp, s, "cycles", cycles);
    SIM_STAT_PRINT_TO_FILE(fp, s, "commits", ins_simulated);
    SIM_STAT_PRINT_TO_FILE(fp, s, "ins_fetch", ins_fetch);

    /* Add simulation time required on host machine to stats file, but only the
     * total time */
    fprintf(fp, "%s,%lu,%lu,%lu,%lu,%lu\n", "sim_time_milli_sec", (uint64_t)0,
            (uint64_t)0, (uint64_t)0, (uint64_t)0, sim_time_milli_sec);

    SIM_STAT_PRINT_TO_FILE(fp, s, "insn_mem_delay", insn_mem_delay);
    SIM_STAT_PRINT_TO_FILE(fp, s, "data_mem_delay", data_mem_delay);
    SIM_STAT_PRINT_TO_FILE(fp, s, "exec_unit_delay", exec_unit_delay);

    SIM_STAT_PRINT_TO_FILE(fp, s, "load_insn", ins_type[INS_TYPE_LOAD]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "store_insn", ins_type[INS_TYPE_STORE]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "atomic_insn", ins_type[INS_TYPE_ATOMIC]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "system_insn", ins_emulated);
    SIM_STAT_PRINT_TO_FILE(fp, s, "aritmetic_insn",
                           ins_type[INS_TYPE_ARITMETIC]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "cond_branches",
                           ins_type[INS_TYPE_COND_BRANCH]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "jal_insn", ins_type[INS_TYPE_JAL]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "jalr_insn", ins_type[INS_TYPE_JALR]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "int_mul_insn", ins_type[INS_TYPE_INT_MUL]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "int_div_insn", ins_type[INS_TYPE_INT_DIV]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fp_load_insn", ins_type[INS_TYPE_FP_LOAD]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fp_store_insn", ins_type[INS_TYPE_FP_STORE]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fp_add_insn", ins_type[INS_TYPE_FP_ADD]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fp_mul_insn", ins_type[INS_TYPE_FP_MUL]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fp_fma_insn", ins_type[INS_TYPE_FP_FMA]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fp_div_sqrt_insn",
                           ins_type[INS_TYPE_FP_DIV_SQRT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fp_misc_insn", ins_type[INS_TYPE_FP_MISC]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "load_byte_insn",
                           ins_type[INS_TYPE_LOAD_BYTE]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "load_half_word_insn",
                           ins_type[INS_TYPE_LOAD_HALF_WORD]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "load_word_insn",
                           ins_type[INS_TYPE_LOAD_WORD]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "load_double_word_insn",
                           ins_type[INS_TYPE_LOAD_DOUBLE_WORD]);

    SIM_STAT_PRINT_TO_FILE(fp, s, "itlb_reads", code_tlb_lookups);
    SIM_STAT_PRINT_TO_FILE(fp, s, "itlb_hits", code_tlb_hits);
    SIM_STAT_PRINT_TO_FILE(fp, s, "load_tlb_reads", load_tlb_lookups);
    SIM_STAT_PRINT_TO_FILE(fp, s, "load_tlb_hits", load_tlb_hits);
    SIM_STAT_PRINT_TO_FILE(fp, s, "store_tlb_reads", store_tlb_lookups);
    SIM_STAT_PRINT_TO_FILE(fp, s, "store_tlb_hits", store_tlb_hits);
    SIM_STAT_PRINT_TO_FILE(fp, s, "victim_tlb_reads", victim_tlb_lookups);
    SIM_STAT_PRINT_TO_FILE(fp, s, "victim_tlb_hits", victim_tlb_hits);

    SIM_STAT_PRINT_TO_FILE(fp, s, "cond_branches_taken", ins_cond_branch_taken);
    SIM_STAT_PRINT_TO_FILE(fp, s, "cond_branches_pred_correct",
                           bpu_cond_correct);
    SIM_STAT_PRINT_TO_FILE(fp, s, "cond_branches_pred_incorrect",
                           bpu_cond_incorrect);
    SIM_STAT_PRINT_TO_FILE(fp, s, "uncond_branches_pred_correct",
                           bpu_uncond_correct);
    SIM_STAT_PRINT_TO_FILE(fp, s, "uncond_branches_pred_incorrect",
                           bpu_uncond_incorrect);

    SIM_STAT_PRINT_TO_FILE(fp, s, "btb_reads", btb_probes);
    SIM_STAT_PRINT_TO_FILE(fp, s, "btb_hits", btb_hits);
    SIM_STAT_PRINT_TO_FILE(fp, s, "btb_inserts", btb_inserts);
    SIM_STAT_PRINT_TO_FILE(fp, s, "btb_updates", btb_updates);

    SIM_STAT_PRINT_TO_FILE(fp, s, "int_regfile_reads", int_regfile_reads);
    SIM_STAT_PRINT_TO_FILE(fp, s, "int_regfile_writes", int_regfile_writes);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fp_regfile_reads", fp_regfile_reads);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fp_regfile_writes", fp_regfile_writes);
    SIM_STAT_PRINT_TO_FILE(fp, s, "csr_reads", csr_reads);
    SIM_STAT_PRINT_TO_FILE(fp, s, "csr_writes", csr_writes);

    SIM_STAT_PRINT_TO_FILE(fp, s, "fu_alu_accesses", fu_access[FU_ALU]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fu_mul_accesses", fu_access[FU_MUL]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fu_div_accesses", fu_access[FU_DIV]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fu_fpu_alu_accesses", fu_access[FU_FPU_ALU]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fu_fpu_fma_accesses", fu_access[FU_FPU_FMA]);

    SIM_STAT_PRINT_TO_FILE(fp, s, "ins_page_walks", ins_page_walks);
    SIM_STAT_PRINT_TO_FILE(fp, s, "load_page_walks", load_page_walks);
    SIM_STAT_PRINT_TO_FILE(fp, s, "store_page_walks", store_page_walks);

    SIM_STAT_PRINT_TO_FILE(fp, s, "misaligned_fetch", exceptions[CAUSE_MISALIGNED_FETCH]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fault_fetch", exceptions[CAUSE_FAULT_FETCH]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "illegal_instruction", exceptions[CAUSE_ILLEGAL_INSTRUCTION]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "breakpoint", exceptions[CAUSE_BREAKPOINT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "misaligned_load", exceptions[CAUSE_MISALIGNED_LOAD]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fault_load", exceptions[CAUSE_FAULT_LOAD]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "misaligned_store", exceptions[CAUSE_MISALIGNED_STORE]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fault_store", exceptions[CAUSE_FAULT_STORE]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "user_ecall", exceptions[CAUSE_USER_ECALL]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "supervisor_ecall", exceptions[CAUSE_SUPERVISOR_ECALL]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "hypervisor_ecall", exceptions[CAUSE_HYPERVISOR_ECALL]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "machine_ecall", exceptions[CAUSE_MACHINE_ECALL]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "fetch_page_fault", exceptions[CAUSE_FETCH_PAGE_FAULT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "load_page_fault", exceptions[CAUSE_LOAD_PAGE_FAULT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "store_page_fault", exceptions[CAUSE_STORE_PAGE_FAULT]);

    SIM_STAT_PRINT_TO_FILE(fp, s, "user_software_interrupt", interrupts[CAUSE_USER_SOFTWARE_INTERRUPT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "supervisor_software_interrupt", interrupts[CAUSE_SUPERVISOR_SOFTWARE_INTERRUPT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "machine_software_interrupt", interrupts[CAUSE_MACHINE_SOFTWARE_INTERRUPT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "user_timer_interrupt", interrupts[CAUSE_USER_TIMER_INTERRUPT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "supervisor_timer_interrupt", interrupts[CAUSE_SUPERVISOR_TIMER_INTERRUPT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "machine_timer_interrupt", interrupts[CAUSE_MACHINE_TIMER_INTERRUPT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "user_external_interrupt", interrupts[CAUSE_USER_EXTERNAL_INTERRUPT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "supervisor_external_interrupt", interrupts[CAUSE_SUPERVISOR_EXTERNAL_INTERRUPT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "machine_external_interrupt", interrupts[CAUSE_MACHINE_EXTERNAL_INTERRUPT]);
    SIM_STAT_PRINT_TO_FILE(fp, s, "pipeline_flush", pipeline_flush);

    SIM_STAT_PRINT_TO_FILE(fp, s, "L1_icache_reads", icache_read);
    SIM_STAT_PRINT_TO_FILE(fp, s, "L1_icache_read_misses", icache_read_miss);

    SIM_STAT_PRINT_TO_FILE(fp, s, "L1_dcache_reads", dcache_read);
    SIM_STAT_PRINT_TO_FILE(fp, s, "L1_dcache_read_misses", dcache_read_miss);
    SIM_STAT_PRINT_TO_FILE(fp, s, "L1_dcache_writes", dcache_write);
    SIM_STAT_PRINT_TO_FILE(fp, s, "L1_dcache_write_misses", dcache_write_miss);

    SIM_STAT_PRINT_TO_FILE(fp, s, "L2_cache_reads", l2_cache_read);
    SIM_STAT_PRINT_TO_FILE(fp, s, "L2_cache_read_misses", l2_cache_read_miss);
    SIM_STAT_PRINT_TO_FILE(fp, s, "L2_cache_writes", l2_cache_write);
    SIM_STAT_PRINT_TO_FILE(fp, s, "L2_cache_write_misses", l2_cache_write_miss);

    fclose(fp);
    sim_log_event(sim_log, "Saved simulation stats in %s", filename);
    free(filename);
}

void
sim_stats_reset(SimStats *s)
{
    memset((void *)s, 0, NUM_MAX_PRV_LEVELS * sizeof(SimStats));
}

int
sim_file_path_valid(const char *path)
{
    struct stat sb;
    return ((stat(path, &sb) == 0) && S_ISDIR(sb.st_mode));
}
