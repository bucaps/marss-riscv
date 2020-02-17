/*
 * Simulation Config and Statistics
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
#ifndef _SIM_PARAMS_STATS_H_
#define _SIM_PARAMS_STATS_H_

#include <fcntl.h>
#include <inttypes.h>
#include <linux/limits.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "riscv_sim_macros.h"

/* Used for creating shared memory for writing stats */
#define POSIX_IPC_NAME_PREFIX "/user-"
#define MARSS_STATS_SHM_NAME POSIX_IPC_NAME_PREFIX "marss-stats-shm"
#define ALL_RW_PERMS (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWGRP)

enum SIM_PARAM_STATUS
{
    DISABLE,
    ENABLE
};

enum CORE_TYPE
{
    CORE_TYPE_INCORE,
    CORE_TYPE_OOCORE
};

enum BPU_TYPE
{
    BPU_TYPE_BIMODAL,
    BPU_TYPE_ADAPTIVE
};

enum BTB_EVICT_POLICY
{
    BTB_RANDOM_EVICT,
    BTB_LRU_EVICT
};

enum CACHE_READ_ALLOC_POLICY
{
    CACHE_READ_ALLOC,
    CACHE_READ_NO_ALLOC
};

enum CACHE_WRITE_ALLOC_POLICY
{
    CACHE_WRITE_ALLOC,
    CACHE_WRITE_NO_ALLOC
};

enum CACHE_WRITE_POLICY
{
    CACHE_WRITEBACK,
    CACHE_WRITETHROUGH,
};

enum CACHE_EVICT_POLICY
{
    CACHE_RANDOM_EVICT,
    CACHE_LRU_EVICT
};

enum BPU_ALIAS_FUNC
{
    BPU_ALIAS_FUNC_XOR,
    BPU_ALIAS_FUNC_AND,
    BPU_ALIAS_FUNC_NONE
};

enum MEM_MODEL_TYPE
{
    MEM_MODEL_BASE,
    MEM_MODEL_DRAMSIM,
};

/* Default values for simulation parameters */
#define DEF_CORE_NAME "default-riscv-core"
#define DEF_CORE_TYPE CORE_TYPE_INCORE
#define DEF_NUM_STAGES 6
#define DEF_START_SIM 0
#define DEF_STATS_DISPLAY 0
#define DEF_CREATE_INS_STR 0
#define DEF_SIM_TRACE_FILE "simtrace.txt"
#define DEF_SIM_STATS_PATH "."
#define DEF_NUM_ALU_STAGES 1
#define DEF_NUM_MUL_STAGES 1
#define DEF_NUM_DIV_STAGES 1
#define DEF_NUM_FPU_ALU_STAGES 1
#define DEF_NUM_FPU_FMA_STAGES 1
#define DEF_STAGE_LATENCY 1

#define DEF_ENABLE_BPU ENABLE
#define DEF_BTB_SIZE 512
#define DEF_BTB_WAYS 2
#define DEF_BHT_SIZE 256
#define DEF_RAS_SIZE 6
#define DEF_GHT_SIZE 1
#define DEF_PHT_SIZE 1
#define DEF_HISTORY_BITS 2
#define DEF_BPU_ALIAS_FUNC BPU_ALIAS_FUNC_NONE
#define DEF_BTB_EVICT_POLICY BTB_RANDOM_EVICT
#define DEF_BPU_TYPE BPU_TYPE_BIMODAL

#define DEF_ENABLE_L1_CACHE ENABLE
#define DEF_L1_CODE_CACHE_PROBE_LATENCY 1
#define DEF_L1_CODE_CACHE_SIZE 32
#define DEF_L1_CODE_CACHE_WAYS 4
#define DEF_L1_CODE_CACHE_EVICT CACHE_LRU_EVICT

#define DEF_L1_DATA_CACHE_PROBE_LATENCY 1
#define DEF_L1_DATA_CACHE_SIZE 32
#define DEF_L1_DATA_CACHE_WAYS 4
#define DEF_L1_DATA_CACHE_EVICT CACHE_LRU_EVICT

#define DEF_ENABLE_L2_CACHE ENABLE
#define DEF_L2_CACHE_PROBE_LATENCY 1
#define DEF_L2_CACHE_SIZE 256
#define DEF_L2_CACHE_WAYS 16
#define DEF_L2_CACHE_EVICT CACHE_LRU_EVICT

#define DEF_CACHE_READ_ALLOC_POLICY CACHE_READ_ALLOC
#define DEF_CACHE_WRITE_ALLOC_POLICY CACHE_WRITE_ALLOC
#define DEF_CACHE_WRITE_POLICY CACHE_WRITEBACK
#define DEF_WORDS_PER_CACHE_LINE 8

#define DEF_TLB_SIZE 32
#define DEF_DRAM_BURST_SIZE 32
#define DEF_MEM_BUS_ACCESS_RTT_LATENCY 1
#define DEF_tCL 7
#define DEF_tRCD 4
#define DEF_tRP 4
#define DEF_ROW_BUFFER_WRITE_LATENCY 4

#define DEF_PRF_INT_SIZE 64
#define DEF_PRF_FP_SIZE 64
#define DEF_IQ_INT_SIZE 16
#define DEF_IQ_FP_SIZE 16
#define DEF_IQ_MEM_SIZE 16
#define DEF_IQ_INT_ISSUE_PORTS 2
#define DEF_IQ_FP_ISSUE_PORTS 2
#define DEF_IQ_MEM_ISSUE_PORTS 2
#define DEF_PRF_INT_WP 3
#define DEF_PRF_FP_WP 16
#define DEF_ROB_SIZE 64
#define DEF_ROB_COMMIT_PORTS 1
#define DEF_LSQ_SIZE 16
#define DEF_BIS_SIZE 8

#define DEF_MEM_MODEL MEM_MODEL_BASE
#define DEF_DRAMSIM_INI_FILE "DRAMSim2/ini/DDR2_micron_16M_8b_x8_sg3E.ini"
#define DEF_DRAMSIM_SYSTEM_INI_FILE "DRAMSim2/system.ini.example"
#define DEF_DRAMSIM_STATS_DIR "."

#define PRINT_SIM_STAT_HEADER(fp)                                         \
  do {                                                                    \
    fprintf(fp, "%s,%s,%s,%s,%s,%s\n", "stat-name", "user", "supervisor", \
            "hypervisor", "machine", "total");                            \
  } while (0)

#define PRINT_SIM_STAT(fp, stats, name, attr)                                 \
  do {                                                                        \
    fprintf(fp, "%s,%lu,%lu,%lu,%lu,%lu\n", name, stats[0].attr,              \
            stats[1].attr, stats[2].attr, stats[3].attr,                      \
            (stats[0].attr + stats[1].attr + stats[2].attr + stats[3].attr)); \
  } while (0)

#define PRINT_SIM_STAT_HEADER_TO_TERMINAL(fp)                                 \
  do {                                                                        \
    fprintf(fp, "%-30s %-18s %-18s %-18s %-18s %-18s\n", "stat-name", "user", \
            "supervisor", "hypervisor", "machine", "total");                  \
  } while (0)

#define PRINT_SIM_STAT_TO_TERMINAL(fp, stats, name, attr)                     \
  do {                                                                        \
    fprintf(fp, "%-30s %-18lu %-18lu %-18lu %-18lu %-18lu\n", name,           \
            stats[0].attr, stats[1].attr, stats[2].attr, stats[3].attr,       \
            (stats[0].attr + stats[1].attr + stats[2].attr + stats[3].attr)); \
  } while (0)

extern const char *core_type_str[];
extern const char *sim_param_status[];
extern const char *cache_evict_str[];
extern const char *cache_ra_str[];
extern const char *cache_wa_str[];
extern const char *cache_wp_str[];
extern const char *bpu_type_str[];
extern const char *btb_evict_str[];
extern const char *bpu_aliasing_func_type_str[];
extern const char *mem_model_type_str[];

typedef struct SimParams
{
    /* System Params */
    char *core_name;
    int core_type;
    int start_in_sim;
    int enable_stats_display;
    int create_ins_str;
    char *sim_trace_file;
    char *sim_stats_path;

    /* For in-order core */
    int num_cpu_stages;

    /* For out-of-order core */
    int prf_int_size;
    int prf_fp_size;
    int iq_int_size;
    int iq_fp_size;
    int iq_mem_size;
    int iq_int_issue_ports;
    int iq_fp_issue_ports;
    int iq_mem_issue_ports;
    int prf_int_write_ports;
    int prf_fp_write_ports;
    int rob_size;
    int rob_commit_ports;
    int lsq_size;
    int bis_size;

    /* FU Latencies */
    int num_alu_stages;
    int *alu_stage_latency;

    int num_mul_stages;
    int *mul_stage_latency;

    int num_div_stages;
    int *div_stage_latency;

    int num_fpu_alu_stages;
    int *fpu_alu_stage_latency;

    int num_fpu_fma_stages;
    int *fpu_fma_stage_latency;

    /* BPU  */
    int enable_bpu;
    int btb_size;
    int btb_ways;
    int bht_size;
    int ras_size;
    int bpu_type;
    int bpu_ght_size;
    int bpu_pht_size;
    int bpu_history_bits;
    int bpu_aliasing_func_type;
    int btb_eviction_policy;

    /* L1 Caches */
    int enable_l1_caches;
    int l1_code_cache_probe_latency;
    int l1_code_cache_size;
    int l1_code_cache_ways;
    int l1_code_cache_evict;
    int l1_data_cache_probe_latency;
    int l1_data_cache_size;
    int l1_data_cache_ways;
    int l1_data_cache_evict;

    /* L2 Caches */
    int enable_l2_cache;
    int l2_probe_latency;
    int l2_shared_cache_size;
    int l2_shared_cache_ways;
    int l2_shared_cache_evict;

    /* Common cache parameters */
    int words_per_cache_line;
    int cache_read_allocate_policy;
    int cache_write_allocate_policy;
    int cache_write_policy;

    /* TLB and DRAM Params */
    uint64_t guest_ram_size;
    int tlb_size;
    uint32_t dram_burst_size;
    int mem_bus_access_rtt_latency;
    int tCL;
    int tRCD;
    int tRP;
    int row_buffer_write_latency;

    /* DRAMSim2 params */
    int mem_model_type;
    char *dramsim_ini_file;
    char *dramsim_system_ini_file;
    char *dramsim_stats_dir;
} SimParams;

typedef struct SimStats
{
    /* General Stats */
    uint64_t sim_time;
    uint64_t cycles;
    uint64_t frontend_mem_delay;
    uint64_t backend_mem_delay;

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
    uint64_t btb_miss_for_branches;
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
    uint64_t l1i_read;
    uint64_t l1i_read_miss;
    uint64_t l1d_read;
    uint64_t l1d_write;
    uint64_t l1d_read_miss;
    uint64_t l1d_write_miss;
    uint64_t l2_read;
    uint64_t l2_write;
    uint64_t l2_read_miss;
    uint64_t l2_write_miss;

    /* Exceptions */
    uint64_t ins_page_faults;
    uint64_t load_page_faults;
    uint64_t store_page_faults;
} SimStats;

SimParams *sim_params_init();
void sim_params_print(const SimParams *p);
void sim_params_validate(const SimParams *p);
void sim_params_free(SimParams *p);

void sim_stats_print(SimStats *s, const char *filename);
void sim_stats_reset(SimStats *s);

#endif
