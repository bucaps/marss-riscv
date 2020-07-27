/*
 * Simulation Parameters
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
#ifndef _SIM_PARAMS_STATS_H_
#define _SIM_PARAMS_STATS_H_

#include <fcntl.h>
#include <inttypes.h>
#include <linux/limits.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "../../json.h"
#include "../riscv_sim_macros.h"

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
#define DEF_START_SIM 0
#define DEF_STATS_DISPLAY 0
#define DEF_DO_SIM_TRACE DISABLE
#define DEF_CREATE_INS_STR 0
#define DEF_SIM_FILE_PATH "."
#define DEF_SIM_FILE_PREFIX "misc"
#define DEF_SIM_TRACE_FILE "trace.txt"
#define DEF_SIM_LOG_FILE "log.txt"

#define DEF_NUM_STAGES 6
#define DEF_ENABLE_PARALLEL_FU DISABLE

#define DEF_IQ_SIZE 16
#define DEF_IQ_ISSUE_PORTS 2
#define DEF_ROB_SIZE 64
#define DEF_ROB_COMMIT_PORTS 1
#define DEF_LSQ_SIZE 16

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
#define DEF_BTB_EVICT_POLICY EVICT_POLICY_RANDOM
#define DEF_BPU_TYPE BPU_TYPE_BIMODAL
#define DEF_FLUSH_BPU_ON_SIMSTART DISABLE

#define DEF_ENABLE_L1_CACHE ENABLE
#define DEF_L1_CODE_CACHE_READ_LATENCY 1
#define DEF_L1_CODE_CACHE_SIZE 32
#define DEF_L1_CODE_CACHE_WAYS 4
#define DEF_L1_CODE_CACHE_EVICT EVICT_POLICY_RANDOM

#define DEF_L1_DATA_CACHE_PROBE_LATENCY 1
#define DEF_L1_DATA_CACHE_READ_LATENCY 1
#define DEF_L1_DATA_CACHE_WRITE_LATENCY 1
#define DEF_L1_DATA_CACHE_SIZE 32
#define DEF_L1_DATA_CACHE_WAYS 4
#define DEF_L1_DATA_CACHE_EVICT EVICT_POLICY_RANDOM

#define DEF_ENABLE_L2_CACHE ENABLE
#define DEF_L2_CACHE_PROBE_LATENCY 1
#define DEF_L2_CACHE_READ_LATENCY 1
#define DEF_L2_CACHE_WRITE_LATENCY 1
#define DEF_L2_CACHE_SIZE 256
#define DEF_L2_CACHE_WAYS 16
#define DEF_L2_CACHE_EVICT EVICT_POLICY_RANDOM

#define DEF_CACHE_READ_ALLOC_POLICY CACHE_READ_ALLOC
#define DEF_CACHE_WRITE_ALLOC_POLICY CACHE_WRITE_ALLOC
#define DEF_CACHE_WRITE_POLICY CACHE_WRITEBACK
#define DEF_CACHE_LINE_SIZE 64

#define DEF_TLB_SIZE 32
#define DEF_DRAM_BURST_SIZE 32
#define DEF_FLUSH_SIM_MEM_ON_SIMSTART DISABLE
#define DEF_MEM_MODEL MEM_MODEL_BASE

#define DEF_PTE_RW_LATENCY 27
#define DEF_MEM_ACCESS_LATENCY 46

#define DEF_DRAMSIM_INI_FILE "DRAMSim2/ini/DDR2_micron_16M_8b_x8_sg3E.ini"
#define DEF_DRAMSIM_SYSTEM_INI_FILE "DRAMSim2/system.ini.example"
#define DEF_DRAMSIM_STATS_DIR "."

#define DEF_SIM_EMULATE_AFTER_ICOUNT 0

extern const char *core_type_str[];
extern const char *sim_param_status[];
extern const char *evict_policy_str[];
extern const char *cache_ra_str[];
extern const char *cache_wa_str[];
extern const char *cache_wp_str[];
extern const char *bpu_type_str[];
extern const char *bpu_aliasing_func_type_str[];
extern const char *dram_model_type_str[];
extern const char *cpu_mode_str[];

typedef struct SimParams
{
    /* Core Params */
    char *core_name;
    int core_type;
    int start_in_sim;
    int enable_stats_display;
    int create_ins_str;
    int do_sim_trace;
    char *sim_trace_file;
    char *sim_file_path;
    char *sim_file_prefix;
    char *sim_log_file;

    /* In-order core */
    int num_cpu_stages;
    int enable_parallel_fu;

    /* Out-of-order core */
    int iq_size;
    int iq_issue_ports;
    int rob_size;
    int rob_commit_ports;
    int lsq_size;

    /* FU Latencies in CPU cycles */
    int num_alu_stages;
    int *alu_stage_latency;

    int num_mul_stages;
    int *mul_stage_latency;

    int num_div_stages;
    int *div_stage_latency;

    int num_fpu_alu_stages;
    int fpu_alu_latency[MAX_FU_FPU_ALU_TYPES];

    int num_fpu_fma_stages;
    int *fpu_fma_stage_latency;

    /* BPU */
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
    int flush_bpu_on_simstart;

    /* L1 Caches */
    int enable_l1_caches;
    int l1_code_cache_read_latency;
    int l1_code_cache_size;
    int l1_code_cache_ways;
    int l1_code_cache_evict;
    int l1_data_cache_read_latency;
    int l1_data_cache_write_latency;
    int l1_data_cache_size;
    int l1_data_cache_ways;
    int l1_data_cache_evict;

    /* L2 Caches */
    int enable_l2_cache;
    int l2_shared_cache_read_latency;
    int l2_shared_cache_write_latency;
    int l2_shared_cache_size;
    int l2_shared_cache_ways;
    int l2_shared_cache_evict;

    /* Common cache parameters */
    int cache_line_size;
    int cache_read_allocate_policy;
    int cache_write_allocate_policy;
    int cache_write_policy;

    /* TLB and DRAM Params */
    int flush_sim_mem_on_simstart;
    uint64_t guest_ram_size;
    int tlb_size;
    int dram_model_type;
    int burst_length;
    int pte_rw_latency;
    int mem_access_latency;

    /* DRAMSim2 Params */
    char *dramsim_ini_file;
    char *dramsim_system_ini_file;
    char *dramsim_stats_dir;

    uint64_t sim_emulate_after_icount;
} SimParams;

SimParams *sim_params_init();
void sim_params_parse(SimParams *p, JSONValue cfg);
void sim_params_log_options(const SimParams *p);
void sim_params_log_exec_unit_config(const SimParams *p);
void sim_params_validate(SimParams *p);
void sim_params_free(SimParams *p);
#endif
