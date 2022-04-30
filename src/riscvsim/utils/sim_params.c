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
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../utils/sim_log.h"
#include "sim_params.h"

/* Max length for comma separated latency string for FU stages specified in
 * TinyEMU configuration file */
#define LATENCY_STRING_MAX_LENGTH 256

const char *core_type_str[] = {"in-order", "out-of-order"};
const char *sim_param_status[] = {"false", "true"};
const char *evict_policy_str[] = {"random", "bit-plru"};
const char *cache_ra_str[] = {"true", "false"};
const char *cache_wa_str[] = {"true", "false"};
const char *cache_wp_str[] = {"writeback", "writethrough"};
const char *bpu_type_str[] = {"bimodal", "adaptive"};
const char *bpu_aliasing_func_type_str[] = {"xor", "and", "none"};
const char *dram_model_type_str[] = {"base", "dramsim3", "ramulator"};
const char *cpu_mode_str[] = {"user", "supervisor", "hypervisor", "machine"};

void
sim_params_log_options(const SimParams *p)
{
    sim_log_event_to_file(sim_log, "%s", "Setting up TinyEMU options");
    sim_log_param_to_file(sim_log, "%s: %d", "code_tlb_size", p->tlb_size);
    sim_log_param_to_file(sim_log, "%s: %d", "load_tlb_size", p->tlb_size);
    sim_log_param_to_file(sim_log, "%s: %d", "store_tlb_size", p->tlb_size);
    sim_log_param_to_file(sim_log, "%s: %d MB", "guest_ram_size",
                          p->guest_ram_size);

    sim_log_event_to_file(sim_log, "%s", "Setting up simulation options");
    if (p->start_in_sim)
    {
        sim_log_param_to_file(sim_log, "%s", "-simstart");
    }

    if (p->enable_stats_display)
    {
        sim_log_param_to_file(sim_log, "%s", "-sim-stats-display");
        sim_log_param_to_file(sim_log, "%s: %s", "-sim-stats-shm-name",
                              p->sim_stats_shm_name);
    }

    sim_log_param_to_file(sim_log, "%s: %s", "-sim_file_path",
                          p->sim_file_path);
    sim_log_param_to_file(sim_log, "%s: %s", "-sim-file-prefix",
                          p->sim_file_prefix);

    sim_log_param_to_file(sim_log, "%s: %s", "-sim-log-file", p->sim_log_file);

    if (p->do_sim_trace)
    {
        sim_log_param_to_file(sim_log, "%s", "-sim-trace");
        sim_log_param_to_file(sim_log, "%s: %s", "-sim-trace-file",
                              p->sim_trace_file);
    }

    if (p->flush_sim_mem_on_simstart)
    {
        sim_log_param_to_file(sim_log, "%s", "-sim-flush-mem");
    }

    if (p->flush_bpu_on_simstart)
    {
        sim_log_param_to_file(sim_log, "%s", "-sim-flush-bpu");
    }

    if (p->sim_emulate_after_icount)
    {
        sim_log_param_to_file(sim_log, "%s %lu", "-sim-emulate-after-icount",
                              p->sim_emulate_after_icount);
    }

    sim_log_param_to_file(sim_log, "%s: %s", "core_type",
                          core_type_str[p->core_type]);
    sim_log_param_to_file(sim_log, "%s: %lu MHz", "rtc_freq_mhz", p->rtc_freq_mhz);
    sim_log_param_to_file(sim_log, "%s: %lu MHz", "cpu_freq_mhz", p->cpu_freq_mhz);
    sim_log_param_to_file(sim_log, "%s: %s", "enable_bpu",
                          sim_param_status[p->enable_bpu]);
    if (p->enable_bpu)
    {
        sim_log_param_to_file(sim_log, "%s: %s", "bpu_type",
                              bpu_type_str[p->bpu_type]);
        sim_log_param_to_file(sim_log, "%s: %s", "bpu_flush_on_context_switch",
                              sim_param_status[p->bpu_flush_on_context_switch]);
    }
    sim_log_param_to_file(sim_log, "%s: %s", "enable_l1_caches",
                          sim_param_status[p->enable_l1_caches]);
    sim_log_param_to_file(sim_log, "%s: %s", "enable_l2_cache",
                          sim_param_status[p->enable_l2_cache]);
    sim_log_param_to_file(sim_log, "%s: %s", "dram_model_type",
                          dram_model_type_str[p->dram_model_type]);
}

static void
sim_params_set_defaults(SimParams *p)
{
    int i;

    p->core_name = strdup(DEF_CORE_NAME);
    assert(p->core_name);

    p->core_type = DEF_CORE_TYPE;
    p->sim_file_path = strdup(DEF_SIM_FILE_PATH);
    assert(p->sim_file_path);

    p->sim_file_prefix = strdup(DEF_SIM_FILE_PREFIX);
    assert(p->sim_file_prefix);

    p->sim_trace_file = strdup(DEF_SIM_TRACE_FILE);
    assert(p->sim_trace_file);

    p->sim_log_file = strdup(DEF_SIM_LOG_FILE);
    assert(p->sim_log_file);

    p->sim_stats_shm_name = strdup(DEF_SIM_STATS_SHM_NAME);
    assert(p->sim_stats_shm_name);

    p->start_in_sim = DEF_START_SIM;
    p->enable_stats_display = DEF_STATS_DISPLAY;
    p->create_ins_str = DEF_CREATE_INS_STR;

    p->num_cpu_stages = DEF_NUM_STAGES;
    p->enable_parallel_fu = DEF_ENABLE_PARALLEL_FU;

    p->iq_size = DEF_IQ_SIZE;
    p->iq_issue_ports = DEF_IQ_ISSUE_PORTS;
    p->rob_size = DEF_ROB_SIZE;
    p->lsq_size = DEF_LSQ_SIZE;

    p->num_alu_stages = DEF_NUM_ALU_STAGES;
    p->alu_stage_latency = (int *)malloc(sizeof(int) * p->num_alu_stages);
    assert(p->alu_stage_latency);
    for (i = 0; i < p->num_alu_stages; ++i)
    {
        p->alu_stage_latency[i] = DEF_STAGE_LATENCY;
    }

    p->num_mul_stages = DEF_NUM_MUL_STAGES;
    p->mul_stage_latency = (int *)malloc(sizeof(int) * p->num_mul_stages);
    assert(p->mul_stage_latency);
    for (i = 0; i < p->num_mul_stages; ++i)
    {
        p->mul_stage_latency[i] = DEF_STAGE_LATENCY;
    }

    p->num_div_stages = DEF_NUM_DIV_STAGES;
    p->div_stage_latency = (int *)malloc(sizeof(int) * p->num_div_stages);
    assert(p->div_stage_latency);
    for (i = 0; i < p->num_div_stages; ++i)
    {
        p->div_stage_latency[i] = DEF_STAGE_LATENCY;
    }

    p->num_fpu_alu_stages = DEF_NUM_FPU_ALU_STAGES;
    p->fpu_alu_latency[FU_FPU_ALU_FADD] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FSUB] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FMUL] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FDIV] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FSQRT] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FSGNJ] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FMIN] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FMAX] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FEQ] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FLT] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FLE] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FCVT] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_CVT] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FMV] = DEF_STAGE_LATENCY;
    p->fpu_alu_latency[FU_FPU_ALU_FCLASS] = DEF_STAGE_LATENCY;

    p->num_fpu_fma_stages = DEF_NUM_FPU_FMA_STAGES;
    p->fpu_fma_stage_latency
        = (int *)malloc(sizeof(int) * p->num_fpu_fma_stages);
    assert(p->fpu_fma_stage_latency);
    for (i = 0; i < p->num_fpu_fma_stages; ++i)
    {
        p->fpu_fma_stage_latency[i] = DEF_STAGE_LATENCY;
    }

    p->enable_bpu = DEF_ENABLE_BPU;
    p->btb_size = DEF_BTB_SIZE;
    p->btb_ways = DEF_BTB_WAYS;
    p->bht_size = DEF_BHT_SIZE;
    p->ras_size = DEF_RAS_SIZE;
    p->bpu_type = DEF_BPU_TYPE;
    p->bpu_ght_size = DEF_GHT_SIZE;
    p->bpu_pht_size = DEF_PHT_SIZE;
    p->bpu_history_bits = DEF_HISTORY_BITS;
    p->bpu_aliasing_func_type = DEF_BPU_ALIAS_FUNC;
    p->btb_eviction_policy = DEF_BTB_EVICT_POLICY;
    p->flush_bpu_on_simstart = DEF_FLUSH_BPU_ON_SIMSTART;

    p->enable_l1_caches = DEF_ENABLE_L1_CACHE;
    p->l1_code_cache_read_latency = DEF_L1_CODE_CACHE_READ_LATENCY;
    p->l1_code_cache_size = DEF_L1_CODE_CACHE_SIZE;
    p->l1_code_cache_ways = DEF_L1_CODE_CACHE_WAYS;
    p->l1_code_cache_evict = DEF_L1_CODE_CACHE_EVICT;
    p->l1_data_cache_read_latency = DEF_L1_DATA_CACHE_READ_LATENCY;
    p->l1_data_cache_write_latency = DEF_L1_DATA_CACHE_WRITE_LATENCY;
    p->l1_data_cache_size = DEF_L1_DATA_CACHE_SIZE;
    p->l1_data_cache_ways = DEF_L1_DATA_CACHE_WAYS;
    p->l1_data_cache_evict = DEF_L1_DATA_CACHE_EVICT;

    p->enable_l2_cache = DEF_ENABLE_L2_CACHE;
    p->l2_shared_cache_read_latency = DEF_L2_CACHE_READ_LATENCY;
    p->l2_shared_cache_write_latency = DEF_L2_CACHE_WRITE_LATENCY;
    p->l2_shared_cache_size = DEF_L2_CACHE_SIZE;
    p->l2_shared_cache_ways = DEF_L2_CACHE_WAYS;
    p->l2_shared_cache_evict = DEF_L2_CACHE_EVICT;

    p->cache_line_size = DEF_CACHE_LINE_SIZE;
    p->cache_read_allocate_policy = DEF_CACHE_READ_ALLOC_POLICY;
    p->cache_write_allocate_policy = DEF_CACHE_WRITE_ALLOC_POLICY;
    p->cache_write_policy = DEF_CACHE_WRITE_POLICY;

    p->tlb_size = DEF_TLB_SIZE;
    p->tlb_evict = DEF_TLB_EVICT_POLICY;
    p->dram_model_type = DEF_MEM_MODEL;
    p->burst_length = DEF_DRAM_BURST_SIZE;
    p->flush_sim_mem_on_simstart = DEF_FLUSH_SIM_MEM_ON_SIMSTART;

    p->mem_access_latency = DEF_MEM_ACCESS_LATENCY;

    p->dramsim_config_file = strdup(DEF_DRAMSIM_CONFIG_FILE);
    assert(p->dramsim_config_file);

    p->ramulator_config_file = strdup(DEF_RAMULATOR_CONFIG_FILE);
    assert(p->ramulator_config_file);

    p->sim_emulate_after_icount = DEF_SIM_EMULATE_AFTER_ICOUNT;
    p->system_insn_latency = DEF_STAGE_LATENCY;
    p->bpu_flush_on_context_switch = DEF_BPU_FLUSH_ON_CONTEXT_SWITCH;
    p->rtc_freq_mhz = DEF_RTC_FREQ_MHZ;
    p->cpu_freq_mhz = DEF_CPU_FREQ_MHZ;
}

static int
is_power_of_two(int value)
{
    return ((value > 0) && ((value & (value - 1)) == 0));
}

static void
validate_param(const char *param_name, int has_range, int min, int max,
               int current)
{
    if (has_range)
    {
        sim_assert((current >= min && current <= max),
                   "error: %s at line %d in %s(): error validating "
                   "param - %s must be either %d or %d",
                   __FILE__, __LINE__, __func__, param_name, min, max);
    }
    else
    {
        sim_assert((current >= min),
                   "error: %s at line %d in %s(): error validating "
                   "param - %s must have a minimum value of %d",
                   __FILE__, __LINE__, __func__, param_name, min);
    }
}

static void
validate_param_p2(const char *param_name, int val)
{
    sim_assert((is_power_of_two(val)),
               "error: %s at line %d in %s(): error validating param - %s "
               "must be a power of 2",
               __FILE__, __LINE__, __func__, param_name);
}

void
sim_params_validate(SimParams *p)
{
    int i;
    char trace_file_name[1024];

    validate_param("start_in_sim", 1, 0, 1, p->start_in_sim);
    validate_param("enable_stats_display", 1, 0, 1, p->enable_stats_display);

    if (strcmp(p->core_name, "incore") == 0)
    {
        validate_param("num_cpu_stages", 1, 5, 6, p->num_cpu_stages);
    }
    else if (strcmp(p->core_name, "oocore") == 0)
    {
        validate_param("iq_size", 0, 1, 2048, p->iq_size);
        validate_param("iq_issue_ports", 0, 1, 2048, p->iq_issue_ports);
        validate_param("rob_size", 0, 1, 2048, p->rob_size);
        validate_param("lsq_size", 0, 1, 2048, p->lsq_size);
    }

    validate_param("rtc_freq_mhz", 1, 1, 1000, p->rtc_freq_mhz);

    /* Validate FU config */
    validate_param("num_alu_stages", 0, 1, 2048, p->num_alu_stages);

    for (i = 0; i < p->num_alu_stages; ++i)
    {
        validate_param("alu_stage_latency", 0, 1, 2048,
                       p->alu_stage_latency[i]);
    }

    validate_param("num_mul_stages", 0, 1, 2048, p->num_mul_stages);
    for (i = 0; i < p->num_mul_stages; ++i)
    {
        validate_param("mul_stage_latency", 0, 1, 2048,
                       p->mul_stage_latency[i]);
    }

    validate_param("num_div_stages", 0, 1, 2048, p->num_div_stages);
    for (i = 0; i < p->num_div_stages; ++i)
    {
        validate_param("div_stage_latency", 0, 1, 2048,
                       p->div_stage_latency[i]);
    }

    validate_param("num_fpu_alu_stages", 0, 1, 2048, p->num_fpu_alu_stages);
    validate_param("fadd", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FADD]);
    validate_param("fsub", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FSUB]);
    validate_param("fmul", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FMUL]);
    validate_param("fdiv", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FDIV]);
    validate_param("fsqrt", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FSQRT]);
    validate_param("fsgnj", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FSGNJ]);
    validate_param("fmin", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FMIN]);
    validate_param("fmax", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FMAX]);
    validate_param("feq", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FEQ]);
    validate_param("flt", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FLT]);
    validate_param("fle", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FLE]);
    validate_param("fcvt", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FCVT]);
    validate_param("cvt", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_CVT]);
    validate_param("fmv", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FMV]);
    validate_param("fclass", 0, 1, 2048, p->fpu_alu_latency[FU_FPU_ALU_FCLASS]);

    validate_param("num_fpu_fma_stages", 0, 1, 2048, p->num_fpu_fma_stages);
    for (i = 0; i < p->num_fpu_fma_stages; ++i)
    {
        validate_param("fpu_fma_stage_latency", 0, 1, 2048,
                       p->fpu_fma_stage_latency[i]);
    }

    validate_param("system_insn_latency", 0, 1, 2048, p->system_insn_latency);

    /* Validate BPU config */
    validate_param("enable_bpu", 1, 0, 1, p->enable_bpu);
    if (p->enable_bpu)
    {
        validate_param_p2("btb_size", p->btb_size);
        validate_param("btb_ways", 0, 1, 2048, p->btb_ways);
        validate_param("bpu_type", 1, 0, 1, p->bpu_type);
        validate_param("bpu_flush_on_context_switch", 1, 0, 1,
                       p->bpu_flush_on_context_switch);

        switch (p->bpu_type)
        {
            case BPU_TYPE_BIMODAL:
            {
                validate_param_p2("bht_size", p->bht_size);
                break;
            }

            case BPU_TYPE_ADAPTIVE:
            {
                validate_param_p2("bpu_ght_size", p->bpu_ght_size);
                validate_param_p2("bpu_pht_size", p->bpu_pht_size);
                validate_param("bpu_history_bits", 0, 1, 2048,
                               p->bpu_history_bits);
                break;
            }
        }
    }

    /* Validate caches */
    validate_param("enable_l1_caches", 1, 0, 1, p->enable_l1_caches);

    if (p->enable_l1_caches)
    {
        validate_param("l1_code_cache_read_latency", 0, 1, 2048,
                       p->l1_code_cache_read_latency);
        validate_param("l1_code_cache_size", 0, 1, 2048, p->l1_code_cache_size);
        validate_param("l1_code_cache_ways", 0, 1, 2048, p->l1_code_cache_ways);
        validate_param("l1_code_cache_evict", 1, 0, 1, p->l1_code_cache_evict);

        validate_param("l1_data_cache_read_latency", 0, 1, 2048,
                       p->l1_data_cache_read_latency);
        validate_param("l1_data_cache_write_latency", 0, 1, 2048,
                       p->l1_data_cache_write_latency);
        validate_param("l1_data_cache_size", 0, 1, 2048, p->l1_data_cache_size);
        validate_param("l1_data_cache_ways", 0, 1, 2048, p->l1_data_cache_ways);
        validate_param("l1_data_cache_evict", 1, 0, 1, p->l1_data_cache_evict);

        validate_param_p2("cache_line_size", p->cache_line_size);
        validate_param("cache_read_allocate_policy", 1, 0, 1,
                       p->cache_read_allocate_policy);
        validate_param("cache_write_allocate_policy", 1, 0, 1,
                       p->cache_write_allocate_policy);
        validate_param("cache_write_policy", 1, 0, 1, p->cache_write_policy);

        validate_param("enable_l2_cache", 1, 0, 1, p->enable_l2_cache);

        if (p->enable_l2_cache)
        {
            validate_param("l2_shared_cache_read_latency", 0, 1, 2048,
                           p->l2_shared_cache_read_latency);
            validate_param("l2_shared_cache_write_latency", 0, 1, 2048,
                           p->l2_shared_cache_write_latency);
            validate_param("l2_shared_cache_size", 0, 1, 2048,
                           p->l2_shared_cache_size);
            validate_param("l2_shared_cache_ways", 0, 1, 2048,
                           p->l2_shared_cache_ways);
            validate_param("l2_shared_cache_evict", 1, 0, 1,
                           p->l2_shared_cache_evict);
        }
    }

    validate_param("tlb_size", 0, 1, 2048, p->tlb_size);
    validate_param("tlb_evict", 1, 0, 1, p->tlb_evict);
    validate_param("burst_length", 0, 1, 2048, (int)p->burst_length);
    validate_param("mem_access_latency", 0, 1, 2048, p->mem_access_latency);

    /* Create full trace file name */
    strcpy(trace_file_name, p->sim_file_path);
    strcat(trace_file_name, "/");
    strcat(trace_file_name, p->sim_file_prefix);
    strcat(trace_file_name, ".trace");

    free(p->sim_trace_file);
    p->sim_trace_file = strdup(trace_file_name);
}

static void
parse_stage_latency_str(int **dest, int max_stage_count, char *str)
{
    int pos;
    char *ptr;

    /* Re-allocate default allocated latency array with the new size parsed from
     * config file */
    *dest = (int *)realloc((*dest), max_stage_count * sizeof(int));
    assert(*dest);

    pos = 0;
    ptr = strtok(str, ",");
    while (ptr != NULL)
    {
        (*dest)[pos++] = atoi(ptr);
        ptr = strtok(NULL, ",");
        if (pos >= max_stage_count)
        {
            break;
        }
    }

    /* if latency string is shorter than max_stage_count, we must exit */
    sim_assert((pos == max_stage_count),
               "error: %s at line %d in %s(): latency string %s does "
               "not specify all the latencies",
               __FILE__, __LINE__, __func__, str);
}

static void
log_default_param_int(const char *obj, const char *param, int val)
{
    sim_log_event(sim_log, "parsing config file: %s object not found, "
                           "selecting default value(s) for %s.%s -> %d",
                  obj, obj, param, val);
}

static void
log_default_param_str(const char *obj, const char *param, const char *val)
{
    sim_log_event(sim_log, "parsing config file: %s object not found, "
                           "selecting default value(s) for %s.%s -> %s",
                  obj, obj, param, val);
}

void
sim_params_parse(SimParams *p, JSONValue cfg)
{
    char buf1[256];
    const char *tag_name, *str;
    JSONValue core_obj, obj, obj1;
    char stage_latency_str[LATENCY_STRING_MAX_LENGTH];

    snprintf(buf1, sizeof(buf1), "%s", "core");
    core_obj = json_object_get(cfg, buf1);

    if (json_is_undefined(core_obj))
    {
        log_default_param_str(buf1, "", "");
    }

    /* Parse simulation options */
    tag_name = "name";
    if (vm_get_str(core_obj, tag_name, &str) < 0)
    {
        log_default_param_str(buf1, tag_name, p->core_name);
    }
    else
    {
        free(p->core_name);
        p->core_name = strdup(str);
    }

    tag_name = "type";
    if (vm_get_str(core_obj, tag_name, &str) < 0)
    {
        log_default_param_str(buf1, tag_name, core_type_str[p->core_type]);
    }
    else
    {
        if (strcmp(str, "incore") == 0)
        {
            p->core_type = CORE_TYPE_INCORE;
        }
        else if (strcmp(str, "oocore") == 0)
        {
            p->core_type = CORE_TYPE_OOCORE;
        }
    }

    tag_name = "rtc_freq_mhz";
    if (vm_get_int(core_obj, tag_name, &p->rtc_freq_mhz) < 0)
    {
        log_default_param_int(buf1, tag_name, p->rtc_freq_mhz);
    }

    tag_name = "cpu_freq_mhz";
    if (vm_get_int(core_obj, tag_name, &p->cpu_freq_mhz) < 0)
    {
        log_default_param_int(buf1, tag_name, p->cpu_freq_mhz);
    }

    if (p->core_type == CORE_TYPE_INCORE)
    {
        snprintf(buf1, sizeof(buf1), "%s", "incore");
        obj1 = json_object_get(core_obj, buf1);

        if (json_is_undefined(obj1))
        {
            log_default_param_str(buf1, "", "");
        }

        tag_name = "num_cpu_stages";
        if (vm_get_int(obj1, tag_name, &p->num_cpu_stages) < 0)
        {
            log_default_param_int(buf1, tag_name, p->num_cpu_stages);
        }
    }
    else if (p->core_type == CORE_TYPE_OOCORE)
    {
        snprintf(buf1, sizeof(buf1), "%s", "oocore");
        obj1 = json_object_get(core_obj, buf1);

        if (json_is_undefined(obj1))
        {
            log_default_param_str(buf1, "", "");
        }

        tag_name = "iq_size";
        if (vm_get_int(obj1, tag_name, &p->iq_size) < 0)
        {
            log_default_param_int(buf1, tag_name, p->iq_size);
        }

        tag_name = "iq_issue_ports";
        if (vm_get_int(obj1, tag_name, &p->iq_issue_ports) < 0)
        {
            log_default_param_int(buf1, tag_name, p->iq_issue_ports);
        }

        tag_name = "rob_size";
        if (vm_get_int(obj1, tag_name, &p->rob_size) < 0)
        {
            log_default_param_int(buf1, tag_name, p->rob_size);
        }

        tag_name = "rob_commit_ports";
        if (vm_get_int(obj1, tag_name, &p->rob_commit_ports) < 0)
        {
            log_default_param_int(buf1, tag_name, p->rob_commit_ports);
        }

        tag_name = "lsq_size";
        if (vm_get_int(obj1, tag_name, &p->lsq_size) < 0)
        {
            log_default_param_int(buf1, tag_name, p->lsq_size);
        }
    }

    snprintf(buf1, sizeof(buf1), "%s", "functional_units");
    obj = json_object_get(core_obj, buf1);

    if (json_is_undefined(obj))
    {
        log_default_param_str(buf1, "", "");
    }

    tag_name = "num_alu_stages";
    if (vm_get_int(obj, tag_name, &p->num_alu_stages) < 0)
    {
        log_default_param_int(buf1, tag_name, p->num_alu_stages);
    }

    tag_name = "alu_stage_latency";
    if (vm_get_str(obj, tag_name, &str) < 0)
    {
        log_default_param_int(buf1, tag_name, DEF_STAGE_LATENCY);
    }
    else
    {
        strncpy(stage_latency_str, str, LATENCY_STRING_MAX_LENGTH - 1);
        stage_latency_str[LATENCY_STRING_MAX_LENGTH - 1] = '\0';
        parse_stage_latency_str(&p->alu_stage_latency, p->num_alu_stages,
                                stage_latency_str);
    }

    tag_name = "num_mul_stages";
    if (vm_get_int(obj, tag_name, &p->num_mul_stages) < 0)
    {
        log_default_param_int(buf1, tag_name, p->num_mul_stages);
    }

    tag_name = "mul_stage_latency";
    if (vm_get_str(obj, tag_name, &str) < 0)
    {
        log_default_param_int(buf1, tag_name, DEF_STAGE_LATENCY);
    }
    else
    {
        strncpy(stage_latency_str, str, LATENCY_STRING_MAX_LENGTH - 1);
        stage_latency_str[LATENCY_STRING_MAX_LENGTH - 1] = '\0';
        parse_stage_latency_str(&p->mul_stage_latency, p->num_mul_stages,
                                stage_latency_str);
    }

    tag_name = "num_div_stages";
    if (vm_get_int(obj, tag_name, &p->num_div_stages) < 0)
    {
        log_default_param_int(buf1, tag_name, p->num_div_stages);
    }

    tag_name = "div_stage_latency";
    if (vm_get_str(obj, tag_name, &str) < 0)
    {
        log_default_param_int(buf1, tag_name, DEF_STAGE_LATENCY);
    }
    else
    {
        strncpy(stage_latency_str, str, LATENCY_STRING_MAX_LENGTH - 1);
        stage_latency_str[LATENCY_STRING_MAX_LENGTH - 1] = '\0';
        parse_stage_latency_str(&p->div_stage_latency, p->num_div_stages,
                                stage_latency_str);
    }

    tag_name = "num_fpu_fma_stages";
    if (vm_get_int(obj, tag_name, &p->num_fpu_fma_stages) < 0)
    {
        log_default_param_int(buf1, tag_name, p->num_fpu_fma_stages);
    }

    tag_name = "fpu_fma_stage_latency";
    if (vm_get_str(obj, tag_name, &str) < 0)
    {
        log_default_param_int(buf1, tag_name, DEF_STAGE_LATENCY);
    }
    else
    {
        strncpy(stage_latency_str, str, LATENCY_STRING_MAX_LENGTH - 1);
        stage_latency_str[LATENCY_STRING_MAX_LENGTH - 1] = '\0';
        parse_stage_latency_str(&p->fpu_fma_stage_latency,
                                p->num_fpu_fma_stages, stage_latency_str);
    }

    tag_name = "system_insn_latency";
    if (vm_get_int(obj, tag_name, &p->system_insn_latency) < 0)
    {
        log_default_param_int(buf1, tag_name, p->system_insn_latency);
    }

    snprintf(buf1, sizeof(buf1), "%s", "fpu_alu_stage_latency");
    obj1 = json_object_get(obj, buf1);

    if (json_is_undefined(obj1))
    {
        log_default_param_str(buf1, "", "");
    }

    tag_name = "fadd";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FADD]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FADD]);
    }

    tag_name = "fsub";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FSUB]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FSUB]);
    }

    tag_name = "fmul";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FMUL]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FMUL]);
    }

    tag_name = "fdiv";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FDIV]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FDIV]);
    }

    tag_name = "fsqrt";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FSQRT]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FSQRT]);
    }

    tag_name = "fsgnj";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FSGNJ]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FSGNJ]);
    }

    tag_name = "fmin";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FMIN]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FMIN]);
    }

    tag_name = "fmax";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FMAX]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FMAX]);
    }

    tag_name = "feq";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FEQ]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FEQ]);
    }

    tag_name = "flt";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FLT]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FLT]);
    }

    tag_name = "fle";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FLE]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FLE]);
    }

    tag_name = "fcvt";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FCVT]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FCVT]);
    }

    tag_name = "cvt";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_CVT]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_CVT]);
    }

    tag_name = "fmv";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FMV]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FMV]);
    }

    tag_name = "fclass";
    if (vm_get_int(obj1, tag_name, &p->fpu_alu_latency[FU_FPU_ALU_FCLASS]) < 0)
    {
        log_default_param_int(buf1, tag_name,
                              p->fpu_alu_latency[FU_FPU_ALU_FCLASS]);
    }

    /* BPU */
    snprintf(buf1, sizeof(buf1), "%s", "bpu");
    obj = json_object_get(core_obj, buf1);

    if (json_is_undefined(obj))
    {
        log_default_param_str(buf1, "", "");
    }

    tag_name = "enable";
    if (vm_get_str(obj, tag_name, &str) < 0)
    {
        log_default_param_str(buf1, tag_name, sim_param_status[p->enable_bpu]);
    }
    else
    {
        if (strcmp(str, "false") == 0)
        {
            p->enable_bpu = DISABLE;
        }
        else if (strcmp(str, "true") == 0)
        {
            p->enable_bpu = ENABLE;
        }
        else
        {
            sim_assert((0), "error: %s at line %d in %s(): error parsing "
                            "param - %s->%s has invalid value",
                       __FILE__, __LINE__, __func__, buf1, tag_name);
        }
    }

    tag_name = "flush_on_context_switch";
    if (vm_get_str(obj, tag_name, &str) < 0)
    {
        log_default_param_str(buf1, tag_name,
                              sim_param_status[p->bpu_flush_on_context_switch]);
    }
    else
    {
        if (strcmp(str, "false") == 0)
        {
            p->bpu_flush_on_context_switch = DISABLE;
        }
        else if (strcmp(str, "true") == 0)
        {
            p->bpu_flush_on_context_switch = ENABLE;
        }
        else
        {
            sim_assert((0), "error: %s at line %d in %s(): error parsing "
                            "param - %s->%s has invalid value",
                       __FILE__, __LINE__, __func__, buf1, tag_name);
        }
    }

    if (p->enable_bpu == ENABLE)
    {
        snprintf(buf1, sizeof(buf1), "%s", "btb");
        obj1 = json_object_get(obj, buf1);

        if (json_is_undefined(obj1))
        {
            log_default_param_str(buf1, "", "");
        }

        tag_name = "size";
        if (vm_get_int(obj1, tag_name, &p->btb_size) < 0)
        {
            log_default_param_int(buf1, tag_name, p->btb_size);
        }

        tag_name = "ways";
        if (vm_get_int(obj1, tag_name, &p->btb_ways) < 0)
        {
            log_default_param_int(buf1, tag_name, p->btb_ways);
        }

        tag_name = "eviction_policy";
        if (vm_get_str(obj1, tag_name, &str) < 0)
        {
            log_default_param_str(buf1, tag_name,
                                  evict_policy_str[p->btb_eviction_policy]);
        }
        else
        {
            if (strcmp(str, "random") == 0)
            {
                p->btb_eviction_policy = EVICT_POLICY_RANDOM;
            }
            else if (strcmp(str, "lru") == 0)
            {
                p->btb_eviction_policy = EVICT_POLICY_BIT_PLRU;
            }
            else
            {
                sim_assert((0), "error: %s at line %d in %s(): error parsing "
                                "param - %s->%s has invalid value",
                           __FILE__, __LINE__, __func__, buf1, tag_name);
            }
        }

        tag_name = "bpu_type";
        if (vm_get_str(obj, tag_name, &str) < 0)
        {
            log_default_param_str(buf1, tag_name, bpu_type_str[p->bpu_type]);
        }
        else
        {
            if (strcmp(str, "bimodal") == 0)
            {
                p->bpu_type = BPU_TYPE_BIMODAL;
            }
            else if (strcmp(str, "adaptive") == 0)
            {
                p->bpu_type = BPU_TYPE_ADAPTIVE;
            }
            else
            {
                sim_assert((0), "error: %s at line %d in %s(): error parsing "
                                "param - %s->%s has invalid value",
                           __FILE__, __LINE__, __func__, buf1, tag_name);
            }
        }

        switch (p->bpu_type)
        {
            case BPU_TYPE_BIMODAL:
            {
                snprintf(buf1, sizeof(buf1), "%s", "bimodal");
                obj1 = json_object_get(obj, buf1);

                if (json_is_undefined(obj1))
                {
                    log_default_param_str(buf1, "", "");
                }

                tag_name = "bht_size";
                if (vm_get_int(obj1, tag_name, &p->bht_size) < 0)
                {
                    log_default_param_int(buf1, tag_name, p->bht_size);
                }
                break;
            }

            case BPU_TYPE_ADAPTIVE:
            {
                snprintf(buf1, sizeof(buf1), "%s", "adaptive");
                obj1 = json_object_get(obj, buf1);

                if (json_is_undefined(obj1))
                {
                    log_default_param_str(buf1, "", "");
                }

                tag_name = "ght_size";
                if (vm_get_int(obj1, tag_name, &p->bpu_ght_size) < 0)
                {
                    log_default_param_int(buf1, tag_name, p->bpu_ght_size);
                }

                tag_name = "pht_size";
                if (vm_get_int(obj1, tag_name, &p->bpu_pht_size) < 0)
                {
                    log_default_param_int(buf1, tag_name, p->bpu_pht_size);
                }

                tag_name = "history_bits";
                if (vm_get_int(obj1, tag_name, &p->bpu_history_bits) < 0)
                {
                    log_default_param_int(buf1, tag_name, p->bpu_history_bits);
                }

                if ((p->bpu_ght_size == 1) && (p->bpu_pht_size == 1))
                {
                    tag_name = "aliasing_func_type";
                    if (vm_get_str(obj1, tag_name, &str) < 0)
                    {
                        log_default_param_str(buf1, tag_name,
                                              bpu_aliasing_func_type_str
                                                  [p->bpu_aliasing_func_type]);
                    }
                    else
                    {
                        if (strcmp(str, "xor") == 0)
                        {
                            p->bpu_aliasing_func_type = BPU_ALIAS_FUNC_XOR;
                        }
                        else if (strcmp(str, "and") == 0)
                        {
                            p->bpu_aliasing_func_type = BPU_ALIAS_FUNC_AND;
                        }
                        else if (strcmp(str, "none") == 0)
                        {
                            p->bpu_aliasing_func_type = BPU_ALIAS_FUNC_NONE;
                        }
                        else
                        {
                            sim_assert(
                                (0),
                                "error: %s at line %d in %s(): error parsing "
                                "param - %s->%s has invalid value",
                                __FILE__, __LINE__, __func__, buf1, tag_name);
                        }
                    }
                }
                break;
            }
        }

        tag_name = "ras_size";
        if (vm_get_int(obj, tag_name, &p->ras_size) < 0)
        {
            log_default_param_int(buf1, tag_name, p->ras_size);
        }
    }

    snprintf(buf1, sizeof(buf1), "%s", "caches");
    obj1 = json_object_get(core_obj, buf1);

    if (json_is_undefined(obj1))
    {
        log_default_param_str(buf1, "", "");
    }

    /* L1 Caches */
    tag_name = "enable_l1_caches";
    if (vm_get_str(obj1, tag_name, &str) < 0)
    {
        log_default_param_str(buf1, tag_name,
                              sim_param_status[p->enable_l1_caches]);
    }
    else
    {
        if (strcmp(str, "false") == 0)
        {
            p->enable_l1_caches = DISABLE;
        }
        else if (strcmp(str, "true") == 0)
        {
            p->enable_l1_caches = ENABLE;
        }
        else
        {
            sim_assert((0), "error: %s at line %d in %s(): error parsing "
                            "param - %s->%s has invalid value",
                       __FILE__, __LINE__, __func__, buf1, tag_name);
        }
    }

    if (p->enable_l1_caches)
    {
        snprintf(buf1, sizeof(buf1), "%s", "icache");
        obj = json_object_get(obj1, buf1);

        if (json_is_undefined(obj))
        {
            log_default_param_str(buf1, "", "");
        }

        tag_name = "latency";
        if (vm_get_int(obj, tag_name, &p->l1_code_cache_read_latency) < 0)
        {
            log_default_param_int(buf1, tag_name,
                                  p->l1_code_cache_read_latency);
        }

        tag_name = "size";
        if (vm_get_int(obj, tag_name, &p->l1_code_cache_size) < 0)
        {
            log_default_param_int(buf1, tag_name, p->l1_code_cache_size);
        }

        tag_name = "ways";
        if (vm_get_int(obj, tag_name, &p->l1_code_cache_ways) < 0)
        {
            log_default_param_int(buf1, tag_name, p->l1_code_cache_ways);
        }

        tag_name = "eviction";
        if (vm_get_str(obj, tag_name, &str) < 0)
        {
            log_default_param_str(buf1, tag_name,
                                  evict_policy_str[p->l1_code_cache_evict]);
        }
        else
        {
            if (strcmp(str, "lru") == 0)
            {
                p->l1_code_cache_evict = EVICT_POLICY_BIT_PLRU;
            }
            else if (strcmp(str, "random") == 0)
            {
                p->l1_code_cache_evict = EVICT_POLICY_RANDOM;
            }
            else
            {
                sim_assert((0), "error: %s at line %d in %s(): error parsing "
                                "param - %s->%s has invalid value",
                           __FILE__, __LINE__, __func__, buf1, tag_name);
            }
        }

        snprintf(buf1, sizeof(buf1), "%s", "dcache");
        obj = json_object_get(obj1, buf1);

        if (json_is_undefined(obj))
        {
            log_default_param_str(buf1, "", "");
        }

        tag_name = "latency";
        if (vm_get_int(obj, tag_name, &p->l1_data_cache_read_latency) < 0)
        {
            log_default_param_int(buf1, tag_name,
                                  p->l1_data_cache_read_latency);
        }

        tag_name = "size";
        if (vm_get_int(obj, tag_name, &p->l1_data_cache_size) < 0)
        {
            log_default_param_int(buf1, tag_name, p->l1_data_cache_size);
        }

        tag_name = "ways";
        if (vm_get_int(obj, tag_name, &p->l1_data_cache_ways) < 0)
        {
            log_default_param_int(buf1, tag_name, p->l1_data_cache_ways);
        }

        tag_name = "eviction";
        if (vm_get_str(obj, tag_name, &str) < 0)
        {
            log_default_param_str(buf1, tag_name,
                                  evict_policy_str[p->l1_data_cache_evict]);
        }
        else
        {
            if (strcmp(str, "lru") == 0)
            {
                p->l1_data_cache_evict = EVICT_POLICY_BIT_PLRU;
            }
            else if (strcmp(str, "random") == 0)
            {
                p->l1_data_cache_evict = EVICT_POLICY_RANDOM;
            }
            else
            {
                sim_assert((0), "error: %s at line %d in %s(): error parsing "
                                "param - %s->%s has invalid value",
                           __FILE__, __LINE__, __func__, buf1, tag_name);
            }
        }

        tag_name = "line_size";
        if (vm_get_int(obj1, tag_name, &p->cache_line_size) < 0)
        {
            log_default_param_int(buf1, tag_name, p->cache_line_size);
        }

        tag_name = "allocate_on_write_miss";
        if (vm_get_str(obj1, tag_name, &str) < 0)
        {
            log_default_param_str(buf1, tag_name,
                                  cache_wa_str[p->cache_write_allocate_policy]);
        }
        else
        {
            if (strcmp(str, "true") == 0)
            {
                p->cache_write_allocate_policy = CACHE_WRITE_ALLOC;
            }
            else if (strcmp(str, "false") == 0)
            {
                p->cache_write_allocate_policy = CACHE_WRITE_NO_ALLOC;
            }
            else
            {
                sim_assert((0), "error: %s at line %d in %s(): error parsing "
                                "param - %s->%s has invalid value",
                           __FILE__, __LINE__, __func__, buf1, tag_name);
            }
        }

        tag_name = "write_policy";
        if (vm_get_str(obj1, tag_name, &str) < 0)
        {
            log_default_param_str(buf1, tag_name,
                                  cache_wp_str[p->cache_write_policy]);
        }
        else
        {
            if (strcmp(str, "writeback") == 0)
            {
                p->cache_write_policy = CACHE_WRITEBACK;
            }
            else if (strcmp(str, "writethrough") == 0)
            {
                p->cache_write_policy = CACHE_WRITETHROUGH;
            }
            else
            {
                sim_assert((0), "error: %s at line %d in %s(): error parsing "
                                "param - %s->%s has invalid value",
                           __FILE__, __LINE__, __func__, buf1, tag_name);
            }
        }

        snprintf(buf1, sizeof(buf1), "%s", "l2_shared_cache");
        obj = json_object_get(obj1, buf1);

        if (json_is_undefined(obj))
        {
            log_default_param_str(buf1, "", "");
        }

        tag_name = "enable";
        if (vm_get_str(obj, tag_name, &str) < 0)
        {
            log_default_param_str(buf1, tag_name,
                                  sim_param_status[p->enable_l2_cache]);
        }
        else
        {
            if (strcmp(str, "false") == 0)
            {
                p->enable_l2_cache = DISABLE;
            }
            else if (strcmp(str, "true") == 0)
            {
                p->enable_l2_cache = ENABLE;
            }
            else
            {
                sim_assert((0), "error: %s at line %d in %s(): error parsing "
                                "param - %s->%s has invalid value",
                           __FILE__, __LINE__, __func__, buf1, tag_name);
            }
        }

        if (p->enable_l2_cache)
        {
            tag_name = "latency";
            if (vm_get_int(obj, tag_name, &p->l2_shared_cache_read_latency) < 0)
            {
                log_default_param_int(buf1, tag_name,
                                      p->l2_shared_cache_read_latency);
            }

            tag_name = "size";
            if (vm_get_int(obj, tag_name, &p->l2_shared_cache_size) < 0)
            {
                log_default_param_int(buf1, tag_name, p->l2_shared_cache_size);
            }

            tag_name = "ways";
            if (vm_get_int(obj, tag_name, &p->l2_shared_cache_ways) < 0)
            {
                log_default_param_int(buf1, tag_name, p->l2_shared_cache_ways);
            }

            tag_name = "eviction";
            if (vm_get_str(obj, tag_name, &str) < 0)
            {
                log_default_param_str(
                    buf1, tag_name, evict_policy_str[p->l2_shared_cache_evict]);
            }
            else
            {
                if (strcmp(str, "lru") == 0)
                {
                    p->l2_shared_cache_evict = EVICT_POLICY_BIT_PLRU;
                }
                else if (strcmp(str, "random") == 0)
                {
                    p->l2_shared_cache_evict = EVICT_POLICY_RANDOM;
                }
                else
                {
                    sim_assert((0),
                               "error: %s at line %d in %s(): error parsing "
                               "param - %s->%s has invalid value",
                               __FILE__, __LINE__, __func__, buf1, tag_name);
                }
            }
        }
    }

    snprintf(buf1, sizeof(buf1), "%s", "memory");
    obj1 = json_object_get(cfg, buf1);

    if (json_is_undefined(obj1))
    {
        log_default_param_str(buf1, "", "");
    }

    tag_name = "tlb_size";
    if (vm_get_int(obj1, tag_name, &p->tlb_size) < 0)
    {
        log_default_param_int(buf1, tag_name, p->tlb_size);
    }

    tag_name = "tlb_evict_policy";
    if (vm_get_str(obj1, tag_name, &str) < 0)
    {
        log_default_param_str(buf1, tag_name,
                              evict_policy_str[p->tlb_evict]);
    }
    else
    {
        if (strcmp(str, "lru") == 0)
        {
            p->tlb_evict = EVICT_POLICY_BIT_PLRU;
        }
        else if (strcmp(str, "random") == 0)
        {
            p->tlb_evict = EVICT_POLICY_RANDOM;
        }
        else
        {
            sim_assert((0), "error: %s at line %d in %s(): error parsing "
                            "param - %s->%s has invalid value",
                       __FILE__, __LINE__, __func__, buf1, tag_name);
        }
    }

    tag_name = "burst_length";
    if (vm_get_int(obj1, tag_name, (int *)&p->burst_length) < 0)
    {
        log_default_param_int(buf1, tag_name, p->burst_length);
    }

    switch (p->dram_model_type)
    {
        case MEM_MODEL_BASE:
        {
            snprintf(buf1, sizeof(buf1), "%s", "base_dram_model");
            obj = json_object_get(obj1, buf1);

            if (json_is_undefined(obj))
            {
                log_default_param_str(buf1, "", "");
            }

            tag_name = "mem_access_latency";
            if (vm_get_int(obj, tag_name, &p->mem_access_latency) < 0)
            {
                log_default_param_int(buf1, tag_name, p->mem_access_latency);
            }
            break;
        }
        case MEM_MODEL_DRAMSIM:
        {
            snprintf(buf1, sizeof(buf1), "%s", "dramsim3");
            obj = json_object_get(obj1, buf1);

            if (json_is_undefined(obj))
            {
                log_default_param_str(buf1, "", "");
            }

            tag_name = "config_file";
            if (vm_get_str(obj, tag_name, &str) < 0)
            {
                log_default_param_str(buf1, tag_name, p->dramsim_config_file);
            }
            else
            {
                free(p->dramsim_config_file);
                p->dramsim_config_file = strdup(str);
            }
            break;
        }
        case MEM_MODEL_RAMULATOR:
        {
            snprintf(buf1, sizeof(buf1), "%s", "ramulator");
            obj = json_object_get(obj1, buf1);

            if (json_is_undefined(obj))
            {
                log_default_param_str(buf1, "", "");
            }

            tag_name = "config_file";
            if (vm_get_str(obj, tag_name, &str) < 0)
            {
                log_default_param_str(buf1, tag_name, p->ramulator_config_file);
            }
            else
            {
                free(p->ramulator_config_file);
                p->ramulator_config_file = strdup(str);
            }
            break;
        }
        default:
        {
            sim_assert((0),
                       "error: %s at line %d in %s(): invalid memory model",
                       __FILE__, __LINE__, __func__);
        }
    }
}

static void
sim_param_log_exec_unit_param(const char *fu_num_stage_name, int num_stages,
                              int *latencies)
{
    int i;

    sim_log_param_to_file(sim_log, "num_%s_stages: %d", fu_num_stage_name,
                          num_stages);
    for (i = 0; i < num_stages; ++i)
    {
        sim_log_param_to_file(sim_log, "%s_stage_%d: %d cycle(s)",
                              fu_num_stage_name, i, latencies[i]);
    }
}

void
sim_params_log_exec_unit_config(const SimParams *p)
{
    sim_log_event_to_file(sim_log, "%s", "Setting up functional units");
    sim_param_log_exec_unit_param("int_alu", p->num_alu_stages,
                                  p->alu_stage_latency);
    sim_param_log_exec_unit_param("int_mul", p->num_mul_stages,
                                  p->mul_stage_latency);
    sim_param_log_exec_unit_param("int_div", p->num_div_stages,
                                  p->div_stage_latency);
    sim_param_log_exec_unit_param("fpu_fma", p->num_fpu_fma_stages,
                                  p->fpu_fma_stage_latency);
    sim_log_param_to_file(sim_log, "num_%s_stages: %d", "fpu_alu",
                          p->num_fpu_alu_stages);
    sim_log_param_to_file(sim_log, "%s latency: %d", "fpu_alu",
                          p->num_fpu_alu_stages);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "fadd",
                          p->fpu_alu_latency[FU_FPU_ALU_FADD]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "fsub",
                          p->fpu_alu_latency[FU_FPU_ALU_FSUB]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "fmul",
                          p->fpu_alu_latency[FU_FPU_ALU_FMUL]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "fdiv",
                          p->fpu_alu_latency[FU_FPU_ALU_FDIV]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "fsqrt",
                          p->fpu_alu_latency[FU_FPU_ALU_FSQRT]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "fsgnj",
                          p->fpu_alu_latency[FU_FPU_ALU_FSGNJ]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "fmin",
                          p->fpu_alu_latency[FU_FPU_ALU_FMIN]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "fmax",
                          p->fpu_alu_latency[FU_FPU_ALU_FMAX]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "feq",
                          p->fpu_alu_latency[FU_FPU_ALU_FEQ]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "flt",
                          p->fpu_alu_latency[FU_FPU_ALU_FLT]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "fle",
                          p->fpu_alu_latency[FU_FPU_ALU_FLE]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "fcvt",
                          p->fpu_alu_latency[FU_FPU_ALU_FCVT]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "cvt",
                          p->fpu_alu_latency[FU_FPU_ALU_CVT]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "fmv",
                          p->fpu_alu_latency[FU_FPU_ALU_FMV]);
    sim_log_param_to_file(sim_log, "%s latency: %d cycle(s)", "fclass",
                          p->fpu_alu_latency[FU_FPU_ALU_FCLASS]);
    sim_log_param_to_file(sim_log, "%s latency: %d", "system_insn_latency",
                          p->system_insn_latency);
}

SimParams *
sim_params_init()
{
    SimParams *p;

    p = calloc(1, sizeof(SimParams));
    assert(p);
    sim_params_set_defaults(p);
    return p;
}

void
sim_params_free(SimParams *p)
{
    free(p->alu_stage_latency);
    p->alu_stage_latency = NULL;
    free(p->mul_stage_latency);
    p->mul_stage_latency = NULL;
    free(p->div_stage_latency);
    p->div_stage_latency = NULL;
    free(p->fpu_fma_stage_latency);
    p->fpu_fma_stage_latency = NULL;

    free(p->sim_file_prefix);
    p->sim_file_prefix = NULL;

    free(p->sim_trace_file);
    p->sim_trace_file = NULL;

    free(p->sim_log_file);
    p->sim_log_file = NULL;

    free(p->sim_file_path);
    p->sim_file_path = NULL;

    free(p->core_name);
    p->core_name = NULL;

    free(p->dramsim_config_file);
    p->dramsim_config_file = NULL;

    free(p->ramulator_config_file);
    p->ramulator_config_file = NULL;

    free(p->sim_stats_shm_name);
    p->sim_stats_shm_name = NULL;

    free(p);
}