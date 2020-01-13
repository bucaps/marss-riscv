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
#include "sim_params_stats.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

const char *core_type_str[] = {"in-order", "out-of-order"};
const char *sim_param_status[] = {"false", "true"};
const char *cache_evict_str[] = {"random", "lru"};
const char *cache_ra_str[] = {"true", "false"};
const char *cache_wa_str[] = {"true", "false"};
const char *cache_wp_str[] = {"writeback", "writethrough"};
const char *bpu_type_str[] = {"bimodal", "adaptive"};
const char *btb_evict_str[] = {"random", "lru"};
const char *bpu_aliasing_func_type_str[] = {"xor", "and", "none"};
const char *mem_model_type_str[] = {"base", "dramsim2"};

static unsigned
next_high_power_of_2(unsigned n)
{
    n--;

    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;

    return ++n;
}

void
sim_params_set_defaults(SimParams *p)
{
    int i;

    p->core_name = strdup(DEF_CORE_NAME);
    assert(p->core_name);

    p->core_type = DEF_CORE_TYPE;

    p->sim_stats_path = strdup(DEF_SIM_STATS_PATH);
    assert(p->sim_stats_path);

    p->sim_trace_file = strdup(DEF_SIM_TRACE_FILE);
    assert(p->sim_trace_file);

    p->num_cpu_stages = DEF_NUM_STAGES;
    p->start_in_sim = DEF_START_SIM;
    p->enable_stats_display = DEF_STATS_DISPLAY;
    p->create_ins_str = DEF_CREATE_INS_STR;

    p->tlb_size = DEF_TLB_SIZE;

    /* FU Latencies */
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
    p->fpu_alu_stage_latency
        = (int *)malloc(sizeof(int) * p->num_fpu_alu_stages);
    assert(p->fpu_alu_stage_latency);
    for (i = 0; i < p->num_fpu_alu_stages; ++i)
    {
        p->fpu_alu_stage_latency[i] = DEF_STAGE_LATENCY;
    }

    p->num_fpu_fma_stages = DEF_NUM_FPU_FMA_STAGES;
    p->fpu_fma_stage_latency
        = (int *)malloc(sizeof(int) * p->num_fpu_fma_stages);
    assert(p->fpu_fma_stage_latency);
    for (i = 0; i < p->num_fpu_fma_stages; ++i)
    {
        p->fpu_fma_stage_latency[i] = DEF_STAGE_LATENCY;
    }

    /* BPU */
    p->enable_bpu = DEF_ENABLE_BPU;
    p->btb_size = DEF_BTB_SIZE;
    p->btb_ways = DEF_BTB_WAYS;
    p->bpu_type = DEF_BPU_TYPE;

    p->bpu_ght_size = DEF_GHT_SIZE;
    p->bpu_pht_size = DEF_PHT_SIZE;
    p->bpu_history_bits = DEF_HISTORY_BITS;
    p->bpu_aliasing_func_type = DEF_BPU_ALIAS_FUNC;
    p->btb_eviction_policy = DEF_BTB_EVICT_POLICY;

    /* L1 Cache */
    p->enable_l1_caches = DEF_ENABLE_L1_CACHE;

    p->l1_code_cache_probe_latency = DEF_L1_CODE_CACHE_PROBE_LATENCY;
    p->l1_code_cache_size = DEF_L1_CODE_CACHE_SIZE;
    p->l1_code_cache_ways = DEF_L1_CODE_CACHE_WAYS;
    p->l1_code_cache_evict = DEF_L1_CODE_CACHE_EVICT;

    p->l1_data_cache_probe_latency = DEF_L1_DATA_CACHE_PROBE_LATENCY;
    p->l1_data_cache_size = DEF_L1_DATA_CACHE_SIZE;
    p->l1_data_cache_ways = DEF_L1_DATA_CACHE_WAYS;
    p->l1_data_cache_evict = DEF_L1_DATA_CACHE_EVICT;

    /* L2 Caches */
    p->enable_l2_cache = DEF_ENABLE_L2_CACHE;
    p->l2_probe_latency = DEF_L2_CACHE_PROBE_LATENCY;
    p->l2_shared_cache_size = DEF_L2_CACHE_SIZE;
    p->l2_shared_cache_ways = DEF_L2_CACHE_WAYS;
    p->l2_shared_cache_evict = DEF_L2_CACHE_EVICT;

    /* Common cache parameters */
    p->words_per_cache_line = DEF_WORDS_PER_CACHE_LINE;
    p->cache_read_allocate_policy = DEF_CACHE_READ_ALLOC_POLICY;
    p->cache_write_allocate_policy = DEF_CACHE_WRITE_ALLOC_POLICY;
    p->cache_write_policy = DEF_CACHE_WRITE_POLICY;

    /* DRAM params */
    p->dram_burst_size = DEF_DRAM_BURST_SIZE;
    p->mem_bus_access_rtt_latency = DEF_MEM_BUS_ACCESS_RTT_LATENCY;
    p->tCL = DEF_tCL;
    p->tRCD = DEF_tRCD;
    p->tRP = DEF_tRP;
    p->row_buffer_write_latency = DEF_ROW_BUFFER_WRITE_LATENCY;

    p->prf_int_size = DEF_PRF_INT_SIZE;
    p->prf_fp_size = DEF_PRF_FP_SIZE;
    p->iq_int_size = DEF_IQ_INT_SIZE;
    p->iq_fp_size = DEF_IQ_FP_SIZE;
    p->iq_mem_size = DEF_IQ_MEM_SIZE;
    p->iq_int_issue_ports = DEF_IQ_INT_ISSUE_PORTS;
    p->iq_fp_issue_ports = DEF_IQ_FP_ISSUE_PORTS;
    p->iq_mem_issue_ports = DEF_IQ_MEM_ISSUE_PORTS;
    p->prf_int_write_ports = DEF_PRF_INT_WP;
    p->prf_fp_write_ports = DEF_PRF_FP_WP;
    p->rob_size = DEF_ROB_SIZE;
    p->lsq_size = DEF_LSQ_SIZE;
    p->bis_size = DEF_BIS_SIZE;

    p->mem_model_type = DEF_MEM_MODEL;
    p->dramsim_ini_file = strdup(DEF_DRAMSIM_INI_FILE);
    assert(p->dramsim_ini_file);
    p->dramsim_system_ini_file = strdup(DEF_DRAMSIM_SYSTEM_INI_FILE);
    assert(p->dramsim_system_ini_file);
    p->dramsim_stats_dir = strdup(DEF_DRAMSIM_STATS_DIR);
    assert(p->dramsim_stats_dir);
}

static void
print_fu_config(const char *fu_num_stage_param_name,
                const char *fu_stage_latency_name, int num_stages,
                int *latencies)
{
    int i;

    fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", fu_num_stage_param_name,
            num_stages);

    fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : ", fu_stage_latency_name);
    for (i = 0; i < num_stages; ++i)
    {
        fprintf(stderr, "%d", latencies[i]);
        if (i == (num_stages - 1))
        {
            fprintf(stderr, "\n");
        }
        else
        {
            fprintf(stderr, ", ");
        }
    }
}

void
sim_params_print(const SimParams *p)
{
    fprintf(stderr, "\x1B[35m Simulation Parameters:\x1B[0m\n");
    fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "core_name",
            p->core_name);

    fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "core_type",
            core_type_str[p->core_type]);
    fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "sim_trace_file",
            p->sim_trace_file);
    fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "sim_stats_path",
            p->sim_stats_path);
    fprintf(stderr, "\n");

    if (p->core_type == CORE_TYPE_INCORE)
    {
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "num_cpu_stages",
                p->num_cpu_stages);
    }
    else if (p->core_type == CORE_TYPE_OOCORE)
    {
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "prf_int_size",
                p->prf_int_size);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "prf_fp_size",
                p->prf_fp_size);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "iq_int_size",
                p->iq_int_size);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "iq_fp_size",
                p->iq_fp_size);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "iq_mem_size",
                p->iq_mem_size);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "iq_int_issue_ports",
                p->iq_int_issue_ports);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "iq_fp_issue_ports",
                p->iq_fp_issue_ports);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "iq_mem_issue_ports",
                p->iq_mem_issue_ports);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "prf_int_write_ports",
                p->prf_int_write_ports);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "prf_fp_write_ports",
                p->prf_fp_write_ports);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "rob_size",
                p->rob_size);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "rob_commit_ports",
                p->rob_commit_ports);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "lsq_size",
                p->lsq_size);
    }
    fprintf(stderr, "\n");

    print_fu_config("num_alu_stages", "alu_stage_latencies", p->num_alu_stages,
                    p->alu_stage_latency);
    print_fu_config("num_mul_stages", "mul_stage_latencies", p->num_mul_stages,
                    p->mul_stage_latency);
    print_fu_config("num_div_stages", "div_stage_latencies", p->num_div_stages,
                    p->div_stage_latency);
    print_fu_config("num_fpu_alu_stages", "fpu_alu_stage_latencies",
                    p->num_fpu_alu_stages, p->fpu_alu_stage_latency);
    print_fu_config("num_fpu_fma_stages", "fpu_fma_stage_latencies",
                    p->num_fpu_fma_stages, p->fpu_fma_stage_latency);

    fprintf(stderr, "\n");

    fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "enable_bpu",
            sim_param_status[p->enable_bpu]);
    if (p->enable_bpu)
    {
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "btb_size",
                p->btb_size);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "btb_ways",
                p->btb_ways);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "btb_eviction_policy",
                btb_evict_str[p->btb_eviction_policy]);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "bpu_type",
                bpu_type_str[p->bpu_type]);

        if (p->bpu_type)
        {
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "bpu_ght_size",
                    p->bpu_ght_size);
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "bpu_pht_size",
                    p->bpu_pht_size);
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n",
                    "bpu_history_bits", p->bpu_history_bits);
            if ((p->bpu_ght_size == 1) && (p->bpu_pht_size == 1))
            {
                fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n",
                        "bpu_aliasing_func_type",
                        bpu_aliasing_func_type_str[p->bpu_aliasing_func_type]);
            }
        }
    }
    fprintf(stderr, "\n");

    /* L1-ICache */
    fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "enable_l1_caches",
            sim_param_status[p->enable_l1_caches]);
    if (p->enable_l1_caches)
    {
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n",
                "l1_code_cache_probe_latency", p->l1_code_cache_probe_latency);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d KB\n",
                "l1_code_cache_size", p->l1_code_cache_size);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "l1_code_cache_ways",
                p->l1_code_cache_ways);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "l1_code_cache_evict",
                cache_evict_str[p->l1_code_cache_evict]);

        /* L1-DCache */
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n",
                "l1_data_cache_probe_latency", p->l1_data_cache_probe_latency);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d KB\n",
                "l1_data_cache_size", p->l1_data_cache_size);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "l1_data_cache_ways",
                p->l1_data_cache_ways);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "l1_data_cache_evict",
                cache_evict_str[p->l1_data_cache_evict]);

        /* L2 Shared Cache Size */
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "enable_l2_cache",
                sim_param_status[p->enable_l2_cache]);

        if (p->enable_l2_cache)
        {
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n",
                    "l2_probe_latency", p->l2_probe_latency);
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d KB\n",
                    "l2_shared_cache_size", p->l2_shared_cache_size);
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n",
                    "l2_shared_cache_ways", p->l2_shared_cache_ways);
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n",
                    "l2_shared_cache_evict",
                    cache_evict_str[p->l2_shared_cache_evict]);
        }

        /* Common cache params */
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n",
                "words_per_cache_line", p->words_per_cache_line);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n",
                "cache_allocate_on_read_miss",
                cache_ra_str[p->cache_read_allocate_policy]);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n",
                "cache_allocate_on_write_miss",
                cache_wa_str[p->cache_write_allocate_policy]);
        fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "cache_write_policy",
                cache_wp_str[p->cache_write_policy]);
    }

    fprintf(stderr, "\n");
    fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %d\n", "tlb_size", p->tlb_size);
    fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %lu MB\n", "guest_ram_size", p->guest_ram_size);
    fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n", "mem_model_type",
            mem_model_type_str[p->mem_model_type]);

    switch (p->mem_model_type)
    {
        case MEM_MODEL_BASE:
        {
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %u\n",
                    "mem_bus_access_rtt_latency",
                    p->mem_bus_access_rtt_latency);
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %u\n", "tCL", p->tCL);
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %u\n", "tRCD", p->tRCD);
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %u\n", "tRP", p->tRP);
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %u\n",
                    "row_buffer_write_latency", p->row_buffer_write_latency);
            break;
        }
        case MEM_MODEL_DRAMSIM:
        {
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n",
                    "dramsim_ini_file", p->dramsim_ini_file);
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n",
                    "dramsim_system_ini_file", p->dramsim_system_ini_file);
            fprintf(stderr, " \x1B[32m*\x1B[0m %-30s : %s\n",
                    "dramsim_stats_dir", p->dramsim_stats_dir);
            break;
        }
        default:
        {
            fprintf(stderr, "error: invalid memory model\n");
            exit(1);
        }
    }
    fprintf(stderr, "\n");
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
    free(p->fpu_alu_stage_latency);
    p->fpu_alu_stage_latency = NULL;
    free(p->fpu_fma_stage_latency);
    p->fpu_fma_stage_latency = NULL;

    free(p->sim_trace_file);
    p->sim_trace_file = NULL;

    free(p->sim_stats_path);
    p->sim_stats_path = NULL;

    free(p->core_name);
    p->core_name = NULL;

    free(p->dramsim_ini_file);
    p->dramsim_ini_file = NULL;
    free(p->dramsim_system_ini_file);
    p->dramsim_system_ini_file = NULL;
    free(p->dramsim_stats_dir);
    p->dramsim_stats_dir = NULL;
}

static void
validate_param(const char *param_name, int has_range, int min, int max,
               int current)
{
    if (has_range)
    {
        if (!(current >= min && current <= max))
        {
            fprintf(stderr,
                    "(marss): config error - %s must be either %d or %d\n",
                    param_name, min, max);
            abort();
        }
    }
    else
    {
        if (!(current >= min))
        {
            fprintf(
                stderr,
                "(marss): config error - %s must have a minimum value of %d\n",
                param_name, min);
            abort();
        }
    }
}

static int
is_power_of_two(int value)
{
    return ((value > 0) && ((value & (value - 1)) == 0));
}

void
sim_params_validate(const SimParams *p)
{
    int i;

    /* Validate general config */
    validate_param("start_in_sim", 1, 0, 1, p->start_in_sim);
    validate_param("enable_stats_display", 1, 0, 1, p->enable_stats_display);
    validate_param("tlb_size", 0, 1, 2048, p->tlb_size);

    if (strcmp(p->core_name, "incore") == 0)
    {
        if (!(p->num_cpu_stages == 5 || p->num_cpu_stages == 6))
        {
            fprintf(stderr,
                    "(marss): error - num_cpu_stages must be either 5 or 6\n");
            abort();
        }
    }
    else if (strcmp(p->core_name, "oocore") == 0)
    {
        validate_param("prf_int_size", 0, 33, 2048, p->prf_int_size);
        validate_param("prf_fp_size", 0, 33, 2048, p->prf_fp_size);
        validate_param("iq_int_size", 0, 1, 2048, p->iq_int_size);
        validate_param("iq_fp_size", 0, 1, 2048, p->iq_fp_size);
        validate_param("iq_mem_size", 0, 1, 2048, p->iq_mem_size);
        validate_param("prf_int_write_ports", 0, 1, 2048,
                       p->prf_int_write_ports);
        validate_param("prf_fp_write_ports", 0, 1, 2048, p->prf_fp_write_ports);
        validate_param("rob_size", 0, 1, 2048, p->rob_size);
        validate_param("lsq_size", 0, 1, 2048, p->lsq_size);
    }

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
    for (i = 0; i < p->num_fpu_alu_stages; ++i)
    {
        validate_param("fpu_alu_stage_latency", 0, 1, 2048,
                       p->fpu_alu_stage_latency[i]);
    }

    validate_param("num_fpu_fma_stages", 0, 1, 2048, p->num_fpu_fma_stages);
    for (i = 0; i < p->num_fpu_fma_stages; ++i)
    {
        validate_param("fpu_fma_stage_latency", 0, 1, 2048,
                       p->fpu_fma_stage_latency[i]);
    }

    /* Validate BPU config */
    validate_param("enable_bpu", 1, 0, 1, p->enable_bpu);
    if (p->enable_bpu)
    {
        if (!is_power_of_two(p->btb_size))
        {
            fprintf(stderr, "(marss): config error - %s must be a power of 2\n",
                    "btb_size");
            abort();
        }
        validate_param("btb_ways", 0, 1, 2048, p->btb_ways);
        validate_param("bpu_type", 1, 0, 1, p->bpu_type);

        if (p->bpu_type)
        {
            if (!is_power_of_two(p->bpu_ght_size))
            {
                fprintf(stderr,
                        "(marss): config error - %s must be a power of 2\n",
                        "bpu_ght_size");
                abort();
            }
            if (!is_power_of_two(p->bpu_pht_size))
            {
                fprintf(stderr,
                        "(marss): config error - %s must be a power of 2\n",
                        "bpu_pht_size");
                abort();
            }
            validate_param("bpu_history_bits", 0, 1, 2048, p->bpu_history_bits);
        }
    }

    /* Validate caches */
    validate_param("enable_l1_caches", 1, 0, 1, p->enable_l1_caches);

    if (p->enable_l1_caches)
    {
        validate_param("l1_code_cache_probe_latency", 0, 1, 2048,
                       p->l1_code_cache_probe_latency);
        validate_param("l1_code_cache_size", 0, 1, 2048, p->l1_code_cache_size);
        validate_param("l1_code_cache_ways", 0, 1, 2048, p->l1_code_cache_ways);
        validate_param("l1_code_cache_evict", 1, 0, 1, p->l1_code_cache_evict);

        validate_param("l1_data_cache_probe_latency", 0, 1, 2048,
                       p->l1_data_cache_probe_latency);
        validate_param("l1_data_cache_size", 0, 1, 2048, p->l1_data_cache_size);
        validate_param("l1_data_cache_ways", 0, 1, 2048, p->l1_data_cache_ways);
        validate_param("l1_data_cache_evict", 1, 0, 1, p->l1_data_cache_evict);

        /* validate common parameters */
        validate_param("words_per_cache_line", 0, 1, 2048,
                       p->words_per_cache_line);
        validate_param("cache_read_allocate_policy", 1, 0, 1,
                       p->cache_read_allocate_policy);
        validate_param("cache_write_allocate_policy", 1, 0, 1,
                       p->cache_write_allocate_policy);
        validate_param("cache_write_policy", 1, 0, 1, p->cache_write_policy);

        validate_param("enable_l2_cache", 1, 0, 1, p->enable_l2_cache);

        if (p->enable_l2_cache)
        {
            validate_param("l2_probe_latency", 0, 1, 2048, p->l2_probe_latency);
            validate_param("l2_shared_cache_size", 0, 1, 2048,
                           p->l2_shared_cache_size);
            validate_param("l2_shared_cache_ways", 0, 1, 2048,
                           p->l2_shared_cache_ways);
            validate_param("l2_shared_cache_evict", 1, 0, 1,
                           p->l2_shared_cache_evict);
        }
    }

    // TODO: Data bus transaction size must be power of 2
    validate_param("dram_burst_size", 0, 1, 2048, (int)p->dram_burst_size);
    validate_param("mem_bus_access_rtt_latency", 0, 0, 2048,
                   p->mem_bus_access_rtt_latency);
    validate_param("tCL", 0, 1, 2048, p->tCL);
    validate_param("tRCD", 0, 1, 2048, p->tRCD);
    validate_param("tRP", 0, 1, 2048, p->tRP);
    validate_param("row_buffer_write_latency", 0, 1, 2048,
                   p->row_buffer_write_latency);
}

void
sim_stats_print(SimStats *s, const char *pathname)
{
    FILE *fp;
    char *filename, *p;
    time_t rawtime;
    char buffer[256];

    time(&rawtime);
    sprintf(buffer, "simstats_%s.txt", ctime(&rawtime));

    p = buffer;
    for (; *p; ++p)
    {
        if (*p == ' ')
            *p = '_';

        if (*p == '\n')
            *p = '_';
    }

    filename = (char *)malloc(strlen(pathname) + strlen(buffer) + 2);
    assert(filename);

    strcpy(filename, pathname);
    strcat(filename, "/");
    strcat(filename, buffer);

    fp = fopen(filename, "w");
    assert(fp);

    PRINT_SIM_STAT_HEADER(fp);

    PRINT_SIM_STAT(fp, s, "cycles", cycles);
    PRINT_SIM_STAT(fp, s, "insn_fetched", ins_fetch);
    PRINT_SIM_STAT(fp, s, "insn_simulated", ins_simulated);

    PRINT_SIM_STAT(fp, s, "operate_insn", ins_type[INS_TYPE_OPERATE]);
    PRINT_SIM_STAT(fp, s, "load_insn", ins_type[INS_TYPE_LOAD]);
    PRINT_SIM_STAT(fp, s, "store_insn", ins_type[INS_TYPE_STORE]);
    PRINT_SIM_STAT(fp, s, "atomic_insn", ins_type[INS_TYPE_ATOMIC]);
    PRINT_SIM_STAT(fp, s, "cond_branches", ins_type[INS_TYPE_COND_BRANCH]);
    PRINT_SIM_STAT(fp, s, "cond_branches_taken", ins_cond_branch_taken);
    PRINT_SIM_STAT(fp, s, "uncond_branches", ins_type[INS_TYPE_UNCOND_BRANCH]);
    PRINT_SIM_STAT(fp, s, "system_insn", ins_emulated);

    // PRINT_SIM_STAT(fp, s, "itlb_reads", code_tlb_lookups);
    // PRINT_SIM_STAT(fp, s, "itlb_hits", code_tlb_hits);
    // PRINT_SIM_STAT(fp, s, "load_tlb_reads", load_tlb_lookups);
    // PRINT_SIM_STAT(fp, s, "load_tlb_hits", load_tlb_hits);
    // PRINT_SIM_STAT(fp, s, "store_tlb_reads", store_tlb_lookups);
    // PRINT_SIM_STAT(fp, s, "store_tlb_hits", store_tlb_hits);

    PRINT_SIM_STAT(fp, s, "cond_branches_pred_correct", bpu_cond_correct);
    PRINT_SIM_STAT(fp, s, "cond_branches_pred_incorrect", bpu_cond_incorrect);
    PRINT_SIM_STAT(fp, s, "uncond_branches_pred_correct", bpu_uncond_correct);
    PRINT_SIM_STAT(fp, s, "uncond_branches_pred_incorrect",bpu_uncond_incorrect);

    PRINT_SIM_STAT(fp, s, "btb_reads", btb_probes);
    PRINT_SIM_STAT(fp, s, "btb_hits", btb_hits);
    PRINT_SIM_STAT(fp, s, "btb_inserts", btb_inserts);
    PRINT_SIM_STAT(fp, s, "btb_updates", btb_updates);

    PRINT_SIM_STAT(fp, s, "int_regfile_reads", int_regfile_reads);
    PRINT_SIM_STAT(fp, s, "int_regfile_writes", int_regfile_writes);
    PRINT_SIM_STAT(fp, s, "fp_regfile_reads", fp_regfile_reads);
    PRINT_SIM_STAT(fp, s, "fp_regfile_writes", fp_regfile_writes);
    PRINT_SIM_STAT(fp, s, "csr_reads", csr_reads);
    PRINT_SIM_STAT(fp, s, "csr_writes", csr_writes);

    PRINT_SIM_STAT(fp, s, "fu_alu_accesses", fu_access[FU_ALU]);
    PRINT_SIM_STAT(fp, s, "fu_mul_accesses", fu_access[FU_MUL]);
    PRINT_SIM_STAT(fp, s, "fu_div_accesses", fu_access[FU_DIV]);
    PRINT_SIM_STAT(fp, s, "fu_fpu_alu_accesses", fu_access[FU_FPU_ALU]);
    PRINT_SIM_STAT(fp, s, "fu_fpu_fma_accesses", fu_access[FU_FPU_FMA]);

    PRINT_SIM_STAT(fp, s, "ins_page_faults", ins_page_faults);
    PRINT_SIM_STAT(fp, s, "load_page_faults", load_page_faults);
    PRINT_SIM_STAT(fp, s, "store_page_faults", store_page_faults);

    PRINT_SIM_STAT(fp, s, "L1_icache_reads", l1i_read);
    PRINT_SIM_STAT(fp, s, "L1_icache_read_misses", l1i_read_miss);

    PRINT_SIM_STAT(fp, s, "L1_dcache_reads", l1d_read);
    PRINT_SIM_STAT(fp, s, "L1_dcache_read_misses", l1d_read_miss);

    PRINT_SIM_STAT(fp, s, "L1_dcache_writes", l1d_write);
    PRINT_SIM_STAT(fp, s, "L1_dcache_write_misses", l1d_write_miss);

    PRINT_SIM_STAT(fp, s, "L2_cache_reads", l2_read);
    PRINT_SIM_STAT(fp, s, "L2_cache_read_misses", l2_read_miss);
    PRINT_SIM_STAT(fp, s, "L2_cache_writes", l2_write);
    PRINT_SIM_STAT(fp, s, "L2_cache_write_misses", l2_write_miss);

    fclose(fp);
    fprintf(stderr, "(marss-riscv): Saved stats in %s\n", filename);
    free(filename);
}

void
sim_stats_reset(SimStats *s)
{
    memset((void *)s, 0, NUM_MAX_PRV_LEVELS * sizeof(SimStats));
}
