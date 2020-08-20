/**
 * RISCV Simulated CPU State
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
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "../../riscv_cpu_priv.h"
#include "../memory_hierarchy/dramsim_wrapper_c_connector.h"
#include "../utils/sim_log.h"
#include "../utils/sim_stats.h"
#include "inorder.h"
#include "ooo.h"
#include "riscv_sim_cpu.h"

#define WRITE_STATS_TO_SHM_CLOCK_CYCLES_INTERVAL 500000
#define GET_TIME(time) clock_gettime(CLOCK_MONOTONIC, &time)
#define GET_TIMER_DIFF(start, end)                                             \
    (1000000000L * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec)

static void
bpu_enabled_fetch_stage_handler(struct RISCVCPUState *s, InstructionLatch *e)
{
    target_ulong bpu_target;

    bpu_target = 0;
    bpu_probe(s->simcpu->bpu, e->ins.pc, &e->bpu_resp_pkt, s->priv);
    if (e->bpu_resp_pkt.bpu_probe_status)
    {
        bpu_target = bpu_get_target(s->simcpu->bpu, e->ins.pc,
                                    e->bpu_resp_pkt.btb_entry);

        /* Non-zero target means branch is taken, according to the prediction,
         * so set the predicted address into pcgen unit */
        if (bpu_target)
        {
            s->code_ptr = NULL;
            s->code_end = NULL;
            s->code_to_pc_addend = bpu_target;
        }
    }

    /* Keep track of the predicted address and probe status, to correct
     * miss-prediction later */
    e->predicted_target = bpu_target;
}

static void
bpu_disabled_fetch_stage_handler(struct RISCVCPUState *s, InstructionLatch *e)
{
    /* In the absence of BPU, no actions required */
    return;
}

static int
bpu_enabled_decode_stage_handler(struct RISCVCPUState *s, InstructionLatch *e)
{
    target_ulong ras_target = 0;

    /* Add the branch PC into BPU structures if probe during fetch results in
     * miss */
    if (!e->bpu_resp_pkt.bpu_probe_status)
    {
        bpu_add(s->simcpu->bpu, e->ins.pc, e->ins.branch_type, &e->bpu_resp_pkt,
                s->priv, e->ins.is_func_ret);
    }

    /* If return address stack is enabled */
    if (s->simcpu->params->ras_size)
    {
        if (e->ins.is_func_call)
        {
            ras_push(
                s->simcpu->bpu->ras,
                ((e->ins.binary & 3) == 3 ? e->ins.pc + 4 : e->ins.pc + 2));
        }

        if (e->ins.is_func_ret)
        {
            ras_target = ras_pop(s->simcpu->bpu->ras);

            /* Start fetch from address returned by RAS if non-zero */
            if (ras_target)
            {
                s->code_ptr = NULL;
                s->code_end = NULL;
                s->code_to_pc_addend = ras_target;
                e->predicted_target = ras_target;

                mem_controller_reset_cpu_stage_queue(
                    &s->simcpu->mem_hierarchy->mem_controller
                         ->frontend_mem_access_queue);

                /* Signal the calling stage to flush previous stages */
                return TRUE;
            }
        }
    }

    /* No flush required because no redirect by RAS */
    return FALSE;
}

static int
bpu_disabled_decode_stage_handler(struct RISCVCPUState *s, InstructionLatch *e)
{
    return FALSE;
}

/* Handles conditional branches with branch prediction enabled, probes the BPU
 * and updates the entry if BPU hit, corrects the control-flow in the case of
 * direction miss-prediction */
static int
bpu_enabled_conditional_branch_handler(RISCVCPUState *s, InstructionLatch *e)
{
    target_ulong restore_pc;
    int pred = 0;
    int mispredict = FALSE;
    RISCVSIMCPUState *simcpu = s->simcpu;

    bpu_probe(simcpu->bpu, e->ins.pc, &e->bpu_resp_pkt, s->priv);

    if (e->ins.cond)
    {
        /* Branch is resolved to be taken*/
        pred = TRUE;
        if (!e->predicted_target)
        {
            ++simcpu->stats[s->priv].bpu_cond_incorrect;
            e->branch_target = e->ins.target;
            mispredict = TRUE;
        }
        else
        {
            sim_assert((e->predicted_target == e->ins.target),
                       "error: %s at line %d in %s(): %s", __FILE__, __LINE__,
                       __func__, "predicted branch target should be equal to "
                                 "the computed branch target on correct "
                                 "prediction");

            e->is_pred_correct = TRUE;
            e->branch_target = e->predicted_target;
            ++simcpu->stats[s->priv].bpu_cond_correct;
        }
        e->is_branch_taken = TRUE;
    }
    else
    {
        /* Branch resolved to be not-taken */
        if (e->predicted_target)
        {
            /* Miss-prediction occurred, flush the pipeline, repair
               the control flow */
            restore_pc
                = ((e->ins.binary & 3) != 3) ? e->ins.pc + 2 : e->ins.pc + 4;
            ++simcpu->stats[s->priv].bpu_cond_incorrect;
            e->branch_target = restore_pc;
            mispredict = TRUE;
        }
        else
        {
            /* Correct Prediction */
            e->is_pred_correct = TRUE;
            ++simcpu->stats[s->priv].bpu_cond_correct;
        }
        e->is_branch_taken = FALSE;
    }

    /* Update BPU if hit, else skip the update */
    bpu_update(simcpu->bpu, e->ins.pc, e->ins.target, pred, BRANCH_COND,
               &e->bpu_resp_pkt, s->priv);

    return mispredict;
}

/* Handles unconditional branches with branch prediction enabled, probes the BPU
 * and updates the entry if BPU hit, corrects the control-flow in the case of
 * target miss-prediction */
static int
bpu_enabled_unconditional_branch_handler(RISCVCPUState *s, InstructionLatch *e)
{
    RISCVSIMCPUState *simcpu = s->simcpu;
    int type = BRANCH_UNCOND;
    int mispredict = FALSE;
    bpu_probe(simcpu->bpu, e->ins.pc, &e->bpu_resp_pkt, s->priv);

    /* Update BPU if hit, else skip the update */
    bpu_update(simcpu->bpu, e->ins.pc, e->ins.target, TRUE, type,
               &e->bpu_resp_pkt, s->priv);

    if (e->predicted_target == e->ins.target)
    {
        /* Prediction Success */
        e->is_pred_correct = TRUE;

        /* Keep track of this branch to resume at this target, in case
           the timeout happens after this branch commits */
        e->branch_target = e->predicted_target;
        ++simcpu->stats[s->priv].bpu_uncond_correct;
    }
    else
    {
        /* Target Miss-prediction */
        ++simcpu->stats[s->priv].bpu_uncond_incorrect;
        e->branch_target = e->ins.target;
        mispredict = TRUE;
    }

    e->is_branch_taken = TRUE;
    return mispredict;
}

static int
bpu_enabled_execute_stage_handler(struct RISCVCPUState *s, InstructionLatch *e)
{
    switch (e->ins.branch_type)
    {
        case BRANCH_UNCOND:
        {
            return bpu_enabled_unconditional_branch_handler(s, e);
        }
        case BRANCH_COND:
        {
            return bpu_enabled_conditional_branch_handler(s, e);
        }
    }

    return 0;
}

static int
bpu_disabled_execute_stage_handler(struct RISCVCPUState *s, InstructionLatch *e)
{
    int mispredict = FALSE;

    switch (e->ins.branch_type)
    {
        case BRANCH_UNCOND:
        {
            e->is_branch_taken = TRUE;
            e->branch_target = e->ins.target;
            mispredict = TRUE;
            break;
        }
        case BRANCH_COND:
        {
            if (e->ins.cond)
            {
                e->is_branch_taken = TRUE;
                e->branch_target = e->ins.target;
                mispredict = TRUE;
            }
            break;
        }
    }

    return mispredict;
}

static void
copy_cache_stats_to_global_stats(RISCVSIMCPUState *simcpu)
{
    int i;
    const CacheStats *cache_stats;

    /* Update cache stats */
    if (simcpu->params->enable_l1_caches)
    {
        for (i = 0; i < NUM_MAX_PRV_LEVELS; ++i)
        {
            cache_stats = cache_get_stats(simcpu->mem_hierarchy->icache);
            simcpu->stats[i].icache_read = cache_stats[i].total_read_cnt;
            simcpu->stats[i].icache_read_miss = cache_stats[i].read_miss_cnt;

            cache_stats = cache_get_stats(simcpu->mem_hierarchy->dcache);
            simcpu->stats[i].dcache_read = cache_stats[i].total_read_cnt;
            simcpu->stats[i].dcache_read_miss = cache_stats[i].read_miss_cnt;
            simcpu->stats[i].dcache_write = cache_stats[i].total_write_cnt;
            simcpu->stats[i].dcache_write_miss = cache_stats[i].write_miss_cnt;

            if (simcpu->params->enable_l2_cache)
            {
                cache_stats = cache_get_stats(simcpu->mem_hierarchy->l2_cache);
                simcpu->stats[i].l2_cache_read = cache_stats[i].total_read_cnt;
                simcpu->stats[i].l2_cache_read_miss
                    = cache_stats[i].read_miss_cnt;
                simcpu->stats[i].l2_cache_write
                    = cache_stats[i].total_write_cnt;
                simcpu->stats[i].l2_cache_write_miss
                    = cache_stats[i].write_miss_cnt;
            }
        }
    }
}

/* Setup shared memory to dump stats, read by sim-stats-display tool */
static void
setup_stats_shm(RISCVSIMCPUState *simcpu)
{
    int stats_shm_fd;

    stats_shm_fd
        = shm_open(simcpu->params->sim_stats_shm_name, O_RDWR | O_CREAT, ALL_RW_PERMS);
    if (stats_shm_fd < 0)
    {
        fprintf(stderr,
                "error: cannot create marss-stats-shm %s, terminating\n",
                simcpu->params->sim_stats_shm_name);
        exit(1);
    }

    if (ftruncate(stats_shm_fd, NUM_MAX_PRV_LEVELS * sizeof(SimStats)) < 0)
    {
        fprintf(stderr,
                "error: cannot resize marss-stats-shm %s, terminating\n",
                simcpu->params->sim_stats_shm_name);
        close(stats_shm_fd);
        exit(1);
    }

    simcpu->stats_shm_ptr = NULL;
    if ((simcpu->stats_shm_ptr = (SimStats *)mmap(
             NULL, NUM_MAX_PRV_LEVELS * sizeof(SimStats),
             PROT_READ | PROT_WRITE, MAP_SHARED, stats_shm_fd, 0))
        == MAP_FAILED)
    {
        fprintf(stderr, "error: cannot mmap shm %s, terminating",
                simcpu->params->sim_stats_shm_name);
        close(stats_shm_fd);
        exit(1);
    }

    memset(simcpu->stats_shm_ptr, 0, NUM_MAX_PRV_LEVELS * sizeof(SimStats));
    sim_log_event_to_file(sim_log, "%s", "Setting up shared memory to write stats");
    sim_log_param_to_file(sim_log, "%s: %s", "posix shared memory name", simcpu->params->sim_stats_shm_name);
}

static void
print_performance_summary(RISCVSIMCPUState *simcpu, uint64_t sim_time)
{
    int i;

    sim_log_event(sim_log, "%s", "Performance Summary:");

    for (i = 0; i < NUM_MAX_PRV_LEVELS; ++i)
    {
        sim_log_param(sim_log, "%s-mode-commits: %lu", cpu_mode_str[i],
                      simcpu->stats[i].ins_simulated);
        sim_log_param(sim_log, "%s-mode-cycles: %lu", cpu_mode_str[i],
                      simcpu->stats[i].cycles);
        sim_log_param(sim_log, "%s-mode-ipc: %.4lf", cpu_mode_str[i],
                      (double)simcpu->stats[i].ins_simulated
                          / (double)simcpu->stats[i].cycles);
    }

    sim_log_param(sim_log, "total-commits: %lu", simcpu->icount);
    sim_log_param(sim_log, "total-cycles: %lu", simcpu->clock);
    sim_log_param(sim_log, "total-ipc: %.4lf",
                  (double)simcpu->icount / (double)simcpu->clock);
    sim_log_param(sim_log, "simulation-time: %lu seconds", sim_time);
    sim_log_param(sim_log, "total-commits-per-second: %lu",
                  simcpu->icount / sim_time);
}

void
update_arch_reg_int(RISCVCPUState *s, InstructionLatch *e)
{
    if (e->ins.rd)
    {
        s->reg[e->ins.rd] = e->ins.buffer;
        ++s->simcpu->stats[s->priv].int_regfile_writes;
    }
}

void
update_arch_reg_fp(RISCVCPUState *s, InstructionLatch *e)
{
    if (e->ins.f32_mask)
    {
        e->ins.buffer |= F32_HIGH;
    }
    else if (e->ins.f64_mask)
    {
        e->ins.buffer |= F64_HIGH;
    }
    s->fp_reg[e->ins.rd] = e->ins.buffer;
    if (e->ins.set_fs)
    {
        s->fs = 3;
    }
    ++s->simcpu->stats[s->priv].fp_regfile_writes;
}

void
update_insn_commit_stats(RISCVCPUState *s, InstructionLatch *e)
{
    ++s->simcpu->icount;
    ++s->simcpu->stats[s->priv].ins_simulated;
    ++s->simcpu->stats[s->priv].ins_type[e->ins.type];

    if ((e->ins.type == INS_TYPE_COND_BRANCH) && e->is_branch_taken)
    {
        ++s->simcpu->stats[s->priv].ins_cond_branch_taken;
    }
}

void
write_stats_to_stats_display_shm(RISCVSIMCPUState *simcpu)
{
    if ((simcpu->clock % WRITE_STATS_TO_SHM_CLOCK_CYCLES_INTERVAL) == 0)
    {
        /* Since cache stats are stored separately inside the Cache structure,
         * they have to be copied to global stats structure before writing stats
         * to shared memory. */
        copy_cache_stats_to_global_stats(simcpu);
        memcpy(simcpu->stats_shm_ptr, simcpu->stats,
               NUM_MAX_PRV_LEVELS * sizeof(SimStats));
    }
}

int
set_max_clock_cycles_for_non_pipe_fu(RISCVCPUState *s, int fu_type,
                                     InstructionLatch *e)
{
    switch (fu_type)
    {
        case FU_FPU_ALU:
        {
            return s->simcpu->params->fpu_alu_latency[e->ins.fpu_alu_type];
        }
    }

    /* Default */
    return 1;
}

/* Read the instruction from TinyEMU memory map into the instruction latch */
void
fetch_cpu_stage_exec(RISCVCPUState *s, InstructionLatch *e)
{
    e->max_clock_cycles = 1;
    s->hw_pg_tb_wlk_stage_id = FETCH;
    s->ins_tlb_lookup_accounted = FALSE;
    s->ins_tlb_hit_accounted = FALSE;

    /* Reset page walk delay before fetching current instruction. This is the
     * cache hierarchy lookup delay for page table entries, on a TLB miss */
    s->simcpu->mem_hierarchy->mem_controller->page_walk_delay = 0;

    /* elasped_clock_cycles: number of CPU cycles spent by this instruction
     * in fetch stage so far */
    e->elasped_clock_cycles = 1;
    s->simcpu->mem_hierarchy->mem_controller->frontend_mem_access_queue.cur_size
        = 0;

    /* Fetch instruction from TinyEMU memory map */
    if (s->simcpu->temu_mem_map_wrapper->read_insn(s, e))
    {
        /* This instruction has raised a page fault exception during
         * fetch */
        e->ins.exception = TRUE;
        e->ins.exception_cause = SIM_MMU_EXCEPTION;
    }
    else
    {
        /* max_clock_cycles: Number of CPU cycles required for TLB and Cache
         * look-up */
        e->max_clock_cycles
            = s->simcpu->mem_hierarchy->mem_controller->page_walk_delay
              + s->simcpu->mem_hierarchy->insn_read_delay(
                    s->simcpu->mem_hierarchy, s->code_guest_paddr, 4, FETCH,
                    s->priv);

        sim_assert((e->max_clock_cycles), "error: %s at line %d in %s(): %s",
                   __FILE__, __LINE__, __func__,
                   "max_clock_cycles execution latency for an instruction "
                   "must be non_zero");

        /* Increment PC for the next instruction */
        if (3 == (e->ins.binary & 3))
        {
            s->code_ptr = s->code_ptr + 4;
            s->code_guest_paddr = s->code_guest_paddr + 4;
        }
        else
        {
            /* For compressed */
            s->code_ptr = s->code_ptr + 2;
            s->code_guest_paddr = s->code_guest_paddr + 2;
        }

        /* Probe the branch predictor */
        s->simcpu->bpu_fetch_stage_handler(s, e);

        ++s->simcpu->stats[s->priv].ins_fetch;
    }
}

void
decode_cpu_stage_exec(RISCVCPUState *s, InstructionLatch *e)
{
    /* For decoding floating point instructions */
    e->ins.current_fs = s->fs;
    e->ins.rm = get_insn_rm(s, (e->ins.binary >> 12) & 7);

    /* Decode the instruction */
    decode_riscv_binary(&e->ins, e->ins.binary);
}

/* Read/Write data to/from TinyEMU memory map into the instruction latch and set
 * latency in clock cycles for the access */
void
mem_cpu_stage_exec(RISCVCPUState *s, InstructionLatch *e)
{
    e->max_clock_cycles = 1;
    s->hw_pg_tb_wlk_stage_id = MEMORY;

    /* Reset page walk delay before executing current memory instruction. This
     * is the cache hierarchy lookup delay for page table entries, on a TLB miss */
    s->simcpu->mem_hierarchy->mem_controller->page_walk_delay = 0;

    if (s->simcpu->temu_mem_map_wrapper->exec_load_store_atomic(s, e))
    {
        /* This load, store or atomic instruction raised a page
         * fault exception */
        e->ins.exception = TRUE;
        e->ins.exception_cause = SIM_MMU_EXCEPTION;
    }
    else
    {
        /* Memory access was successful, no page fault, so calculate the memory
         * access latency */
        e->max_clock_cycles
            = s->simcpu->mem_hierarchy->mem_controller->page_walk_delay;

        if (s->is_device_io || !s->data_guest_paddr)
        {
            /* This was a non RAM access, probably a device, so set
             * max_clock_cycles to 1 */
            e->max_clock_cycles += 1;
        }
        else
        {
            /* RAM access */
            if ((e->ins.is_load || e->ins.is_atomic_load))
            {
                e->max_clock_cycles
                    += s->simcpu->mem_hierarchy->data_read_delay(
                        s->simcpu->mem_hierarchy, s->data_guest_paddr,
                        e->ins.bytes_to_rw, MEMORY, s->priv);
            }

            if ((e->ins.is_store || e->ins.is_atomic_store))
            {
                e->max_clock_cycles
                    += s->simcpu->mem_hierarchy->data_write_delay(
                        s->simcpu->mem_hierarchy, s->data_guest_paddr,
                        e->ins.bytes_to_rw, MEMORY, s->priv);
            }
        }

        sim_assert((e->max_clock_cycles), "error: %s at line %d in %s(): %s",
                   __FILE__, __LINE__, __func__,
                   "max_clock_cycles execution latency for an instruction "
                   "must be non_zero");
    }
}

void
riscv_sim_cpu_start(RISCVSIMCPUState *simcpu, target_ulong pc)
{
    if (!simcpu->simulation)
    {
        simcpu->simulation = TRUE;
        simcpu->clock = 0;
        simcpu->icount = 0;

        sim_stats_reset(simcpu->stats);
        GET_TIME(simcpu->sim_start_time);

        /* Reset BPU at every new simulation run */
        if (simcpu->params->enable_bpu && simcpu->params->flush_bpu_on_simstart)
        {
            bpu_flush(simcpu->bpu);
        }

        /* Reset Caches at every new simulation run */
        if (simcpu->params->enable_l1_caches)
        {
            cache_reset_stats(simcpu->mem_hierarchy->icache);
            cache_reset_stats(simcpu->mem_hierarchy->dcache);

            if (simcpu->params->flush_sim_mem_on_simstart)
            {
                cache_flush(simcpu->mem_hierarchy->icache);
                cache_flush(simcpu->mem_hierarchy->dcache);
            }

            if (simcpu->params->enable_l2_cache)
            {
                cache_reset_stats(simcpu->mem_hierarchy->l2_cache);

                if (simcpu->params->flush_sim_mem_on_simstart)
                {
                    cache_flush(simcpu->mem_hierarchy->l2_cache);
                }
            }
        }

        /* Reset DRAMs at every new simulation run */
        switch (simcpu->mem_hierarchy->mem_controller->dram_model_type)
        {
            case MEM_MODEL_BASE:
            {
                break;
            }
            case MEM_MODEL_DRAMSIM:
            {
                dramsim_wrapper_destroy();
                dramsim_wrapper_init(simcpu->params->dramsim_config_file,
                                     simcpu->params->sim_file_path);
                break;
            }
        }

        /* Open trace file if running in trace mode */
        if (simcpu->params->do_sim_trace)
        {
            simcpu->params->create_ins_str = TRUE;
            sim_log_event(sim_log, "Starting simulation trace "
                                   "at pc = 0x%" PR_target_ulong " in file: %s",
                          pc, simcpu->params->sim_trace_file);
            sim_trace_start(simcpu->trace, simcpu->params->sim_trace_file);
        }

        sim_log_event(sim_log, "Switching to full-system simulation "
                               "mode at pc = 0x%" PR_target_ulong,
                      pc);
    }
}

void
riscv_sim_cpu_stop(RISCVSIMCPUState *simcpu, target_ulong pc)
{
    uint64_t sim_time;

    if (simcpu->simulation)
    {
        simcpu->simulation = FALSE;
        GET_TIME(simcpu->sim_end_time);
        sim_time = GET_TIMER_DIFF(simcpu->sim_start_time, simcpu->sim_end_time)
                   / 1000000000;

        print_performance_summary(simcpu, sim_time);

        if (simcpu->mem_hierarchy->mem_controller->dram_model_type
            == MEM_MODEL_DRAMSIM)
        {
            dramsim_wrapper_print_stats();
        }

        if (simcpu->params->do_sim_trace)
        {
            sim_trace_stop(simcpu->trace);
            sim_log_event(sim_log, "Saved simulation trace in %s",
                          simcpu->params->sim_trace_file);
        }

        copy_cache_stats_to_global_stats(simcpu);
        sim_stats_print_to_file(simcpu->stats, simcpu->params->sim_file_path,
                                simcpu->params->sim_file_prefix, sim_time);

        sim_log_event(sim_log, "Switching to emulation mode "
                               "mode at pc = 0x%" PR_target_ulong,
                      pc);
    }
}

void
riscv_sim_cpu_reset(RISCVSIMCPUState *simcpu)
{
    simcpu->exception->pending = FALSE;
    reset_insn_latch_pool(simcpu->insn_latch_pool);
    mem_controller_reset(simcpu->mem_hierarchy->mem_controller);
    simcpu->core_reset(simcpu->core);
}

int
riscv_sim_cpu_switch_to_cpu_simulation(RISCVSIMCPUState *simcpu)
{
    riscv_sim_cpu_reset(simcpu);
    return simcpu->core_run(simcpu->core);
}

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
    simcpu->return_to_sim = FALSE;

    simcpu->stats = (SimStats *)calloc(NUM_MAX_PRV_LEVELS, sizeof(SimStats));
    assert(simcpu->stats != NULL);

    simcpu->insn_latch_pool = (InstructionLatch *)calloc(
        INSN_LATCH_POOL_SIZE, sizeof(InstructionLatch));
    assert(simcpu->insn_latch_pool);

    sim_params_log_options(p);

    switch (p->core_type)
    {
        case CORE_TYPE_INCORE:
        {
            simcpu->core = (void *)in_core_init(simcpu->params, simcpu);
            simcpu->core_reset = in_core_reset;
            simcpu->core_run = in_core_run;
            simcpu->core_free = in_core_free;
            break;
        }
        case CORE_TYPE_OOCORE:
        {
            simcpu->core = (void *)oo_core_init(simcpu->params, simcpu);
            simcpu->core_reset = oo_core_reset;
            simcpu->core_run = oo_core_run;
            simcpu->core_free = oo_core_free;
            break;
        }
    }

    sim_params_log_exec_unit_config(p);

    simcpu->mem_hierarchy = memory_hierarchy_init(simcpu->params, sim_log);

    /* Seed for random eviction, if used in BPU and caches */
    srand(time(NULL));

    if (p->enable_bpu)
    {
        simcpu->bpu = bpu_init(p, simcpu->stats);
        simcpu->bpu_fetch_stage_handler = &bpu_enabled_fetch_stage_handler;
        simcpu->bpu_decode_stage_handler = &bpu_enabled_decode_stage_handler;
        simcpu->bpu_execute_stage_handler = &bpu_enabled_execute_stage_handler;
    }
    else
    {
        simcpu->bpu_fetch_stage_handler = &bpu_disabled_fetch_stage_handler;
        simcpu->bpu_decode_stage_handler = &bpu_disabled_decode_stage_handler;
        simcpu->bpu_execute_stage_handler = &bpu_disabled_execute_stage_handler;
    }

    simcpu->temu_mem_map_wrapper = temu_mem_map_wrapper_init();
    simcpu->exception = sim_exception_init();
    simcpu->trace = sim_trace_init();

    if (p->enable_stats_display)
    {
        setup_stats_shm(simcpu);
    }

    /* We are booting TinyEMU in simulation mode */
    if (p->start_in_sim)
    {
        riscv_sim_cpu_start(simcpu, simcpu->pc);
    }

    sim_assert((sim_file_path_valid(p->sim_file_path)),
               "error: %s at line %d in %s(): %s", __FILE__, __LINE__, __func__,
               "top-level stats directory not found");
    return simcpu;
}

void
riscv_sim_cpu_free(RISCVSIMCPUState **simcpu)
{
    free((*simcpu)->stats);
    (*simcpu)->stats = NULL;

    free((*simcpu)->insn_latch_pool);
    (*simcpu)->insn_latch_pool = NULL;

    memory_hierarchy_free(&((*simcpu)->mem_hierarchy));

    if ((*simcpu)->params->enable_bpu)
    {
        bpu_free(&((*simcpu)->bpu));
    }

    (*simcpu)->core_free(&(*simcpu)->core);
    temu_mem_map_wrapper_free(&(*simcpu)->temu_mem_map_wrapper);
    sim_exception_free(&(*simcpu)->exception);
    sim_trace_free(&(*simcpu)->trace);
    free(*simcpu);
}
