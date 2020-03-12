/*
 * Terminal based Simulation Statistics Viewer
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2017-2019 Gaurav Kothari {gkothar1@binghamton.edu}
 * State University of New York at Binghamton
 *
 * Copyright (c) 2018-2019 Parikshit Sarnaik {psarnai1@binghamton.edu}
 * State University of New York at Binghamton
 *
 * Copyright (c) 2018 Göktürk Yüksek {gokturk@binghamton.edu}
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
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "riscvsim/sim_params_stats.h"

#define GET_TOTAL_STAT(attr) (s[0].attr + s[1].attr + s[2].attr + s[3].attr)

static int stats_shm_fd;
static SimStats *s;

static void
setup_connection()
{
    unsigned long nsec_passed = 0;
    unsigned noent_print = 0;
retry:
    stats_shm_fd = shm_open(MARSS_STATS_SHM_NAME, O_RDWR, 0);
    if (stats_shm_fd < 0)
    {
        if (errno == ENOENT)
        {
            const unsigned long nsec = 1000000;
            struct timespec ts = {.tv_sec = 0, .tv_nsec = nsec};
            if (noent_print == 0)
            {
                fprintf(stderr, "cannot open shm %s,"
                                " retrying every %ld nanoseconds ",
                        MARSS_STATS_SHM_NAME, nsec);
                ++noent_print;
            }
            else if (nsec_passed > 1000000000)
            {
                fprintf(stderr, ".");
                ++noent_print;
                nsec_passed = 0;
            }
            nsec_passed += nsec;
            nanosleep(&ts, NULL);
            goto retry;
        }
        else
        {
            perror("setup_connection(): "
                   "Unable to establish the shared memory");
            exit(errno);
        }
    }
    if (noent_print)
        fprintf(stderr, "\n");
    if ((s = (SimStats *)mmap(NULL, NUM_MAX_PRV_LEVELS * sizeof(SimStats), PROT_READ | PROT_WRITE,
                              MAP_SHARED, stats_shm_fd, 0))
        == MAP_FAILED)
    {
        fprintf(stderr, "cannot mmap shm %s:", MARSS_STATS_SHM_NAME);
    }
    fprintf(stderr, "memory attached at %p\n", s);
}

static void
print_header()
{
    printf("%s\n",
           "MARSS-RISCV : Micro-Architectural System Simulator for RISC-V");
    printf("%s\n\n", "Terminal based Simulation Statistics Viewer");
}


static void
print_ins_stats()
{
    uint64_t cycles = GET_TOTAL_STAT(cycles);
    uint64_t commits = GET_TOTAL_STAT(ins_simulated);
    uint64_t fetches = GET_TOTAL_STAT(ins_fetch);
    uint64_t flushed = fetches - commits;

    printf("%-22s : %0.2lf\n", "ipc", ((double)commits / (double)cycles));
    printf("%-22s : %-22" PRIu64 "\n", "cycles", cycles);
    printf("%-22s : %-22" PRIu64 "\n", "total-commits", commits);
    printf("%-22s : %-22" PRIu64 "\n", "total-fetches", fetches);
    printf("%-22s : %-22" PRIu64 " (%0.2lf %%)\n", "insn-flushed", flushed,
           (((double)flushed / (double)fetches)) * 100);
    printf("\n");
}

static void
print_bpu_stats()
{
    uint64_t btb_probes = GET_TOTAL_STAT(btb_probes);
    uint64_t btb_hits = GET_TOTAL_STAT(btb_hits);
    uint64_t correct_pred = GET_TOTAL_STAT(bpu_cond_correct) + GET_TOTAL_STAT(bpu_uncond_correct);
    uint64_t incorrect_pred = GET_TOTAL_STAT(bpu_cond_incorrect) + GET_TOTAL_STAT(bpu_uncond_incorrect);
    uint64_t total_branches = GET_TOTAL_STAT(ins_type[INS_TYPE_COND_BRANCH]) + GET_TOTAL_STAT(ins_type[INS_TYPE_JAL]) + GET_TOTAL_STAT(ins_type[INS_TYPE_JALR]);

    printf("%-22s : %-22" PRIu64 "\n", "btb-probes", btb_probes);
    printf("%-22s : %-22" PRIu64 " (%0.2lf %%)\n", "btb-hits", btb_hits,
           ((double)btb_hits / (double)btb_probes) * 100);
    printf("%-22s : %-22" PRIu64 " (%0.2lf %%)\n", "correct-predictions",
           correct_pred, ((double)correct_pred / (double)total_branches) * 100);
    printf("%-22s : %-22" PRIu64 " (%0.2lf %%)\n", "miss-predictions",
           incorrect_pred,
           ((double)incorrect_pred / (double)total_branches) * 100);
    printf("\n");
}

static void
print_tlb_stats()
{
    uint64_t code_tlb_hits = GET_TOTAL_STAT(code_tlb_hits);
    uint64_t load_tlb_hits = GET_TOTAL_STAT(load_tlb_hits);
    uint64_t store_tlb_hits = GET_TOTAL_STAT(store_tlb_hits);

    printf("%-22s : %-22" PRIu64 " (%0.2lf %%)\n", "itlb-hits", code_tlb_hits,
           ((double)code_tlb_hits / (double)GET_TOTAL_STAT(code_tlb_lookups))
               * 100);

    printf("%-22s : %-22" PRIu64 " (%0.2lf %%)\n", "load-tlb-hits",
           load_tlb_hits,
           ((double)load_tlb_hits / (double)GET_TOTAL_STAT(load_tlb_lookups))
               * 100);

    printf("%-22s : %-22" PRIu64 " (%0.2lf %%)\n", "store-tlb-hits",
           store_tlb_hits,
           ((double)store_tlb_hits / (double)GET_TOTAL_STAT(store_tlb_lookups))
               * 100);

    printf("\n");
}

static void
print_caches_stats()
{
    uint64_t icache_hit = GET_TOTAL_STAT(icache_read) - GET_TOTAL_STAT(icache_read_miss);
    uint64_t dcache_read_hit = GET_TOTAL_STAT(dcache_read) - GET_TOTAL_STAT(dcache_read_miss);
    uint64_t dcache_write_hit = GET_TOTAL_STAT(dcache_write) - GET_TOTAL_STAT(dcache_write_miss);
    uint64_t l2_cache_read_hit = GET_TOTAL_STAT(l2_cache_read) - GET_TOTAL_STAT(l2_cache_read_miss);
    uint64_t l2_cache_write_hit = GET_TOTAL_STAT(l2_cache_write) - GET_TOTAL_STAT(l2_cache_write_miss);

    printf("%-22s : %-22" PRIu64 " (%0.2lf %%)\n", "icache-hits", icache_hit,
           ((double)icache_hit / (double)GET_TOTAL_STAT(icache_read)) * 100);

    printf("%-22s : %-22" PRIu64 " (%0.2lf %%)\n", "dcache-read-hits",
           dcache_read_hit,
           ((double)dcache_read_hit / (double)GET_TOTAL_STAT(dcache_read))
               * 100);

    printf("%-22s : %-22" PRIu64 " (%0.2lf %%)\n", "dcache-write-hits",
           dcache_write_hit,
           ((double)dcache_write_hit / (double)GET_TOTAL_STAT(dcache_write))
               * 100);

    printf("%-22s : %-22" PRIu64 " (%0.2lf %%)\n", "l2-shared-read-hits",
           l2_cache_read_hit,
           ((double)l2_cache_read_hit / (double)GET_TOTAL_STAT(l2_cache_read)) * 100);

    printf("%-22s : %-22" PRIu64 " (%0.2lf %%)\n", "l2-shared-write-hits",
           l2_cache_write_hit,
           ((double)l2_cache_write_hit / (double)GET_TOTAL_STAT(l2_cache_write)) * 100);

    printf("\n");
}

static void
print_exception_stats()
{
    printf("%-22s : %-22" PRIu64 "\n", "ins-page-faults",
           GET_TOTAL_STAT(ins_page_faults));
    printf("%-22s : %-22" PRIu64 "\n", "load-page-faults",
           GET_TOTAL_STAT(load_page_faults));
    printf("%-22s : %-22" PRIu64 "\n", "store-page-faults",
           GET_TOTAL_STAT(store_page_faults));
}

int
main(int argc, char const *argv[])
{
    setup_connection();
    while (1)
    {
        /* Clear to the end of screen */
        printf("\033[H"
               "\033[J");

        print_header();
        print_ins_stats();
        print_bpu_stats();
        print_caches_stats();
        print_exception_stats();
        usleep(100000);
    }
    return 0;
}
