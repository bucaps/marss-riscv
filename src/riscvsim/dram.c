/**
 * Basic Dram Model
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
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "dram.h"

/**
 * Physical address to DRAM mapping: MSB-LSB
 * | row | rank | col_high | bank | col_low | dimm | bus_offset |
 */

#define PADDR_GET(paddr, left_shift_by, mask)                                  \
    (((paddr) >> (left_shift_by)) & (mask))

#define MASK(bits) ((1 << (bits)) - 1)

static int row_buffer_hit_latency[2];
static int row_buffer_miss_latency[2];

Dram *
dram_init(const SimParams *p, uint64_t size, int num_dimms, int num_banks,
          int mem_bus_width, int col_size)
{
    Dram *d;
    int i, j, k, remaining_bits;

    d = calloc(1, sizeof(Dram));
    assert(d);
    d->dram_size = size;
    d->num_dimms = num_dimms;
    d->num_dimm_bits = GET_NUM_BITS(d->num_dimms);
    d->num_banks = num_banks;
    d->num_bank_bits = GET_NUM_BITS(d->num_banks);
    d->mem_bus_width = mem_bus_width;
    d->mem_bus_width_bytes = d->mem_bus_width / SIZE_OF_BYTE;
    d->bus_offset_bits = GET_NUM_BITS(d->mem_bus_width / SIZE_OF_BYTE);
    d->col_size = col_size;
    d->num_paddr_bits = GET_NUM_BITS(d->dram_size);
    remaining_bits
        = d->num_paddr_bits - (d->bus_offset_bits + d->num_dimm_bits
                               + d->num_bank_bits + DRAM_NUM_RANK_BITS);
    d->num_row_bits = 0.5 * remaining_bits;
    d->num_col_high_bits = 0.7 * (remaining_bits - d->num_row_bits);
    d->num_col_low_bits
        = remaining_bits - (d->num_row_bits + d->num_col_high_bits);

    d->row_buffer_read_hit_latency = p->mem_bus_access_rtt_latency + p->tCL;

    d->row_buffer_write_hit_latency
        = p->mem_bus_access_rtt_latency + p->row_buffer_write_latency;

    d->row_buffer_read_miss_latency
        = p->mem_bus_access_rtt_latency + p->tRP + p->tRCD + p->tCL;

    d->row_buffer_write_miss_latency = p->mem_bus_access_rtt_latency + p->tRP
                                       + p->tRCD + p->row_buffer_write_latency;

    row_buffer_hit_latency[0] = d->row_buffer_read_hit_latency;
    row_buffer_hit_latency[1] = d->row_buffer_write_hit_latency;
    row_buffer_miss_latency[0] = d->row_buffer_read_miss_latency;
    row_buffer_miss_latency[1] = d->row_buffer_write_miss_latency;

    d->dimm = calloc(d->num_dimms, sizeof(DIMM));
    assert(d->dimm);

    for (i = 0; i < d->num_dimms; ++i)
    {
        for (j = 0; j < DRAM_NUM_RANKS; ++j)
        {
            d->dimm[i].rank[j].chip.bank
                = calloc(d->num_banks, sizeof(DramBank));
            assert(d->dimm[i].rank[j].chip.bank);
            for (k = 0; k < d->num_banks; ++k)
            {
                d->dimm[i].rank[j].chip.bank[k].last_accessed_row_id = -1;
            }
        }
    }

    return d;
}

/**
 * Maps the given paddr to DRAM row, checks for row-buffer hit or miss
 * and returns latency in CPU cycles accordingly.
 */
int
dram_get_latency(Dram *d, target_ulong paddr, MemAccessType type)
{
    int dimm_idx, bank_idx, rank_idx, row_idx;
    int latency = 0;

    /* Map the physical address into dimm, bank, rank and row */
    dimm_idx = PADDR_GET(paddr, d->bus_offset_bits, MASK(d->num_dimm_bits));
    bank_idx = PADDR_GET(paddr, d->num_col_low_bits + d->num_dimm_bits
                                    + d->bus_offset_bits,
                         MASK(d->num_bank_bits));
    rank_idx = PADDR_GET(paddr, d->num_col_high_bits + d->num_bank_bits
                                    + d->num_col_low_bits + d->num_dimm_bits
                                    + d->bus_offset_bits,
                         MASK(DRAM_NUM_RANK_BITS));
    row_idx = PADDR_GET(paddr, DRAM_NUM_RANK_BITS + d->num_col_high_bits
                                   + d->num_bank_bits + d->num_col_low_bits
                                   + d->num_dimm_bits + d->bus_offset_bits,
                        MASK(d->num_row_bits));

    /* Get the bank in which the given paddr is mapped */
    DramBank *bank = &d->dimm[dimm_idx].rank[rank_idx].chip.bank[bank_idx];

    /* Check if DRAM page is already open in the row-buffer */
    if (bank->last_accessed_row_id == row_idx)
    {
        /* Row buffer hit (open-page) */
        latency = row_buffer_hit_latency[type];
    }
    else
    {
        /* Row buffer miss */
        latency = row_buffer_miss_latency[type];
        bank->last_accessed_row_id = row_idx;
    }

    return latency;
}

void
dram_print_config(Dram *d)
{
    printf("%-15s = %lu\n", "dram_size", d->dram_size);
    printf("%-15s = %d\n", "num_paddr_bits", d->num_paddr_bits);
    printf("%-15s = %d\n", "num_dimms", d->num_dimms);
    printf("%-15s = %d\n", "num_dimm_bits", d->num_dimm_bits);
    printf("%-15s = %d\n", "num_banks", d->num_banks);
    printf("%-15s = %d\n", "num_bank_bits", d->num_bank_bits);
    printf("%-15s = %d\n", "num_row_bits", d->num_row_bits);
    printf("%-15s = %d\n", "num_col_high_bits", d->num_col_high_bits);
    printf("%-15s = %d\n", "num_col_low_bits", d->num_col_low_bits);
    printf("%-15s = %d bits\n", "mem_bus_width", d->mem_bus_width);
    printf("%-15s = %d\n", "bus_offset_bits", d->bus_offset_bits);
}

void
dram_free(Dram **d)
{
    int i, j;

    for (i = 0; i < (*d)->num_dimms; ++i)
    {
        for (j = 0; j < DRAM_NUM_RANKS; ++j)
        {
            free((*d)->dimm[i].rank[j].chip.bank);
            (*d)->dimm[i].rank[j].chip.bank = NULL;
        }
    }

    free((*d)->dimm);
    (*d)->dimm = NULL;
    free(*d);
    *d = NULL;
}

void
dram_flush(Dram *d)
{
    int i, j, k;

    for (i = 0; i < d->num_dimms; ++i)
    {
        for (j = 0; j < DRAM_NUM_RANKS; ++j)
        {
            for (k = 0; k < d->num_banks; ++k)
            {
                d->dimm[i].rank[j].chip.bank[k].last_accessed_row_id = -1;
            }
        }
    }
}
