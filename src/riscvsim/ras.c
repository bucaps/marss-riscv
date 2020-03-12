/**
 * Return address stack
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2020 Gaurav Kothari {gkothar1@binghamton.edu}
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
#include <stdlib.h>

#include "ras.h"

Ras *
ras_init(const SimParams *p)
{
    Ras *r;

    r = calloc(1, sizeof(Ras));
    assert(r);

    r->entry = calloc(p->ras_size, sizeof(target_ulong));
    assert(r->entry);

    r->max_size = p->ras_size;
    ras_flush(r);
    return r;
}

void
ras_free(Ras **ras)
{
	free((*ras)->entry);
	(*ras)->entry = NULL;

	free(*ras);
	*ras = NULL;
}

void
ras_flush(Ras *ras)
{
    ras->cur_size = 0;
    ras->sptop = ras->max_size - 1;
    ras->spfill = 0;
    ras->empty_reg = 0;
}

int
ras_empty(Ras *ras)
{
    return (!ras->cur_size);
}

void
ras_push(Ras *ras, target_ulong pc)
{
    ras->entry[ras->spfill] = pc;
    ras->sptop = ras->spfill;
    ras->spfill++;
    ras->cur_size++;

    /* Overflow */
    if (ras->spfill >= ras->max_size)
    {
        ras->spfill = 0;
    }
}

target_ulong
ras_pop(Ras *ras)
{
    target_ulong ret_addr;

    if (ras_empty(ras))
    {
        /* For empty RAS, return the last popped address in RAS empty register*/
        return ras->empty_reg;
    }

    ret_addr = ras->entry[ras->sptop];
    ras->spfill = ras->sptop;
    ras->sptop--;
    ras->cur_size--;

    /* Underflow */
    if (ras->sptop < 0)
    {
        ras->sptop = ras->max_size - 1;
    }

    /* save pop address into empty register */
    ras->empty_reg = ret_addr;
    return ret_addr;
}