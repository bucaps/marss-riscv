/*
 * Decode Logic for Floating Point Arithmetics
 *
 * Copyright (c) 2016 Fabrice Bellard
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
#if F_SIZE == 32
#define OPID 0
#elif F_SIZE == 64
#define OPID 1
#elif F_SIZE == 128
#define OPID 3
#else
#error unsupported F_SIZE
#endif

case (0x00 << 2) | OPID: /* fadd */
case (0x01 << 2) | OPID: /* fsub */
case (0x02 << 2) | OPID: /* fmul */
case (0x03 << 2) | OPID: /* fdiv */
{
    if (ins->rm < 0)
        goto exception;
    ins->set_fs = TRUE;
    ins->has_fp_src1 = TRUE;
    ins->has_fp_src2 = TRUE;
    ins->has_fp_dest = TRUE;
    break;
}

case (0x0b << 2) | OPID: /* fsqrt */
{
    if (ins->rm < 0)
        goto exception;
    ins->set_fs = TRUE;
    ins->has_fp_src1 = TRUE;
    ins->has_fp_dest = TRUE;
    break;
}

case (0x04 << 2) | OPID: /* sign inject */
{
    ins->set_fs = TRUE;
    ins->has_fp_src1 = TRUE;
    ins->has_fp_src2 = TRUE;
    ins->has_fp_dest = TRUE;
    break;
}

case (0x05 << 2) | OPID: /* fmin-fmax */
{
    ins->set_fs = TRUE;
    ins->has_fp_src1 = TRUE;
    ins->has_fp_src2 = TRUE;
    ins->has_fp_dest = TRUE;
    break;
}

case (0x18 << 2) | OPID: /* f-convert */
{
    if (ins->rm < 0)
        goto exception;
    ins->has_fp_src1 = TRUE;
    ins->has_dest = TRUE;
    break;
}

case (0x14 << 2) | OPID: /* floating point comparisons */
{
    ins->has_dest = TRUE;
    ins->has_fp_src1 = TRUE;
    ins->has_fp_src2 = TRUE;
    break;
}

case (0x1a << 2) | OPID: /* floating point convert int-fp */
{
    if (ins->rm < 0)
        goto exception;
    ins->set_fs = TRUE;
    ins->has_fp_dest = TRUE;
    ins->has_src1 = TRUE;
    break;
}

case (0x08 << 2) | OPID: /* floating point convert */
{
    if (ins->rm < 0)
        goto exception;
    ins->set_fs = TRUE;
    ins->has_fp_dest = TRUE;
    ins->has_fp_src1 = TRUE;
    break;
}

case (0x1c << 2) | OPID: /* floating point move fp-int*/
{
    if (ins->rs2 != 0)
        goto exception;
    ins->has_dest = TRUE;
    ins->has_fp_src1 = TRUE;
    break;
}

#if F_SIZE <= BIT_SIZE
case (0x1e << 2) | OPID: /* fmv.s.x */
{
    if ((ins->rs2 != 0) || (ins->rm != 0))
        goto exception;
    ins->has_fp_dest = TRUE;
    ins->has_src1 = TRUE;
    break;
}
#endif /* F_SIZE <= BIT_SIZE */

#undef F_SIZE
#undef OPID
