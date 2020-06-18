/*
 * Instruction String Generation Logic for Floating Point Arithmetics
 *
 * Copyright (c) 2016-2017 Fabrice Bellard
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
#define F_HIGH F32_HIGH
#elif F_SIZE == 64
#define OPID 1
#define F_HIGH F64_HIGH
#elif F_SIZE == 128
#define OPID 3
#define F_HIGH 0
#else
#error unsupported F_SIZE
#endif

case (0x00 << 2) | OPID: /* fadd */
{
    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fadd%s %s,%s,%s",
             get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd], fp_reg[i->rs1],
             fp_reg[i->rs2]);
    break;
}

case (0x01 << 2) | OPID: /* fsub */
{
    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fsub%s %s,%s,%s",
             get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd], fp_reg[i->rs1],
             fp_reg[i->rs2]);
    break;
}

case (0x02 << 2) | OPID: /* fmul */
{
    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fmul%s %s,%s,%s",
             get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd], fp_reg[i->rs1],
             fp_reg[i->rs2]);
    break;
}

case (0x03 << 2) | OPID: /* fdiv */
{
    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fdiv%s %s,%s,%s",
             get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd], fp_reg[i->rs1],
             fp_reg[i->rs2]);
    break;
}

case (0x0b << 2) | OPID: /* fsqrt */
{
    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fsqrt%s %s,%s",
             get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd], fp_reg[i->rs1]);
    break;
}

case (0x04 << 2) | OPID: /* sign inject */
{
    switch (rm)
    {
        case 0: /* fsgnj */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fsgnj%s %s,%s,%s",
                     get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd],
                     fp_reg[i->rs1], fp_reg[i->rs2]);
            break;
        }
        case 1: /* fsgnjn */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fsgnjn%s %s,%s,%s",
                     get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd],
                     fp_reg[i->rs1], fp_reg[i->rs2]);
            break;
        }
        case 2: /* fsgnjx */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fsgnjx%s %s,%s,%s",
                     get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd],
                     fp_reg[i->rs1], fp_reg[i->rs2]);
            break;
        }
    }
    break;
}

case (0x05 << 2) | OPID:
{
    switch (rm)
    {
        case 0: /* fmin */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fmin%s %s,%s,%s",
                     get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd],
                     fp_reg[i->rs1], fp_reg[i->rs2]);
            break;
        }
        case 1: /* fmax */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fmax%s %s,%s,%s",
                     get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd],
                     fp_reg[i->rs1], fp_reg[i->rs2]);
            break;
        }
    }
    break;
}

case (0x18 << 2) | OPID: /* f-convert */
{
    switch (i->rs2)
    {
        case 0: /* fcvt.w.[sdq] */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fcvt.w%s %s,%s",
                     get_flen_suffix[(F_SIZE / 32)], reg[i->rd],
                     fp_reg[i->rs1]);
            break;
        }
        case 1: /* fcvt.wu.[sdq] */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fcvt.wu%s %s,%s",
                     get_flen_suffix[(F_SIZE / 32)], reg[i->rd],
                     fp_reg[i->rs1]);
            break;
        }
#if BIT_SIZE >= 64
        case 2: /* fcvt.l.[sdq] */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fcvt.l%s %s,%s",
                     get_flen_suffix[(F_SIZE / 32)], reg[i->rd],
                     fp_reg[i->rs1]);
            break;
        }
        case 3: /* fcvt.lu.[sdq] */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fcvt.lu%s %s,%s",
                     get_flen_suffix[(F_SIZE / 32)], reg[i->rd],
                     fp_reg[i->rs1]);
            break;
        }
#endif
    }
    break;
}

case (0x14 << 2) | OPID:
{
    switch (rm)
    {
        case 0: /* fle */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fle%s %s,%s,%s",
                     get_flen_suffix[(F_SIZE / 32)], reg[i->rd], fp_reg[i->rs1],
                     fp_reg[i->rs2]);
            break;
        }
        case 1: /* flt */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "flt%s %s,%s,%s",
                     get_flen_suffix[(F_SIZE / 32)], reg[i->rd], fp_reg[i->rs1],
                     fp_reg[i->rs2]);
            break;
        }
        case 2: /* feq */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "feq%s %s,%s,%s",
                     get_flen_suffix[(F_SIZE / 32)], reg[i->rd], fp_reg[i->rs1],
                     fp_reg[i->rs2]);
            break;
        }
    }
    break;
}

case (0x1a << 2) | OPID:
{
    switch (i->rs2)
    {
        case 0: /* fcvt.[sdq].w */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fcvt%s.w %s,%s",
                     get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd],
                     reg[i->rs1]);
            break;
        }
        case 1: /* fcvt.[sdq].wu */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fcvt%s.wu %s,%s",
                     get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd],
                     reg[i->rs1]);
            break;
        }
#if BIT_SIZE >= 64
        case 2: /* fcvt.[sdq].l */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fcvt%s.l %s,%s",
                     get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd],
                     reg[i->rs1]);
            break;
        }
        case 3: /* fcvt.[sdq].lu */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fcvt%s.lu %s,%s",
                     get_flen_suffix[(F_SIZE / 32)], fp_reg[i->rd],
                     reg[i->rs1]);
            break;
        }
#endif
    }
    break;
}

case (0x08 << 2) | OPID:
{
    switch (i->rs2)
    {
#if F_SIZE == 32 && FLEN >= 64
        case 1: /* cvt.s.d */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "cvt.s.d %s,%s",
                     fp_reg[i->rd], fp_reg[i->rs1]);
            break;
        }
#endif /* F_SIZE == 32 */
#if F_SIZE == 64
        case 0: /* cvt.d.s */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "cvt.d.s %s,%s",
                     fp_reg[i->rd], fp_reg[i->rs1]);
            break;
        }
#endif /* F_SIZE == 64 */
    }
    break;
}

case (0x1c << 2) | OPID:
{
    switch (rm)
    {
#if F_SIZE <= BIT_SIZE
        case 0: /* fmv.x.s */
        {
#if F_SIZE == 32
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fmv.x.s %s,%s",
                     reg[i->rd], fp_reg[i->rs1]);
#elif F_SIZE == 64
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fmv.x.d %s,%s",
                     reg[i->rd], fp_reg[i->rs1]);
#else
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fmv.x.q %s,%s",
                     reg[i->rd], fp_reg[i->rs1]);
#endif
            break;
        }
#endif          /* F_SIZE <= BIT_SIZE */
        case 1: /* fclass */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fclass%s %s,%s",
                     get_flen_suffix[(F_SIZE) / 32], reg[i->rd],
                     fp_reg[i->rs1]);
            break;
        }
    }
    break;
}

#if F_SIZE <= BIT_SIZE
case (0x1e << 2) | OPID: /* fmv.s.x */
{
#if F_SIZE == 32
    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fmv.s.x %s,%s", fp_reg[i->rd],
             reg[i->rs1]);
#elif F_SIZE == 64
    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fmv.s.x %s,%s", fp_reg[i->rd],
             reg[i->rs1]);
#else
    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fmv.s.x %s,%s", fp_reg[i->rd],
             reg[i->rs1]);
#endif
    break;
}
#endif /* F_SIZE <= BIT_SIZE */

#undef F_SIZE
#undef F_HIGH
#undef OPID
#undef FSIGN_MASK
