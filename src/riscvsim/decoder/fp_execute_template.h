/*
 * Execute Logic for Floating Point Arithmetics
 *
 * Copyright (c) 2016 Fabrice Bellard
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

#define FSIGN_MASK simglue(FSIGN_MASK, F_SIZE)

case (0x00 << 2) | OPID: /* fadd */
    i->fpu_alu_type = FU_FPU_ALU_FADD;
    i->buffer = simglue(add_sf, F_SIZE)(i->rs1_val, i->rs2_val,
                                        (RoundingModeEnum)rm, fflags)
                | F_HIGH;
    i->type = INS_TYPE_FP_ADD;
    break;

case (0x01 << 2) | OPID: /* fsub */
    i->fpu_alu_type = FU_FPU_ALU_FSUB;
    i->buffer = simglue(sub_sf, F_SIZE)(i->rs1_val, i->rs2_val,
                                        (RoundingModeEnum)rm, fflags)
                | F_HIGH;
    break;

case (0x02 << 2) | OPID: /* fmul */
    i->fpu_alu_type = FU_FPU_ALU_FMUL;
    i->buffer = simglue(mul_sf, F_SIZE)(i->rs1_val, i->rs2_val,
                                        (RoundingModeEnum)rm, fflags)
                | F_HIGH;
    i->type = INS_TYPE_FP_MUL;
    break;

case (0x03 << 2) | OPID: /* fdiv */
    i->fpu_alu_type = FU_FPU_ALU_FDIV;
    i->buffer = simglue(div_sf, F_SIZE)(i->rs1_val, i->rs2_val,
                                        (RoundingModeEnum)rm, fflags)
                | F_HIGH;
    i->type = INS_TYPE_FP_DIV_SQRT;
    break;

case (0x0b << 2) | OPID: /* fsqrt */
    i->fpu_alu_type = FU_FPU_ALU_FSQRT;
    i->buffer
        = simglue(sqrt_sf, F_SIZE)(i->rs1_val, (RoundingModeEnum)rm, fflags)
          | F_HIGH;
    i->type = INS_TYPE_FP_DIV_SQRT;
    break;

case (0x04 << 2) | OPID: /* sign inject */
    i->fpu_alu_type = FU_FPU_ALU_FSGNJ;
    switch (rm)
    {
        case 0: /* fsgnj */
            i->buffer = (i->rs1_val & ~FSIGN_MASK) | (i->rs2_val & FSIGN_MASK);
            break;
        case 1: /* fsgnjn */
            i->buffer = (i->rs1_val & ~FSIGN_MASK)
                        | ((i->rs2_val & FSIGN_MASK) ^ FSIGN_MASK);
            break;
        case 2: /* fsgnjx */
            i->buffer = i->rs1_val ^ (i->rs2_val & FSIGN_MASK);
            break;
    }
    break;

case (0x05 << 2) | OPID:
    switch (rm)
    {
        case 0: /* fmin */
            i->fpu_alu_type = FU_FPU_ALU_FMIN;
            i->buffer = simglue(min_sf, F_SIZE)(i->rs1_val, i->rs2_val, fflags,
                                                FMINMAX_IEEE754_201X)
                        | F_HIGH;
            break;
        case 1: /* fmax */
            i->fpu_alu_type = FU_FPU_ALU_FMAX;
            i->buffer = simglue(max_sf, F_SIZE)(i->rs1_val, i->rs2_val, fflags,
                                                FMINMAX_IEEE754_201X)
                        | F_HIGH;
            break;
    }
    break;

case (0x18 << 2) | OPID: /* f-convert */
    i->fpu_alu_type = FU_FPU_ALU_FCVT;
    switch (i->rs2)
    {
        case 0: /* fcvt.w.[sdq] */
            i->buffer = (int32_t)simglue(simglue(cvt_sf, F_SIZE), _i32)(
                i->rs1_val, (RoundingModeEnum)rm, fflags);
            break;
        case 1: /* fcvt.wu.[sdq] */
            i->buffer = (int32_t)simglue(simglue(cvt_sf, F_SIZE), _u32)(
                i->rs1_val, (RoundingModeEnum)rm, fflags);
            break;
#if BIT_SIZE >= 64
        case 2: /* fcvt.l.[sdq] */
            i->buffer = (int64_t)simglue(simglue(cvt_sf, F_SIZE), _i64)(
                i->rs1_val, (RoundingModeEnum)rm, fflags);
            break;
        case 3: /* fcvt.lu.[sdq] */
            i->buffer = (int64_t)simglue(simglue(cvt_sf, F_SIZE), _u64)(
                i->rs1_val, (RoundingModeEnum)rm, fflags);
            break;
#endif
    }
    break;

case (0x14 << 2) | OPID:
    switch (rm)
    {
        case 0: /* fle */
            i->fpu_alu_type = FU_FPU_ALU_FLE;
            i->buffer = simglue(le_sf, F_SIZE)(i->rs1_val, i->rs2_val, fflags);
            break;
        case 1: /* flt */
            i->fpu_alu_type = FU_FPU_ALU_FLT;
            i->buffer = simglue(lt_sf, F_SIZE)(i->rs1_val, i->rs2_val, fflags);
            break;
        case 2: /* feq */
            i->fpu_alu_type = FU_FPU_ALU_FEQ;
            i->buffer
                = simglue(eq_quiet_sf, F_SIZE)(i->rs1_val, i->rs2_val, fflags);
            break;
    }
    break;

case (0x1a << 2) | OPID:
    i->fpu_alu_type = FU_FPU_ALU_FCVT;
    switch (i->rs2)
    {
        case 0: /* fcvt.[sdq].w */
            i->buffer = simglue(cvt_i32_sf, F_SIZE)(
                            i->rs1_val, (RoundingModeEnum)rm, fflags)
                        | F_HIGH;
            break;
        case 1: /* fcvt.[sdq].wu */
            i->buffer = simglue(cvt_u32_sf, F_SIZE)(
                            i->rs1_val, (RoundingModeEnum)rm, fflags)
                        | F_HIGH;
            break;
#if BIT_SIZE >= 64
        case 2: /* fcvt.[sdq].l */
            i->buffer = simglue(cvt_i64_sf, F_SIZE)(
                            i->rs1_val, (RoundingModeEnum)rm, fflags)
                        | F_HIGH;
            break;
        case 3: /* fcvt.[sdq].lu */
            i->buffer = simglue(cvt_u64_sf, F_SIZE)(
                            i->rs1_val, (RoundingModeEnum)rm, fflags)
                        | F_HIGH;
            break;
#endif
    }
    break;

case (0x08 << 2) | OPID:
    i->fpu_alu_type = FU_FPU_ALU_CVT;
    switch (i->rs2)
    {
#if F_SIZE == 32 && FLEN >= 64
        case 1: /* cvt.s.d */
            i->buffer = cvt_sf64_sf32(i->rs1_val, (RoundingModeEnum)rm, fflags)
                        | F32_HIGH;
            break;
#endif /* F_SIZE == 32 */
#if F_SIZE == 64
        case 0: /* cvt.d.s */
            i->buffer = cvt_sf32_sf64(i->rs1_val, fflags) | F64_HIGH;
            break;
#endif /* F_SIZE == 64 */
    }
    break;

case (0x1c << 2) | OPID:
    i->fpu_alu_type = FU_FPU_ALU_FMV;
    switch (rm)
    {
#if F_SIZE <= BIT_SIZE
        case 0: /* fmv.x.s */
#if F_SIZE == 32
            i->buffer = (int32_t)i->rs1_val;
#elif F_SIZE == 64
            i->buffer = (int64_t)i->rs1_val;
#else
            i->buffer = (int128_t)i->rs1_val;
#endif
            break;
#endif          /* F_SIZE <= BIT_SIZE */
        case 1: /* fclass */
            i->fpu_alu_type = FU_FPU_ALU_FCLASS;
            i->buffer = simglue(fclass_sf, F_SIZE)(i->rs1_val);
            break;
    }
    break;

#if F_SIZE <= BIT_SIZE
case (0x1e << 2) | OPID: /* fmv.s.x */
    i->fpu_alu_type = FU_FPU_ALU_FMV;
#if F_SIZE == 32
    i->buffer = (int32_t)i->rs1_val;
#elif F_SIZE == 64
    i->buffer = (int64_t)i->rs1_val;
#else
    i->buffer = (int128_t)i->rs1_val;
#endif
    break;
#endif /* F_SIZE <= BIT_SIZE */

#undef F_SIZE
#undef F_HIGH
#undef OPID
#undef FSIGN_MASK
