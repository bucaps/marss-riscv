/**
 * RISCV Instruction Execution Logic
 *
 * Copyright (c) 2016-2017 Fabrice Bellard
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../riscv_cpu_priv.h"
#include "../../softfp.h"
#include "../riscv_sim_typedefs.h"
#include "riscv_instruction.h"

#define xsimglue(x, y) x##y
#define simglue(x, y) xsimglue(x, y)

static inline target_long simglue(div, BIT_SIZE)(target_long a, target_long b)
{
    if (b == 0)
    {
        return -1;
    }
    else if (a == ((target_long)1 << (BIT_SIZE - 1)) && b == -1)
    {
        return a;
    }
    else
    {
        return a / b;
    }
}

static inline target_ulong simglue(divu, BIT_SIZE)(target_ulong a,
                                                   target_ulong b)
{
    if (b == 0)
    {
        return -1;
    }
    else
    {
        return a / b;
    }
}

static inline target_long simglue(rem, BIT_SIZE)(target_long a, target_long b)
{
    if (b == 0)
    {
        return a;
    }
    else if (a == ((target_long)1 << (BIT_SIZE - 1)) && b == -1)
    {
        return 0;
    }
    else
    {
        return a % b;
    }
}

static inline target_ulong simglue(remu, BIT_SIZE)(target_ulong a,
                                                   target_ulong b)
{
    if (b == 0)
    {
        return a;
    }
    else
    {
        return a % b;
    }
}

#if BIT_SIZE == 32

static inline uint32_t
mulh32(int32_t a, int32_t b)
{
    return ((int64_t)a * (int64_t)b) >> 32;
}

static inline uint32_t
mulhsu32(int32_t a, uint32_t b)
{
    return ((int64_t)a * (int64_t)b) >> 32;
}

static inline uint32_t
mulhu32(uint32_t a, uint32_t b)
{
    return ((int64_t)a * (int64_t)b) >> 32;
}

#elif BIT_SIZE == 64 && defined(HAVE_INT128)

static inline uint64_t
mulh64(int64_t a, int64_t b)
{
    return ((int128_t)a * (int128_t)b) >> 64;
}

static inline uint64_t
mulhsu64(int64_t a, uint64_t b)
{
    return ((int128_t)a * (int128_t)b) >> 64;
}

static inline uint64_t
mulhu64(uint64_t a, uint64_t b)
{
    return ((int128_t)a * (int128_t)b) >> 64;
}

#else

#if BIT_SIZE == 64
#define UHALF uint32_t
#define UHALF_LEN 32
#elif BIT_SIZE == 128
#define UHALF uint64_t
#define UHALF_LEN 64
#else
#error unsupported BIT_SIZE
#endif

static target_ulong simglue(mulhu, BIT_SIZE)(target_ulong a, target_ulong b)
{
    UHALF a0, a1, b0, b1, r2, r3;
    target_ulong r00, r01, r10, r11, c;
    a0 = a;
    a1 = a >> UHALF_LEN;
    b0 = b;
    b1 = b >> UHALF_LEN;

    r00 = (target_ulong)a0 * (target_ulong)b0;
    r01 = (target_ulong)a0 * (target_ulong)b1;
    r10 = (target_ulong)a1 * (target_ulong)b0;
    r11 = (target_ulong)a1 * (target_ulong)b1;

    //    r0 = r00;
    c = (r00 >> UHALF_LEN) + (UHALF)r01 + (UHALF)r10;
    //    r1 = c;
    c = (c >> UHALF_LEN) + (r01 >> UHALF_LEN) + (r10 >> UHALF_LEN) + (UHALF)r11;
    r2 = c;
    r3 = (c >> UHALF_LEN) + (r11 >> UHALF_LEN);

    //    *plow = ((target_ulong)r1 << UHALF_LEN) | r0;
    return ((target_ulong)r3 << UHALF_LEN) | r2;
}

#undef UHALF

static inline target_ulong simglue(mulh, BIT_SIZE)(target_long a, target_long b)
{
    target_ulong r1;
    r1 = simglue(mulhu, BIT_SIZE)(a, b);
    if (a < 0)
        r1 -= a;
    if (b < 0)
        r1 -= b;
    return r1;
}

static inline target_ulong simglue(mulhsu, BIT_SIZE)(target_long a,
                                                     target_ulong b)
{
    target_ulong r1;
    r1 = simglue(mulhu, BIT_SIZE)(a, b);
    if (a < 0)
        r1 -= a;
    return r1;
}
#endif

static void
execute_op_imm(RVInstruction *i)
{
    uint32_t insn = i->binary;
    uint32_t funct3 = (insn >> 12) & 7;

    switch (funct3)
    {
        case 0: /* addi */
            i->buffer = (target_long)(i->rs1_val + i->imm);
            break;
        case 1: /* slli */
            i->buffer = (target_long)(i->rs1_val << (i->imm & (BIT_SIZE - 1)));
            break;
        case 2: /* slti */
            i->buffer = (target_long)i->rs1_val < (target_long)i->imm;
            break;
        case 3: /* sltiu */
            i->buffer = i->rs1_val < (target_ulong)i->imm;
            break;
        case 4: /* xori */
            i->buffer = i->rs1_val ^ i->imm;
            break;
        case 5: /* srli/srai */
            if (i->imm & 0x400)
                i->buffer
                    = (target_long)i->rs1_val >> (i->imm & (BIT_SIZE - 1));
            else
                i->buffer = (target_long)((target_ulong)i->rs1_val
                                          >> (i->imm & (BIT_SIZE - 1)));
            break;
        case 6: /* ori */
            i->buffer = i->rs1_val | i->imm;
            break;
        case 7: /* andi */
            i->buffer = i->rs1_val & i->imm;
            break;
    }
    return;
}

static void
execute_op_imm_32(RVInstruction *i)
{
    uint32_t funct3 = (i->binary >> 12) & 7;

    switch (funct3)
    {
        case 0: /* addiw */
            i->buffer = (int32_t)(i->rs1_val + i->imm);
            break;
        case 1: /* slliw */
            i->buffer = (int32_t)(i->rs1_val << (i->imm & 31));
            break;
        case 5: /* srliw/sraiw */
            if (i->imm & 0x400)
                i->buffer = (int32_t)i->rs1_val >> (i->imm & 31);
            else
                i->buffer = (int32_t)((uint32_t)i->rs1_val >> (i->imm & 31));
            break;
    }
    return;
}

static void
execute_op(RVInstruction *i)
{
    uint32_t funct3;
    uint32_t insn = i->binary;
    int32_t imm = insn >> 25;

    if (imm == 1)
    {
        funct3 = (insn >> 12) & 7;
        switch (funct3)
        {
            case 0: /* mul */
                i->buffer = (target_long)((target_long)i->rs1_val
                                          * (target_long)i->rs2_val);
                break;
            case 1: /* mulh */
                i->buffer = (target_long)simglue(mulh, BIT_SIZE)(i->rs1_val,
                                                                 i->rs2_val);
                break;
            case 2: /* mulhsu */
                i->buffer = (target_long)simglue(mulhsu, BIT_SIZE)(i->rs1_val,
                                                                   i->rs2_val);
                break;
            case 3: /* mulhu */
                i->buffer = (target_long)simglue(mulhu, BIT_SIZE)(i->rs1_val,
                                                                  i->rs2_val);
                break;
            case 4: /* div */
                i->buffer = simglue(div, BIT_SIZE)(i->rs1_val, i->rs2_val);
                break;
            case 5: /* divu */
                i->buffer = (target_long)simglue(divu, BIT_SIZE)(i->rs1_val,
                                                                 i->rs2_val);
                break;
            case 6: /* rem */
                i->buffer = simglue(rem, BIT_SIZE)(i->rs1_val, i->rs2_val);
                break;
            case 7: /* remu */
                i->buffer = (target_long)simglue(remu, BIT_SIZE)(i->rs1_val,
                                                                 i->rs2_val);
                break;
        }
    }
    else
    {
        funct3 = ((insn >> 12) & 7) | ((insn >> (30 - 3)) & (1 << 3));
        switch (funct3)
        {
            case 0: /* add */
                i->buffer = (target_long)(i->rs1_val + i->rs2_val);
                break;
            case 0 | 8: /* sub */
                i->buffer = (target_long)(i->rs1_val - i->rs2_val);
                break;
            case 1: /* sll */
                i->buffer = (target_long)(i->rs1_val
                                          << (i->rs2_val & (BIT_SIZE - 1)));
                break;
            case 2: /* slt */
                i->buffer = (target_long)i->rs1_val < (target_long)i->rs2_val;
                break;
            case 3: /* sltu */
                i->buffer = i->rs1_val < i->rs2_val;
                break;
            case 4: /* xor */
                i->buffer = i->rs1_val ^ i->rs2_val;
                break;
            case 5: /* srl */
                i->buffer = (target_long)((target_ulong)i->rs1_val
                                          >> (i->rs2_val & (BIT_SIZE - 1)));
                break;
            case 5 | 8: /* sra */
                i->buffer
                    = (target_long)i->rs1_val >> (i->rs2_val & (BIT_SIZE - 1));
                break;
            case 6: /* or */
                i->buffer = i->rs1_val | i->rs2_val;
                break;
            case 7: /* and */
                i->buffer = i->rs1_val & i->rs2_val;
                break;
        }
    }
    return;
}

static void
execute_op_32(RVInstruction *i)
{
    uint32_t insn = i->binary;
    int32_t imm = insn >> 25;
    uint32_t funct3;

    if (imm == 1)
    {
        funct3 = (insn >> 12) & 7;
        switch (funct3)
        {
            case 0: /* mulw */
                i->buffer
                    = (int32_t)((int32_t)i->rs1_val * (int32_t)i->rs2_val);
                break;
            case 4: /* divw */
                i->buffer = simglue(div, BIT_SIZE)(i->rs1_val, i->rs2_val);
                break;
            case 5: /* divuw */
                i->buffer
                    = (int32_t)simglue(divu, BIT_SIZE)(i->rs1_val, i->rs2_val);
                break;
            case 6: /* remw */
                i->buffer = simglue(rem, BIT_SIZE)(i->rs1_val, i->rs2_val);
                break;
            case 7: /* remuw */
                i->buffer
                    = (int32_t)simglue(remu, BIT_SIZE)(i->rs1_val, i->rs2_val);
                break;
        }
    }
    else
    {
        funct3 = ((insn >> 12) & 7) | ((insn >> (30 - 3)) & (1 << 3));
        switch (funct3)
        {
            case 0: /* addw */
                i->buffer = (int32_t)(i->rs1_val + i->rs2_val);
                break;
            case 0 | 8: /* subw */
                i->buffer = (int32_t)(i->rs1_val - i->rs2_val);
                break;
            case 1: /* sllw */
                i->buffer
                    = (int32_t)((uint32_t)i->rs1_val << (i->rs2_val & 31));
                break;
            case 5: /* srlw */
                i->buffer
                    = (int32_t)((uint32_t)i->rs1_val >> (i->rs2_val & 31));
                break;
            case 5 | 8: /* sraw */
                i->buffer = (int32_t)i->rs1_val >> (i->rs2_val & 31);
                break;
        }
    }
    return;
}

static void
execute_type_b(RVInstruction *i)
{
    uint32_t insn = i->binary;
    uint32_t funct3 = (insn >> 12) & 7;
    int32_t cond = -1;

    switch (funct3 >> 1)
    {
        case 0: /* beq/bne */
            cond = (i->rs1_val == i->rs2_val);
            break;
        case 2: /* blt/bge */
            cond = ((target_long)i->rs1_val < (target_long)i->rs2_val);
            break;
        case 3: /* bltu/bgeu */
            cond = (i->rs1_val < i->rs2_val);
            break;
    }
    cond ^= (funct3 & 1);
    i->cond = cond;
    i->target = i->pc + i->imm;
    return;
}

static void
execute_ext_c_q0(RVInstruction *i)
{
    switch (i->funct3)
    {
        case 0: /* c.addi4spn */
        {
            i->buffer = (target_long)(i->rs1_val + i->imm);
        }
        break;
#if SIM_FLEN >= 64
        case 1: /* c.fld */
        {
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
        }
        break;
#endif
        case 2: /* c.lw */
        {
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
        }
        break;
#if defined(RV64)
        case 3: /* c.ld */
        {
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
        }
        break;
#elif SIM_FLEN >= 32
        case 3: /* c.flw */
        {
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
        }
        break;
#endif
#if SIM_FLEN >= 64
        case 5: /* c.fsd */
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
            break;
#endif
        case 6: /* c.sw */
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
            break;
#if defined(RV64)
        case 7: /* c.sd */
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
            break;
#elif SIM_FLEN >= 32
        case 7: /* c.fsw */
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
            break;
#endif
    }
    return;
}

static void
execute_ext_c_q1(RVInstruction *i)
{
    switch (i->funct3)
    {
        case 0: /* c.addi/c.nop */
            i->buffer = (target_long)(i->rs1_val + i->imm);
            break;
#if defined(RV32)
        case 1: /* c.jal */
            i->buffer = (target_ulong)(i->pc + 2);
            i->target = (target_long)(i->pc + i->imm);
            break;
#elif defined(RV64)
        case 1: /* c.addiw */
            i->buffer = (int32_t)(i->rs1_val + i->imm);
            break;
#endif
        case 2: /* c.li */
            i->buffer = i->imm;
            break;
        case 3:
            if (i->rd == 2)
            {
                /* c.addi16sp */
                i->buffer = (target_long)(i->rs1_val + i->imm);
            }
            else if (i->rd != 0)
            {
                /* c.lui */
                i->buffer = i->imm;
            }
            break;
        case 4:
            switch (i->funct4)
            {
                case 0: /* c.srli */
                case 1: /* c.srai */
                    if (i->funct4 == 0)
                    {
                        i->buffer
                            = (target_long)((target_ulong)i->rs1_val >> i->imm);
                    }
                    else
                    {
                        i->buffer = (target_long)i->rs1_val >> i->imm;
                    }
                    break;
                case 2: /* c.andi */
                    i->buffer = (target_long)i->rs1_val & i->imm;
                    break;
                case 3:
                    switch (i->funct5)
                    {
                        case 0: /* c.sub */
                            i->buffer = (target_long)(i->rs1_val - i->rs2_val);
                            break;
                        case 1: /* c.xor */
                            i->buffer = (i->rs1_val ^ i->rs2_val);
                            break;
                        case 2: /* c.or */
                            i->buffer = (i->rs1_val | i->rs2_val);
                            break;
                        case 3: /* c.and */
                            i->buffer = (i->rs1_val & i->rs2_val);
                            break;
#if defined(RV64)
                        case 4: /* c.subw */
                            i->buffer = (int32_t)(i->rs1_val - i->rs2_val);
                            break;
                        case 5: /* c.addw */
                            i->buffer = (int32_t)(i->rs1_val + i->rs2_val);
                            break;
#endif
                    }
                    break;
            }
            break;
        case 5: /* c.j */
        {
            i->target = (target_ulong)(i->pc + i->imm);
        }
        break;
        case 6: /* c.beqz */
        {
            i->cond = (i->rs1_val == 0);
            i->target = (target_ulong)(i->pc + i->imm);
        }
        break;
        case 7: /* c.bnez */
        {
            i->cond = (i->rs1_val != 0);
            i->target = (target_ulong)(i->pc + i->imm);
        }
        break;
    }
    return;
}

static void
execute_ext_c_q2(RVInstruction *i)
{
    uint32_t insn = i->binary;

    switch (i->funct3)
    {
        case 0: /* c.slli */
            i->buffer = (target_long)(i->rs1_val << i->imm);
            break;
#if SIM_FLEN >= 64
        case 1: /* c.fldsp */
        {
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
        }
        break;
#endif
        case 2: /* c.lwsp */
        {
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
        }
        break;
#if defined(RV64)
        case 3: /* c.ldsp */
        {
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
        }
        break;
#elif SIM_FLEN >= 32
        case 3: /* c.flwsp */
        {
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
        }
        break;
#endif
        case 4:
            if (((insn >> 12) & 1) == 0)
            {
                if (i->rs2 == 0)
                {
                    /* c.jr */
                    i->target = (i->rs1_val & ~1);
                }
                else
                {
                    /* c.mv */
                    i->buffer = i->rs2_val;
                }
            }
            else
            {
                if (i->rs2 == 0)
                {
                    if (i->rd == 0)
                    {
                        /* c.ebreak */
                    }
                    else
                    {
                        /* c.jalr */
                        i->buffer = i->pc + 2;
                        i->target = i->rs1_val & ~1;
                    }
                }
                else
                {
                    /** c.add */
                    i->buffer = (target_long)(i->rs1_val + i->rs2_val);
                }
            }
            break;
#if SIM_FLEN >= 64
        case 5: /* c.fsdsp */
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
            break;
#endif
        case 6: /* c.swsp */
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
            break;
#if defined(RV64)
        case 7: /* c.sdsp */
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
            break;
#elif SIM_FLEN >= 32
        case 7: /* c.fswsp */
            i->mem_addr = (target_long)(i->rs1_val + i->imm);
            break;
#endif
    }
    return;
}

static void
execute_ext_c_path(RVInstruction *i)
{
    switch (i->quad)
    {
        case 0:
        {
            execute_ext_c_q0(i);
            break;
        }
        case 1:
        {
            execute_ext_c_q1(i);
            break;
        }
        case 2:
        {
            execute_ext_c_q2(i);
            break;
        }
    }
}

void
execute_riscv_instruction(RVInstruction *i, uint32_t *fflags)
{
    int32_t rm;

    /* For 16-bit compressed instructions */
    if ((i->binary & 3) != 3)
    {
        execute_ext_c_path(i);
        return;
    }

    /* For 32-bit integer and floating point instructions */
    switch (i->major_opcode)
    {

        case OP_IMM_MASK:
        {
            execute_op_imm(i);
            break;
        }

        case OP_IMM_32_MASK:
        {
            execute_op_imm_32(i);
            break;
        }

        case OP_MASK:
        {
            execute_op(i);
            break;
        }

        case OP_MASK_32:
        {
            execute_op_32(i);
            break;
        }

        case LUI_MASK:
        {
            i->buffer = i->imm;
            break;
        }

        case AUIPC_MASK:
        {
            i->buffer = (target_long)(i->pc + i->imm);
            break;
        }

        case LOAD_MASK:
        {
            i->mem_addr = i->rs1_val + i->imm;
            break;
        }
        case STORE_MASK:
        {
            i->mem_addr = i->rs1_val + i->imm;
            break;
        }

        case JAL_MASK:
        {
            i->buffer = i->pc + 4;
            i->target = i->pc + i->imm;
            break;
        }

        case JALR_MASK:
        {
            i->buffer = i->pc + 4;
            i->target = i->rs1_val + i->imm;
            break;
        }

        case BRANCH_MASK:
        {
            execute_type_b(i);
            break;
        }

        case ATOMIC_MASK:
        {
            i->mem_addr = i->rs1_val;
            break;
        }

        case FLOAD_MASK:
        {
            i->mem_addr = i->rs1_val + i->imm;
            break;
        }

        case FSTORE_MASK:
        {
            i->mem_addr = i->rs1_val + i->imm;
            break;
        }

        case 0x43:
        { /* fmadd */
            rm = i->rm;
            switch (i->funct3)
            {
                case 0:
                {
                    i->buffer = fma_sf32(i->rs1_val, i->rs2_val, i->rs3_val,
                                         (RoundingModeEnum)rm, fflags)
                                | F32_HIGH;
                    break;
                }
                case 1:
                {
                    i->buffer = fma_sf64(i->rs1_val, i->rs2_val, i->rs3_val,
                                         (RoundingModeEnum)rm, fflags)
                                | F64_HIGH;
                    break;
                }
            }
            break;
        }

        case 0x47:
        { /* fmsub*/
            rm = i->rm;
            switch (i->funct3)
            {
                case 0:
                {
                    i->buffer = fma_sf32(i->rs1_val, i->rs2_val,
                                         i->rs3_val ^ FSIGN_MASK32,
                                         (RoundingModeEnum)rm, fflags)
                                | F32_HIGH;
                    break;
                }
                case 1:
                {
                    i->buffer = fma_sf64(i->rs1_val, i->rs2_val,
                                         i->rs3_val ^ FSIGN_MASK64,
                                         (RoundingModeEnum)rm, fflags)
                                | F64_HIGH;
                }
                break;
            }
            break;
        }

        case 0x4b:
        { /* fnmsub */
            rm = i->rm;
            switch (i->funct3)
            {
                case 0:
                {
                    i->buffer
                        = fma_sf32(i->rs1_val ^ FSIGN_MASK32, i->rs2_val,
                                   i->rs3_val, (RoundingModeEnum)rm, fflags)
                          | F32_HIGH;
                    break;
                }

                case 1:
                {
                    i->buffer
                        = fma_sf64(i->rs1_val ^ FSIGN_MASK64, i->rs2_val,
                                   i->rs3_val, (RoundingModeEnum)rm, fflags)
                          | F64_HIGH;
                    break;
                }
            }
            break;
        }

        case 0x4f:
        { /* fmadd */
            rm = i->rm;
            switch (i->funct3)
            {
                case 0:
                {
                    i->buffer = fma_sf32(i->rs1_val ^ FSIGN_MASK32, i->rs2_val,
                                         i->rs3_val ^ FSIGN_MASK32,
                                         (RoundingModeEnum)rm, fflags)
                                | F32_HIGH;
                    break;
                }
                case 1:
                {
                    i->buffer = fma_sf64(i->rs1_val ^ FSIGN_MASK64, i->rs2_val,
                                         i->rs3_val ^ FSIGN_MASK64,
                                         (RoundingModeEnum)rm, fflags)
                                | F64_HIGH;
                    break;
                }
            }
            break;
        }

        case F_ARITHMETIC_MASK:
        {
            rm = i->rm;
            switch (i->funct7)
            {
#define F_SIZE 32
#include "fp_execute_template.h"
#if SIM_FLEN >= 64
#define F_SIZE 64
#include "fp_execute_template.h"
#endif
            }
            break;
        }
    }
}