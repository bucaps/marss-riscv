/**
 * RISCV Instruction String Generator
 *
 * Copyright (c) 2016-2017 Fabrice Bellard
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "riscv_instruction.h"

static const char reg[][6] = {
    "zero", // x0
    "ra",   // x1
    "sp",   // x2
    "gp",   // x3
    "tp",   // x4
    "t0",   // x5
    "t1",   // x6
    "t2",   // x7
    "s0",   // x8
    "s1",   // x9
    "a0",   // x10
    "a1",   // x11
    "a2",   // x12
    "a3",   // x13
    "a4",   // x14
    "a5",   // x15
    "a6",   // x16
    "a7",   // x17
    "s2",   // x18
    "s3",   // x19
    "s4",   // x20
    "s5",   // x21
    "s6",   // x22
    "s7",   // x23
    "s8",   // x24
    "s9",   // x25
    "s10",  // x26
    "s11",  // x27
    "t3",   // x28
    "t4",   // x29
    "t5",   // x30
    "t6"    // x31
};

static const char fp_reg[][6] = {
    "ft0",  // fp0
    "ft1",  // fp1
    "ft2",  // fp2
    "ft3",  // fp3
    "ft4",  // fp4
    "ft5",  // fp5
    "ft6",  // fp6
    "ft7",  // fp7
    "fs0",  // fp8
    "fs1",  // fp9
    "fa0",  // fp10
    "fa1",  // fp11
    "fa2",  // fp12
    "fa3",  // fp13
    "fa4",  // fp14
    "fa5",  // fp15
    "fa6",  // fp16
    "fa7",  // fp17
    "fs2",  // fp18
    "fs3",  // fp19
    "fs4",  // fp20
    "fs5",  // fp21
    "fs6",  // fp22
    "fs7",  // fp23
    "fs8",  // fp24
    "fs9",  // fp25
    "fs10", // fp26
    "fs11", // fp27
    "ft8",  // fp28
    "ft9",  // fp29
    "ft10", // fp30
    "ft11"  // fp31
};

static const char get_xlen_suffix[][3] = {"", ".w", ".d"};
static const char get_flen_suffix[][3] = {"", ".s", ".d"};

static void
set_op_imm_str(RVInstruction *i)
{
    switch (i->funct3)
    {
        case 0: /* addi */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "addi %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
            break;
        }
        case 1: /* slli */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "slli %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
            break;
        }
        case 2: /* slti */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "slti %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
            break;
        }
        case 3: /* sltiu */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sltiu %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
            break;
        }
        case 4: /* xori */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "xori %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
            break;
        }
        case 5: /* srli/srai */
        {
            if (i->imm & 0x400)
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "srai %s,%s,%d",
                         reg[i->rd], reg[i->rs1], i->imm);
            }
            else
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "srli %s,%s,%d",
                         reg[i->rd], reg[i->rs1], i->imm);
            }
            break;
        }
        case 6: /* ori */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "ori %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
            break;
        }
        case 7: /* andi */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "andi %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
            break;
        }
    }
}

static void
set_op_imm_32_str(RVInstruction *i)
{
    switch (i->funct3)
    {
        case 0: /* addiw */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "addiw %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
            break;
        }
        case 1: /* slliw */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "slliw %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
            break;
        }
        case 5: /* srliw/sraiw */
        {
            if (i->imm & 0x400)
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sraiw %s,%s,%d",
                         reg[i->rd], reg[i->rs1], i->imm);
            }
            else
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "srliw %s,%s,%d",
                         reg[i->rd], reg[i->rs1], i->imm);
            }
            break;
        }
    }
}

static void
set_op_str(RVInstruction *i)
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
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "mul %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 1: /* mulh */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "mulh %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 2: /* mulhsu */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "mulhsu %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 3: /* mulhu */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "mulhu %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 4: /* div */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "div %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 5: /* divu */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "divu %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 6: /* rem */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "rem %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 7: /* remu */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "remu %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
        }
    }
    else
    {
        funct3 = ((insn >> 12) & 7) | ((insn >> (30 - 3)) & (1 << 3));
        switch (funct3)
        {
            case 0: /* add */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "add %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 0 | 8: /* sub */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sub %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 1: /* sll */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sll %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 2: /* slt */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "slt %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 3: /* sltu */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sltu %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 4: /* xor */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "xor %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 5: /* srl */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "srl %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 5 | 8: /* sra */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sra %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 6: /* or */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "or %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 7: /* and */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "and %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
        }
    }
}

static void
set_op_32_str(RVInstruction *i)
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
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "mulw %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 4: /* divw */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "divw %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 5: /* divuw */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "divuw %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 6: /* remw */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "remw %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 7: /* remuw */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "remuw %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
        }
    }
    else
    {
        funct3 = ((insn >> 12) & 7) | ((insn >> (30 - 3)) & (1 << 3));
        switch (funct3)
        {
            case 0: /* addw */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "addw %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 0 | 8: /* subw */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "subw %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 1: /* sllw */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sllw %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 5: /* srlw */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "srlw %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
            case 5 | 8: /* sraw */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sraw %s,%s,%s",
                         reg[i->rd], reg[i->rs1], reg[i->rs2]);
                break;
            }
        }
    }
}

static void
set_branch_str(RVInstruction *i)
{
    switch (i->funct3)
    {
        case 0: /* beq */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "beq %s,%s,%d",
                     reg[i->rs1], reg[i->rs2], i->imm);
            break;
        }
        case 1: /* bne */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "bne %s,%s,%d",
                     reg[i->rs1], reg[i->rs2], i->imm);
            break;
        }
        case 4: /* blt */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "blt %s,%s,%d",
                     reg[i->rs1], reg[i->rs2], i->imm);
            break;
        }
        case 5: /* bge */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "bge %s,%s,%d",
                     reg[i->rs1], reg[i->rs2], i->imm);
            break;
        }
        case 6: /* bltu */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "bltu %s,%s,%d",
                     reg[i->rs1], reg[i->rs2], i->imm);
            break;
        }
        case 7: /* bgeu */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "bgeu %s,%s,%d",
                     reg[i->rs1], reg[i->rs2], i->imm);
            break;
        }
    }
}

static void
set_ext_c_q0_str(RVInstruction *i)
{
    switch (i->funct3)
    {
        case 0: /* c.addi4spn */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.addi4spn %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
        }
        break;
#if SIM_FLEN >= 64
        case 1: /* c.fld */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.fld %s,%s,%d",
                     fp_reg[i->rd], reg[i->rs1], i->imm);
        }
        break;
#endif
        case 2: /* c.lw */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.lw %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
        }
        break;
#if defined(RV64)
        case 3: /* c.ld */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.ld %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
        }
        break;
#elif SIM_FLEN >= 32
        case 3: /* c.flw */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.flw %s,%s,%d",
                     fp_reg[i->rd], reg[i->rs1], i->imm);
        }
        break;
#endif
#if SIM_FLEN >= 64
        case 5: /* c.fsd */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.fsd %s,%s,%d",
                     reg[i->rs1], fp_reg[i->rs2], i->imm);
            break;
        }
#endif
        case 6: /* c.sw */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.sw %s,%s,%d",
                     reg[i->rs1], reg[i->rs2], i->imm);
            break;
        }
#if defined(RV64)
        case 7: /* c.sd */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.sd %s,%s,%d",
                     reg[i->rs1], reg[i->rs2], i->imm);
            break;
        }
#elif SIM_FLEN >= 32
        case 7: /* c.fsw */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.fsw %s,%s,%d",
                     reg[i->rs1], fp_reg[i->rs2], i->imm);
            break;
        }
#endif
    }
}

static void
set_ext_c_q1_str(RVInstruction *i)
{
    switch (i->funct3)
    {
        case 0: /* c.addi/c.nop */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.addi %s,%d",
                     reg[i->rd], i->imm);
            break;
        }
#if defined(RV32)
        case 1: /* c.jal */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.jal %d", i->imm);
            break;
        }
#elif defined(RV64)
        case 1: /* c.addiw */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.addiw %s,%d",
                     reg[i->rd], i->imm);
            break;
        }
#endif
        case 2: /* c.li */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.li %s,%d", reg[i->rd],
                     i->imm);
            break;
        }
        case 3:
        {
            if (i->rd == 2)
            {
                /* c.addi16sp */
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.addi16sp %s,%d",
                         reg[i->rd], i->imm);
            }
            else if (i->rd != 0)
            {
                /* c.lui */
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.lui %s,%d",
                         reg[i->rd], i->imm);
            }
            break;
        }
        case 4:
        {
            switch (i->funct4)
            {
                case 0: /* c.srli */
                case 1: /* c.srai */
                {
                    if (i->funct4 == 0)
                    {
                        snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                                 "c.srli %s,%d", reg[i->rd], i->imm);
                    }
                    else
                    {
                        snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                                 "c.srai %s,%d", reg[i->rd], i->imm);
                    }
                    break;
                }
                case 2: /* c.andi */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.andi %s,%d",
                             reg[i->rd], i->imm);
                    break;
                }
                case 3:
                {
                    switch (i->funct5)
                    {
                        case 0: /* c.sub */
                        {
                            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                                     "c.sub %s,%s", reg[i->rd], reg[i->rs2]);
                            break;
                        }
                        case 1: /* c.xor */
                        {
                            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                                     "c.xor %s,%s", reg[i->rd], reg[i->rs2]);
                            break;
                        }
                        case 2: /* c.or */
                        {
                            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                                     "c.or %s,%s", reg[i->rd], reg[i->rs2]);
                            break;
                        }
                        case 3: /* c.and */
                        {
                            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                                     "c.and %s,%s", reg[i->rd], reg[i->rs2]);
                            break;
                        }
#if defined(RV64)
                        case 4: /* c.subw */
                        {
                            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                                     "c.subw %s,%s", reg[i->rd], reg[i->rs2]);
                            break;
                        }
                        case 5: /* c.addw */
                        {
                            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                                     "c.addw %s,%s", reg[i->rd], reg[i->rs2]);
                            break;
                        }
#endif
                    }
                    break;
                }
            }
            break;
        }
        case 5: /* c.j */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.j %d", i->imm);
            break;
        }
        case 6: /* c.beqz */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.beqz %s,%d",
                     reg[i->rs1], i->imm);
            break;
        }
        case 7: /* c.bnez */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.bnez %s,%d",
                     reg[i->rs1], i->imm);
            break;
        }
    }
}

static void
set_ext_c_q2_str(RVInstruction *i)
{
    uint32_t insn = i->binary;

    switch (i->funct3)
    {
        case 0: /* c.slli */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.slli %s,%d",
                     reg[i->rd], i->imm);
            break;
        }
#if SIM_FLEN >= 64
        case 1: /* c.fldsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.fldsp %s,%d",
                     fp_reg[i->rd],i->imm);
            break;
        }
#endif
        case 2: /* c.lwsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.lwsp %s,%d",
                     reg[i->rd], i->imm);
            break;
        }
#if defined(RV64)
        case 3: /* c.ldsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.ldsp %s,%d",
                     reg[i->rd], i->imm);
            break;
        }
#elif SIM_FLEN >= 32
        case 3: /* c.flwsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.flwsp %s,%d",
                     fp_reg[i->rd], i->imm);
        }
        break;
#endif
        case 4:
        {
            if (((insn >> 12) & 1) == 0)
            {
                if (i->rs2 == 0)
                {
                    /* c.jr */
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.jr %s",
                             reg[i->rs1]);
                }
                else
                {
                    /* c.mv */
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.mv %s,%s",
                             reg[i->rd], reg[i->rs2]);
                }
            }
            else
            {
                if (i->rs2 == 0)
                {
                    if (i->rd == 0)
                    {
                        /* c.ebreak */
                        snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.break");
                    }
                    else
                    {
                        /* c.jalr */
                        snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.jalr %s",
                                 reg[i->rs1]);
                    }
                }
                else
                {
                    /* c.add */
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.add %s,%s",
                             reg[i->rd], reg[i->rs2]);
                }
            }
            break;
        }
#if SIM_FLEN >= 64
        case 5: /* c.fsdsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.fsdsp %s,%d",
                     fp_reg[i->rs2], i->imm);
            break;
        }
#endif
        case 6: /* c.swsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.swsp %s,%d",
                     reg[i->rs2], i->imm);
            break;
        }
#if defined(RV64)
        case 7: /* c.sdsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.sdsp %s,%d",
                     reg[i->rs2], i->imm);
            break;
        }
#elif SIM_FLEN >= 32
        case 7: /* c.fswsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.fswsp %s,%d",
                     fp_reg[i->rs2], i->imm);
            break;
        }
#endif
    }
}

static void
set_ext_c_str(RVInstruction *i)
{
    switch (i->quad)
    {
        case 0:
        {
            set_ext_c_q0_str(i);
            break;
        }
        case 1:
        {
            set_ext_c_q1_str(i);
            break;
        }
        case 2:
        {
            set_ext_c_q2_str(i);
            break;
        }
    }
}

static void
set_load_str(RVInstruction *i)
{
    switch (i->bytes_to_rw)
    {
        case 1:
        {
            if (i->is_unsigned)
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "lbu %s,%s,%d",
                         reg[i->rd], reg[i->rs1], i->imm);
            }
            else
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "lb %s,%s,%d",
                         reg[i->rd], reg[i->rs1], i->imm);
            }
            break;
        }

        case 2:
        {
            if (i->is_unsigned)
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "lhu %s,%s,%d",
                         reg[i->rd], reg[i->rs1], i->imm);
            }
            else
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "lh %s,%s,%d",
                         reg[i->rd], reg[i->rs1], i->imm);
            }
            break;
        }

        case 4:
        {
            if (i->is_unsigned)
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "lwu %s,%s,%d",
                         reg[i->rd], reg[i->rs1], i->imm);
            }
            else
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "lw %s,%s,%d",
                         reg[i->rd], reg[i->rs1], i->imm);
            }
            break;
        }

        case 8:
        {
            if (i->is_unsigned)
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "ldu %s,%s,%d",
                         reg[i->rd], reg[i->rs1], i->imm);
            }
            else
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "ld %s,%s,%d",
                         reg[i->rd], reg[i->rs1], i->imm);
            }
            break;
        }
    }
}

static void
set_store_str(RVInstruction *i)
{
    switch (i->bytes_to_rw)
    {
        case 1:
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sb %s,%s,%d",
                     reg[i->rs1], reg[i->rs2], i->imm);
            break;
        }

        case 2:
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sh %s,%s,%d",
                     reg[i->rs1], reg[i->rs2], i->imm);
            break;
        }

        case 4:
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sw %s,%s,%d",
                     reg[i->rs1], reg[i->rs2], i->imm);
            break;
        }

        case 8:
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sd %s,%s,%d",
                     reg[i->rs1], reg[i->rs2], i->imm);
            break;
        }
    }
}

static void
set_csr_str(RVInstruction *i)
{
    int32_t imm;
    uint32_t funct3;
    int has_reg_src1;

    funct3 = (i->binary >> 12) & 7;
    imm = i->binary >> 20;

    if (funct3 & 4)
    {
        /* CSR instructions with immediate */
        has_reg_src1 = 0;
    }
    else
    {
        /* CSR instructions with register source 1 */
        has_reg_src1 = 1;
    }

    funct3 &= 3;
    switch (funct3)
    {
        case 1: /* csrrw */
        {
            if (has_reg_src1)
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "csrrw %s,%d,%s",
                         reg[i->rd], imm, reg[i->rs1]);
            }
            else
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "csrrwi %s,%d,%d",
                         reg[i->rd], imm, i->rs1);
            }
            break;
        }
        case 2: /* csrrs */
        {
            if (has_reg_src1)
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "csrrs %s,%d,%s",
                         reg[i->rd], imm, reg[i->rs1]);
            }
            else
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "csrrsi %s,%d,%d",
                         reg[i->rd], imm, i->rs1);
            }
            break;
        }
        case 3: /* csrrc */
        {
            if (has_reg_src1)
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "csrrc %s,%d,%s",
                         reg[i->rd], imm, reg[i->rs1]);
            }
            else
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "csrrci %s,%d,%d",
                         reg[i->rd], imm, i->rs1);
            }
            break;
        }
        case 0:
        {
            switch (imm)
            {
                case 0x000: /* ecall */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "ecall");
                    break;
                }
                case 0x001: /* ebreak */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "ebreak");
                    break;
                }
                case 0x102: /* sret */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sret");
                    break;
                }
                case 0x302: /* mret */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "mret");
                    break;
                }
                case 0x105: /* wfi */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "wfi");
                    break;
                }
                default:
                {
                    if ((imm >> 5) == 0x09)
                    {
                        /* sfence.vma */
                        snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                                 "sfence.vma");
                    }
                    break;
                }
            }
            break;
        }
    }
}

static void
set_fence_str(RVInstruction *i)
{
    switch (i->funct3)
    {
        case 0: /* fence */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fence");
            break;
        }
        case 1: /* fence.i */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fence.i");
            break;
        }
    }
}

void
get_riscv_ins_str(RVInstruction *i)
{
    int32_t rm;

    /* For 16-bit compressed instructions */
    if ((i->binary & 3) != 3)
    {
        set_ext_c_str(i);
        return;
    }

    /* For 32-bit integer and floating point instructions */
    switch (i->major_opcode)
    {

        case OP_IMM_MASK:
        {
            set_op_imm_str(i);
            break;
        }

        case OP_IMM_32_MASK:
        {
            set_op_imm_32_str(i);
            break;
        }

        case OP_MASK:
        {
            set_op_str(i);
            break;
        }

        case OP_MASK_32:
        {
            set_op_32_str(i);
            break;
        }

        case LUI_MASK:
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "lui %s,%d", reg[i->rd],
                     i->imm);
            break;
        }

        case AUIPC_MASK:
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "auipc %s,%d",
                     reg[i->rd], i->imm);
            break;
        }

        case LOAD_MASK:
        {
            set_load_str(i);
            break;
        }
        case STORE_MASK:
        {
            set_store_str(i);
            break;
        }

        case JAL_MASK:
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "jal %s,%d", reg[i->rd],
                     i->imm);
            break;
        }

        case JALR_MASK:
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "jalr %s,%s,%d",
                     reg[i->rd], reg[i->rs1], i->imm);
            break;
        }

        case BRANCH_MASK:
        {
            set_branch_str(i);
            break;
        }

        case ATOMIC_MASK:
        {
            uint32_t funct3;
            funct3 = i->binary >> 27;
            switch (funct3)
            {
                case 2: /* lr.w */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "lr%s %s,%s",
                             get_xlen_suffix[(BIT_SIZE / 32)], reg[i->rd],
                             reg[i->rs1]);
                    break;
                }
                case 3: /* sc.w */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "sc%s %s,%s",
                             get_xlen_suffix[(BIT_SIZE / 32)], reg[i->rs1],
                             reg[i->rs1]);
                    break;
                }
                case 1: /* amiswap.w */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "amiswap%s %s,%s,%s",
                             get_xlen_suffix[(BIT_SIZE / 32)], reg[i->rd],
                             reg[i->rs1], reg[i->rs2]);
                    break;
                }
                case 0: /* amoadd.w */
                {

                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "amoadd%s %s,%s,%s",
                             get_xlen_suffix[(BIT_SIZE / 32)], reg[i->rd],
                             reg[i->rs1], reg[i->rs2]);
                    break;
                }
                case 4: /* amoxor.w */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "amoxor%s %s,%s,%s",
                             get_xlen_suffix[(BIT_SIZE / 32)], reg[i->rd],
                             reg[i->rs1], reg[i->rs2]);
                    break;
                }
                case 0xc: /* amoand.w */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "amoand%s %s,%s,%s",
                             get_xlen_suffix[(BIT_SIZE / 32)], reg[i->rd],
                             reg[i->rs1], reg[i->rs2]);
                    break;
                }
                case 0x8: /* amoor.w */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "amoor%s %s,%s,%s",
                             get_xlen_suffix[(BIT_SIZE / 32)], reg[i->rd],
                             reg[i->rs1], reg[i->rs2]);
                    break;
                }
                case 0x10: /* amomin.w */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "amomin%s %s,%s,%s",
                             get_xlen_suffix[(BIT_SIZE / 32)], reg[i->rd],
                             reg[i->rs1], reg[i->rs2]);
                    break;
                }
                case 0x14: /* amomax.w */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "amomax%s %s,%s,%s",
                             get_xlen_suffix[(BIT_SIZE / 32)], reg[i->rd],
                             reg[i->rs1], reg[i->rs2]);
                    break;
                }
                case 0x18: /* amominu.w */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "amominu%s %s,%s,%s",
                             get_xlen_suffix[(BIT_SIZE / 32)], reg[i->rd],
                             reg[i->rs1], reg[i->rs2]);
                    break;
                }
                case 0x1c: /* amomaxu.w */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "amomaxu%s %s,%s,%s",
                             get_xlen_suffix[(BIT_SIZE / 32)], reg[i->rd],
                             reg[i->rs1], reg[i->rs2]);
                    break;
                }
            }
            break;
        }

        case CSR_MASK:
        {
            set_csr_str(i);
            break;
        }

        case FENCE_MASK:
        {
            set_fence_str(i);
            break;
        }

        case FLOAD_MASK:
        {
            switch (i->funct3)
            {
                case 2: /* flw */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "flw %s,%s,%d",
                             fp_reg[i->rd], reg[i->rs1], i->imm);
                    break;
                }
#if SIM_FLEN >= 64
                case 3: /* fld */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fld %s,%s,%d",
                             fp_reg[i->rd], reg[i->rs1], i->imm);
                    break;
                }
#endif
            }
            break;
        }

        case FSTORE_MASK:
        {
            switch (i->funct3)
            {
                case 2: /* fsw */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fsw %s,%s,%d",
                             reg[i->rs1], fp_reg[i->rs2], i->imm);
                    break;
                }
#if SIM_FLEN >= 64
                case 3: /* fsd */
#endif
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fsd %s,%s,%d",
                             reg[i->rs1], fp_reg[i->rs2], i->imm);
                }
                break;
            }
            break;
        }

        case 0x43:
        { /* fmadd */
            rm = i->rm;
            switch (i->funct3)
            {
                case 0:
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "fmadd.s %s,%s,%s,%s", fp_reg[i->rd],
                             fp_reg[i->rs1], fp_reg[i->rs2], fp_reg[i->rs3]);
                    break;
                }
                case 1:
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "fmadd.d %s,%s,%s,%s", fp_reg[i->rd],
                             fp_reg[i->rs1], fp_reg[i->rs2], fp_reg[i->rs3]);
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
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "fmsub.s %s,%s,%s,%s", fp_reg[i->rd],
                             fp_reg[i->rs1], fp_reg[i->rs2], fp_reg[i->rs3]);
                    break;
                }
                case 1:
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "fmsub.d %s,%s,%s,%s", fp_reg[i->rd],
                             fp_reg[i->rs1], fp_reg[i->rs2], fp_reg[i->rs3]);
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
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "fnmsub.s %s,%s,%s,%s", fp_reg[i->rd],
                             fp_reg[i->rs1], fp_reg[i->rs2], fp_reg[i->rs3]);
                    break;
                }

                case 1:
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "fnmsub.d %s,%s,%s,%s", fp_reg[i->rd],
                             fp_reg[i->rs1], fp_reg[i->rs2], fp_reg[i->rs3]);
                    break;
                }
            }
            break;
        }

        case 0x4f:
        { /* fnmadd */
            rm = i->rm;
            switch (i->funct3)
            {
                case 0:
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "fnmadd.s %s,%s,%s,%s", fp_reg[i->rd],
                             fp_reg[i->rs1], fp_reg[i->rs2], fp_reg[i->rs3]);
                    break;
                }
                case 1:
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "fnmadd.d %s,%s,%s,%s", fp_reg[i->rd],
                             fp_reg[i->rs1], fp_reg[i->rs2], fp_reg[i->rs3]);
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
#include "fpa_str_creator_template.h"
#if SIM_FLEN >= 64
#define F_SIZE 64
#include "fpa_str_creator_template.h"
#endif
            }
            break;
        }
    }
}