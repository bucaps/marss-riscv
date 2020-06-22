/**
 * RISCV Instruction String Generator
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

typedef struct CsrName
{
    int i;
    const char *name;
    int n;
} CsrName;

static const CsrName csr_names[] = {
    {0x300,"mstatus"},   {0x100,"sstatus"}, {0x000,"ustatus"}, {0x200,"vsstatus"}, {0x600,"hstatus"},
    {0x301,"misa"},
    {0x302,"medeleg"},   {0x102,"sedeleg"},                                        {0x602,"hedeleg"},
    {0x303,"mideleg"},   {0x103,"sideleg"},                                        {0x603,"hideleg"},
    {0x304,"mie"},       {0x104,"sie"},     {0x004,"uie"},     {0x204,"vsie"},
    {0x305,"mtvec"},     {0x105,"stvec"},   {0x005,"utvec"},   {0x204,"vstvec"},
    {0x306,"mcounteren"},{0x106,"scounteren"},                                     {0x606,"hcounteren"},
    {0x340,"mscratch"},  {0x140,"sscratch"},{0x040,"uscratch"},{0x204,"vsscratch"},
    {0x341,"mepc"},      {0x141,"sepc"},    {0x041,"uepc"},    {0x204,"vsepc"},
    {0x342,"mcause"},    {0x142,"scause"},  {0x042,"ucause"},  {0x204,"vscause"},
    {0x343,"mtval"},     {0x143,"stval"},   {0x043,"utval"},   {0x204,"vstval"},
    {0x344,"mip"},       {0x144,"sip"},     {0x044,"uip"},     {0x204,"vsip"},
                         {0x180,"satp"},                       {0x280,"vsatp"},    {0x680,"hgatp"},
    {0xF11,"mvendorid"},
    {0xF12,"marchid"},
    {0xF13,"mimpid"},
    {0xF14,"mhartid"},
    {0x3A0,"pmpcfg%d",3},
    {0x3B0,"pmpaddr%d",15},
    {0x001,"fflags"}, {0x002,"frm"}, {0x003,"fcsr"},

    {0xB00,"mcycle"},    {0xC00,"cycle"},
    {0xB80,"mcycleh"},   {0xC80,"cycleh"},
                         {0xC01,"time"},
                         {0xC81,"timeh"},
    {0xB02,"minstret"},  {0xC02,"instret"},
    {0xB82,"minstreth"}, {0xC82,"instreth"},
    {0xB00,"mhpmcounter%d",31},  {0xC00,"hpmcounter%d",31},
    {0xB80,"mhpmcounter%dh",31}, {0xC80,"hpmcounter%dh",31},
    {0x320,"mcountinhibit"},
    {0x320,"mhpmevent%d",31},
    {0x7A0,"tselect"},  {0x7A1,"mcontrol"},  {0x7A2,"tdata2"},   {0x7A3,"textra"},
    {0x7A4,"tinfo"},    {0x7A5,"tcontrol"},  {0x7A8,"mcontext"}, {0x7AA,"scontext"},
    {0x7B0,"dcsr"},     {0x7B1,"dpc"},       {0x7B2,"dscratch%d",1},
    {0,0,0}
};

static const char size_suffix[8][3] = {"b", "h", "w", "d", "q", "5", "6", "7"};
static const char *const aqrl_suffix[4] = {"", ".rl", ".aq", ".aqrl"};
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
    const char *const branches[8]
        = {"beq", "bne", 0, 0, "blt", "bge", "bltu", "bgeu"};
    const char *s;

    if (i->funct3 < 8)
    {
        s = branches[i->funct3];
        if (s != 0)
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "%s %s,%s,%d", s,
                     reg[i->rs1], reg[i->rs2], i->imm);
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
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.fld %s,%d(%s)",
                     fp_reg[i->rd], i->imm, reg[i->rs1]);
        }
        break;
#endif
        case 2: /* c.lw */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.lw %s,%d(%s)",
                     reg[i->rd], i->imm, reg[i->rs1]);
        }
        break;
#if defined(RV64)
        case 3: /* c.ld */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.ld %s,%d(%s)",
                     reg[i->rd], i->imm, reg[i->rs1]);
        }
        break;
#elif SIM_FLEN >= 32
        case 3: /* c.flw */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.flw %s,%d(%s)",
                     fp_reg[i->rd], i->imm, reg[i->rs1]);
        }
        break;
#endif
#if SIM_FLEN >= 64
        case 5: /* c.fsd */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.fsd %s,%d(%s)",
                     fp_reg[i->rs2], i->imm, reg[i->rs1]);
            break;
        }
#endif
        case 6: /* c.sw */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.sw %s,%d(%s)",
                     reg[i->rs2], i->imm, reg[i->rs1]);
            break;
        }
#if defined(RV64)
        case 7: /* c.sd */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.sd %s,%d(%s)",
                     reg[i->rs2], i->imm, reg[i->rs1]);
            break;
        }
#elif SIM_FLEN >= 32
        case 7: /* c.fsw */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.fsw %s,%d(%s)",
                     fp_reg[i->rs2], i->imm, reg[i->rs1]);
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
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.fldsp %s,%d(sp)",
                     fp_reg[i->rd], i->imm);
            break;
        }
#endif
        case 2: /* c.lwsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.lwsp %s,%d(sp)",
                     reg[i->rd], i->imm);
            break;
        }
#if defined(RV64)
        case 3: /* c.ldsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.ldsp %s,%d(sp)",
                     reg[i->rd], i->imm);
            break;
        }
#elif SIM_FLEN >= 32
        case 3: /* c.flwsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.flwsp %s,%d(sp)",
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
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.fsdsp %s,%d(sp)",
                     fp_reg[i->rs2], i->imm);
            break;
        }
#endif
        case 6: /* c.swsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.swsp %s,%d(sp)",
                     reg[i->rs2], i->imm);
            break;
        }
#if defined(RV64)
        case 7: /* c.sdsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.sdsp %s,%d(sp)",
                     reg[i->rs2], i->imm);
            break;
        }
#elif SIM_FLEN >= 32
        case 7: /* c.fswsp */
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "c.fswsp %s,%d(sp)",
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
    const char *name;

    switch (i->bytes_to_rw)
    {
        case 1:
        {
            name = "lb";
            break;
        }
        case 2:
        {
            name = "lh";
            break;
        }
        case 4:
        {
            name = "lw";
            break;
        }
        case 8:
        {
            name = "ld";
            break;
        }
        default:
        {
            return;
        }
    }

    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "%s%s %s,%d(%s)", name,
             (i->is_unsigned ? "u" : ""), reg[i->rd], i->imm, reg[i->rs1]);
}

static void
set_store_str(RVInstruction *i)
{
    const char *name;

    switch (i->bytes_to_rw)
    {
        case 1:
        {
            name = "sb";
            break;
        }
        case 2:
        {
            name = "sh";
            break;
        }
        case 4:
        {
            name = "sw";
            break;
        }
        case 8:
        {
            name = "sd";
            break;
        }
        default:
        {
            return;
        }
    }

    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "%s %s,%d(%s)", name,
             reg[i->rs2], i->imm, reg[i->rs1]);
}

static void
set_csr_str(RVInstruction *i)
{
    const CsrName *csr;
    const char *name;
    char cname[20];
    int32_t imm;
    uint32_t funct3;
    int has_imm;

    funct3 = (i->binary >> 12) & 7;
    imm = i->binary >> 20;
    has_imm = (funct3 & 4);
    funct3 &= 3;

    switch (funct3)
    {
        case 1:
        {
            name = "csrrw";
            break;
        }
        case 2:
        {
            name = "csrrs";
            break;
        }
        case 3:
        {
            name = "csrrc";
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
            return;
        }
    }

    for (csr = csr_names; csr->i != 0 || csr->name != 0; csr++)
    {
        if ((imm & ~csr->n) == csr->i)
            break;
    }

    if (csr->name != 0)
    {
        snprintf(cname, sizeof(cname), csr->name, (imm & csr->n));
        if (has_imm)
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "%si %s,%s,%d", name,
                     reg[i->rd], cname, i->rs1);
        }
        else
        {
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "%s %s,%s,%s", name,
                     reg[i->rd], cname, reg[i->rs1]);
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
generate_riscv_instruction_string(RVInstruction *i)
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
            snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "lui %s,0x%x",
                     reg[i->rd], i->imm);
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
            uint32_t funct7 = (i->binary >> 27) & 0x1F;
            const char *size = size_suffix[(i->binary >> 12) & 7];
            const char *aqrl = aqrl_suffix[(i->binary >> 25) & 3];
            if (funct7 == 2) /* lr.[wd] */
            {
                snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "lr.%s%s %s,(%s)",
                         size, aqrl, reg[i->rd], reg[i->rs1]);
            }
            else
            {
                const char *const names[32]
                    = {"amoadd",  "amoswap", 0, "sc", "amoxor",  0, 0, 0,
                       "amoor",   0,         0, 0,    "amoand",  0, 0, 0,
                       "amomin",  0,         0, 0,    "amomax",  0, 0, 0,
                       "amominu", 0,         0, 0,    "amomaxu", 0, 0, 0};
                const char *name = names[funct7];
                if (name != 0)
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH,
                             "%s.%s%s %s,%s,(%s)", name, size, aqrl, reg[i->rd],
                             reg[i->rs2], reg[i->rs1]);
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
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "flw %s,%d(%s)",
                             fp_reg[i->rd], i->imm, reg[i->rs1]);
                    break;
                }
#if SIM_FLEN >= 64
                case 3: /* fld */
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fld %s,%d(%s)",
                             fp_reg[i->rd], i->imm, reg[i->rs1]);
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
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fsw %s,%d(%s)",
                             fp_reg[i->rs2], i->imm, reg[i->rs1]);
                    break;
                }
#if SIM_FLEN >= 64
                case 3: /* fsd */
#endif
                {
                    snprintf(i->str, RISCV_INS_STR_MAX_LENGTH, "fsd %s,%d(%s)",
                             fp_reg[i->rs2], i->imm, reg[i->rs1]);
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
#include "fp_string_generator_template.h"
#if SIM_FLEN >= 64
#define F_SIZE 64
#include "fp_string_generator_template.h"
#endif
            }
            break;
        }
    }
}
