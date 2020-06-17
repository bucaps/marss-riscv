/**
 * RISC-V Instruction Decoding Library
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
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../cutils.h"
#include "riscv_ins_str_creator.h"
#include "riscv_instruction.h"
#include "riscv_sim_typedefs.h"

static inline int32_t
sextc(int32_t val, int n)
{
    return (val << (32 - n)) >> (32 - n);
}

static inline uint32_t
cget_field1(uint32_t val, int src_pos, int dst_pos, int dst_pos_max)
{
    int mask;
    assert(dst_pos_max >= dst_pos);
    mask = ((1 << (dst_pos_max - dst_pos + 1)) - 1) << dst_pos;
    if (dst_pos >= src_pos)
        return (val << (dst_pos - src_pos)) & mask;
    else
        return (val >> (src_pos - dst_pos)) & mask;
}

static void
decode_compressed_q0(struct RVInstruction *ins)
{
    uint32_t insn, rd, rs1, rs2, funct3, quad;
    int32_t imm;

    insn = ins->binary;
    funct3 = (insn >> 13) & 7;
    rd = ((insn >> 2) & 7) | 8;
    rs2 = rd;
    quad = C_QUADRANT0;
    switch (funct3)
    {
        case 0: /* c.addi4spn */
        {
            ins->has_dest = TRUE;
            ins->has_src1 = TRUE;
            rs1 = 2;
            imm = cget_field1(insn, 11, 4, 5) | cget_field1(insn, 7, 6, 9)
                  | cget_field1(insn, 6, 2, 2) | cget_field1(insn, 5, 3, 3);
            if (imm == 0)
                goto illegal_insn;
            break;
        }
#if SIM_FLEN >= 64
        case 1: /* c.fld */
        {
            if (ins->current_fs == 0)
                goto illegal_insn;
            ins->is_load = TRUE;
            ins->has_fp_dest = TRUE;
            ins->has_src1 = TRUE;
            ins->bytes_to_rw = 8;
            ins->f64_mask = TRUE;
            ins->set_fs = TRUE;
            ins->type = INS_TYPE_FP_LOAD;
            imm = cget_field1(insn, 10, 3, 5) | cget_field1(insn, 5, 6, 7);
            rs1 = ((insn >> 7) & 7) | 8;
            break;
        }
#endif
        case 2: /* c.lw */
        {
            ins->is_load = TRUE;
            ins->has_dest = TRUE;
            ins->has_src1 = TRUE;
            ins->bytes_to_rw = 4;
            ins->type = INS_TYPE_LOAD;
            imm = cget_field1(insn, 10, 3, 5) | cget_field1(insn, 6, 2, 2)
                  | cget_field1(insn, 5, 6, 6);
            rs1 = ((insn >> 7) & 7) | 8;
            break;
        }
#if defined(RV64)
        case 3: /* c.ld */
        {
            ins->is_load = TRUE;
            ins->has_dest = TRUE;
            ins->has_src1 = TRUE;
            ins->bytes_to_rw = 8;
            ins->type = INS_TYPE_LOAD;
            imm = cget_field1(insn, 10, 3, 5) | cget_field1(insn, 5, 6, 7);
            rs1 = ((insn >> 7) & 7) | 8;
            break;
        }
#elif SIM_FLEN >= 32
        case 3: /* c.flw */
        {
            ins->is_load = TRUE;
            ins->has_fp_dest = TRUE;
            ins->has_src1 = TRUE;
            ins->bytes_to_rw = 4;
            ins->f32_mask = TRUE;
            ins->set_fs = TRUE;
            ins->type = INS_TYPE_FP_LOAD;
            imm = cget_field1(insn, 10, 3, 5) | cget_field1(insn, 6, 2, 2)
                  | cget_field1(insn, 5, 6, 6);
            rs1 = ((insn >> 7) & 7) | 8;
            break;
        }
#endif
#if SIM_FLEN >= 64
        case 5: /* c.fsd */
        {
            ins->is_store = TRUE;
            ins->has_src1 = TRUE;
            ins->has_fp_src2 = TRUE;
            ins->bytes_to_rw = 8;
            ins->type = INS_TYPE_FP_STORE;
            imm = cget_field1(insn, 10, 3, 5) | cget_field1(insn, 5, 6, 7);
            rs1 = ((insn >> 7) & 7) | 8;
            break;
        }
#endif
        case 6: /* c.sw */
        {
            ins->is_store = TRUE;
            ins->has_src1 = TRUE;
            ins->has_src2 = TRUE;
            ins->bytes_to_rw = 4;
            ins->type = INS_TYPE_STORE;
            imm = cget_field1(insn, 10, 3, 5) | cget_field1(insn, 6, 2, 2)
                  | cget_field1(insn, 5, 6, 6);
            rs1 = ((insn >> 7) & 7) | 8;
            break;
        }
#if defined(RV64)
        case 7: /* c.sd */
        {
            ins->is_store = TRUE;
            ins->has_src1 = TRUE;
            ins->has_src2 = TRUE;
            ins->bytes_to_rw = 8;
            ins->type = INS_TYPE_STORE;
            imm = cget_field1(insn, 10, 3, 5) | cget_field1(insn, 5, 6, 7);
            rs1 = ((insn >> 7) & 7) | 8;
            break;
        }
#elif SIM_FLEN >= 32
        case 7: /* c.fsw */
        {
            ins->is_store = TRUE;
            ins->has_src1 = TRUE;
            ins->has_fp_src2 = TRUE;
            ins->bytes_to_rw = 4;
            ins->type = INS_TYPE_FP_STORE;
            imm = cget_field1(insn, 10, 3, 5) | cget_field1(insn, 6, 2, 2)
                  | cget_field1(insn, 5, 6, 6);
            rs1 = ((insn >> 7) & 7) | 8;
            break;
        }
#endif
        default:
            goto illegal_insn;
    }
    ins->rd = rd;
    ins->rs1 = rs1;
    ins->rs2 = rs2;
    ins->funct3 = funct3;
    ins->imm = imm;
    ins->quad = quad;
    return;
illegal_insn:
    ins->exception = TRUE;
    ins->exception_cause = SIM_ILLEGAL_OPCODE;
}

static void
decode_compressed_q1(struct RVInstruction *ins)
{
    uint32_t insn, rd, rs1, rs2 = 0, funct3, funct4 = 0, funct5 = 0, quad;
    int32_t imm = 0;

    insn = ins->binary;
    funct3 = (insn >> 13) & 7;
    rd = (insn >> 7) & 0x1f;
    rs1 = rd;
    quad = C_QUADRANT1;
    switch (funct3)
    {
        case 0: /* c.addi/c.nop */
        {
            ins->has_dest = TRUE;
            ins->has_src1 = TRUE;
            imm = sextc(
                cget_field1(insn, 12, 5, 5) | cget_field1(insn, 2, 0, 4), 6);
            break;
        }
#if defined(RV32)
        case 1: /* c.jal */
        {
            ins->has_dest = TRUE;
            ins->is_branch = TRUE;
            ins->branch_type = BRANCH_UNCOND;
            ins->type = INS_TYPE_JAL;
            rd = 1;
            imm = sextc(
                cget_field1(insn, 12, 11, 11) | cget_field1(insn, 11, 4, 4)
                    | cget_field1(insn, 9, 8, 9) | cget_field1(insn, 8, 10, 10)
                    | cget_field1(insn, 7, 6, 6) | cget_field1(insn, 6, 7, 7)
                    | cget_field1(insn, 3, 1, 3) | cget_field1(insn, 2, 5, 5),
                12);
            ins->is_func_call = TRUE;
            break;
        }
#elif defined(RV64)
        case 1: /* c.addiw */
        {
            ins->has_dest = TRUE;
            ins->has_src1 = TRUE;
            imm = sextc(
                cget_field1(insn, 12, 5, 5) | cget_field1(insn, 2, 0, 4), 6);
            break;
        }
#endif
        case 2: /* c.li */
        {
            ins->has_dest = TRUE;
            imm = sextc(
                cget_field1(insn, 12, 5, 5) | cget_field1(insn, 2, 0, 4), 6);
            break;
        }
        case 3:
        {
            if (rd == 2)
            {
                /* c.addi16sp */
                ins->has_dest = TRUE;
                ins->has_src1 = TRUE;
                rd = 2;
                rs1 = rd;
                imm = sextc(cget_field1(insn, 12, 9, 9)
                                | cget_field1(insn, 6, 4, 4)
                                | cget_field1(insn, 5, 6, 6)
                                | cget_field1(insn, 3, 7, 8)
                                | cget_field1(insn, 2, 5, 5),
                            10);
                if (imm == 0)
                    goto illegal_insn;
            }
            else if (rd != 0)
            {
                /* c.lui */
                ins->has_dest = TRUE;
                imm = sextc(cget_field1(insn, 12, 17, 17)
                                | cget_field1(insn, 2, 12, 16),
                            18);
            }
            break;
        }
        case 4:
        {
            ins->has_dest = TRUE;
            ins->has_src1 = TRUE;
            funct4 = (insn >> 10) & 3;
            rd = ((insn >> 7) & 7) | 8;
            rs1 = rd;
            switch (funct4)
            {
                case 0: /* c.srli */
                case 1: /* c.srai */
                    imm = cget_field1(insn, 12, 5, 5)
                          | cget_field1(insn, 2, 0, 4);
#if defined(RV32)
                    if (imm & 0x20)
                        goto illegal_insn;
#endif
                    break;
                case 2: /* c.andi */
                    imm = sextc(cget_field1(insn, 12, 5, 5)
                                    | cget_field1(insn, 2, 0, 4),
                                6);
                    break;
                case 3:
                    rs2 = ((insn >> 2) & 7) | 8;
                    ins->has_src2 = TRUE;
                    funct5 = ((insn >> 5) & 3) | ((insn >> (12 - 2)) & 4);
                    switch (funct5)
                    {
                        case 0: /* c.sub */
                        case 1: /* c.xor */
                        case 2: /* c.or */
                        case 3: /* c.and */
#if defined(RV64)
                        case 4: /* c.subw */
                        case 5: /* c.addw */
#endif
                            break;
                        default:
                            goto illegal_insn;
                    }
                    break;
            }
            break;
        }
        case 5: /* c.j */
        {
            ins->is_branch = TRUE;
            ins->branch_type = BRANCH_UNCOND;
            ins->type = INS_TYPE_JAL;
            imm = sextc(
                cget_field1(insn, 12, 11, 11) | cget_field1(insn, 11, 4, 4)
                    | cget_field1(insn, 9, 8, 9) | cget_field1(insn, 8, 10, 10)
                    | cget_field1(insn, 7, 6, 6) | cget_field1(insn, 6, 7, 7)
                    | cget_field1(insn, 3, 1, 3) | cget_field1(insn, 2, 5, 5),
                12);
            break;
        }
        case 6: /* c.beqz */
        {
            ins->is_branch = TRUE;
            ins->branch_type = BRANCH_COND;
            ins->type = INS_TYPE_COND_BRANCH;
            ins->has_src1 = TRUE;
            rs1 = ((insn >> 7) & 7) | 8;
            rs2 = 0;
            imm = sextc(
                cget_field1(insn, 12, 8, 8) | cget_field1(insn, 10, 3, 4)
                    | cget_field1(insn, 5, 6, 7) | cget_field1(insn, 3, 1, 2)
                    | cget_field1(insn, 2, 5, 5),
                9);
            break;
        }
        case 7: /* c.bnez */
        {
            ins->is_branch = TRUE;
            ins->branch_type = BRANCH_COND;
            ins->type = INS_TYPE_COND_BRANCH;
            ins->has_src1 = TRUE;
            rs1 = ((insn >> 7) & 7) | 8;
            rs2 = 0;
            imm = sextc(
                cget_field1(insn, 12, 8, 8) | cget_field1(insn, 10, 3, 4)
                    | cget_field1(insn, 5, 6, 7) | cget_field1(insn, 3, 1, 2)
                    | cget_field1(insn, 2, 5, 5),
                9);
            break;
        }
        default:
        {
            goto illegal_insn;
        }
    }
    ins->rd = rd;
    ins->rs1 = rs1;
    ins->rs2 = rs2;
    ins->imm = imm;
    ins->quad = quad;
    ins->funct3 = funct3;
    ins->funct4 = funct4;
    ins->funct5 = funct5;
    return;
illegal_insn:
    ins->exception = TRUE;
    ins->exception_cause = SIM_ILLEGAL_OPCODE;
}

static void
decode_compressed_q2(struct RVInstruction *ins)
{
    uint32_t insn, rd, rs1, rs2, funct3, quad;
    int32_t imm = 0;

    insn = ins->binary;
    quad = C_QUADRANT2;
    funct3 = (insn >> 13) & 7;
    rd = (insn >> 7) & 0x1f;
    rs1 = rd;
    rs2 = (insn >> 2) & 0x1f;
    switch (funct3)
    {
        case 0: /* c.slli */
        {
            ins->has_dest = TRUE;
            ins->has_src1 = TRUE;
            rs1 = rd;
            imm = cget_field1(insn, 12, 5, 5) | rs2;
#if XLEN == 32
            if (imm & 0x20)
                goto illegal_insn;
#endif
            break;
        }
#if SIM_FLEN >= 64
        case 1: /* c.fldsp */
        {
            ins->is_load = TRUE;
            ins->has_fp_dest = TRUE;
            ins->has_src1 = TRUE;
            ins->bytes_to_rw = 8;
            ins->f64_mask = TRUE;
            ins->set_fs = TRUE;
            ins->type = INS_TYPE_FP_LOAD;
            rs1 = 2;
            imm = cget_field1(insn, 12, 5, 5) | (rs2 & (3 << 3))
                  | cget_field1(insn, 2, 6, 8);
            break;
        }
#endif
        case 2: /* c.lwsp */
        {
            ins->is_load = TRUE;
            ins->bytes_to_rw = 4;
            ins->has_dest = TRUE;
            ins->has_src1 = TRUE;
            ins->type = INS_TYPE_LOAD;
            rs1 = 2;
            imm = cget_field1(insn, 12, 5, 5) | (rs2 & (7 << 2))
                  | cget_field1(insn, 2, 6, 7);
            break;
        }
#if defined(RV64)
        case 3: /* c.ldsp */
        {
            ins->is_load = TRUE;
            ins->bytes_to_rw = 8;
            ins->has_dest = TRUE;
            ins->has_src1 = TRUE;
            ins->type = INS_TYPE_LOAD;
            rs1 = 2;
            imm = cget_field1(insn, 12, 5, 5) | (rs2 & (3 << 3))
                  | cget_field1(insn, 2, 6, 8);
            break;
        }
#elif SIM_FLEN >= 32
        case 3: /* c.flwsp */
        {
            ins->is_load = TRUE;
            ins->has_fp_dest = TRUE;
            ins->has_src1 = TRUE;
            ins->bytes_to_rw = 4;
            ins->f32_mask = TRUE;
            ins->set_fs = TRUE;
            ins->type = INS_TYPE_FP_LOAD;
            rs1 = 2;
            imm = cget_field1(insn, 12, 5, 5) | (rs2 & (7 << 2))
                  | cget_field1(insn, 2, 6, 7);
            break;
        }
#endif
        case 4:
        {
            if (((insn >> 12) & 1) == 0)
            {
                if (rs2 == 0)
                {
                    /* c.jr */
                    ins->is_branch = TRUE;
                    ins->branch_type = BRANCH_UNCOND;
                    ins->type = INS_TYPE_JALR;
                    ins->has_src1 = TRUE;
                    if (rd == 0)
                        goto illegal_insn;
                    if (rs1 == 1)
                    {
                        ins->is_func_ret = TRUE;
                    }
                }
                else
                {
                    /* c.mv */
                    ins->has_dest = TRUE;
                    ins->has_src2 = TRUE;
                }
            }
            else
            {
                if (rs2 == 0)
                {
                    if (rd == 0)
                    {
                        /* c.ebreak */
                        ins->exception = TRUE;
                        ins->exception_cause = SIM_COMPLEX_OPCODE;
                        ins->type = INS_TYPE_SYSTEM;
                    }
                    else
                    {
                        /* c.jalr */
                        ins->is_branch = TRUE;
                        ins->branch_type = BRANCH_UNCOND;
                        ins->type = INS_TYPE_JALR;
                        ins->has_dest = TRUE;
                        ins->has_src1 = TRUE;
                        rd = 1;
                        ins->is_func_call = TRUE;
                    }
                }
                else
                {
                    /* c.add */
                    ins->has_dest = TRUE;
                    ins->has_src1 = TRUE;
                    ins->has_src2 = TRUE;
                }
            }
            break;
        }
#if SIM_FLEN >= 64
        case 5: /* c.fsdsp */
        {
            ins->is_store = TRUE;
            ins->has_src1 = TRUE;
            ins->has_fp_src2 = TRUE;
            ins->bytes_to_rw = 8;
            ins->type = INS_TYPE_FP_STORE;
            rs1 = 2;
            imm = cget_field1(insn, 10, 3, 5) | cget_field1(insn, 7, 6, 8);
            break;
        }
#endif
        case 6: /* c.swsp */
        {
            ins->is_store = TRUE;
            ins->bytes_to_rw = 4;
            ins->has_src1 = TRUE;
            ins->has_src2 = TRUE;
            ins->type = INS_TYPE_STORE;
            rs1 = 2;
            imm = cget_field1(insn, 9, 2, 5) | cget_field1(insn, 7, 6, 7);
            break;
        }
#if defined(RV64)
        case 7: /* c.sdsp */
        {
            ins->is_store = TRUE;
            ins->bytes_to_rw = 8;
            ins->has_src1 = TRUE;
            ins->has_src2 = TRUE;
            ins->type = INS_TYPE_STORE;
            rs1 = 2;
            imm = cget_field1(insn, 10, 3, 5) | cget_field1(insn, 7, 6, 8);
            break;
        }
#elif SIM_FLEN >= 32
        case 7: /* c.fswsp */
        {
            ins->is_store = TRUE;
            ins->has_src1 = TRUE;
            ins->has_fp_src2 = TRUE;
            ins->bytes_to_rw = 4;
            ins->type = INS_TYPE_FP_STORE;
            rs1 = 2;
            imm = cget_field1(insn, 9, 2, 5) | cget_field1(insn, 7, 6, 7);
            break;
        }
#endif
        default:
        {
            goto illegal_insn;
        }
    }
    ins->rd = rd;
    ins->rs1 = rs1;
    ins->rs2 = rs2;
    ins->imm = imm;
    ins->quad = quad;
    ins->funct3 = funct3;
    return;
illegal_insn:
    ins->exception = TRUE;
    ins->exception_cause = SIM_ILLEGAL_OPCODE;
}

static void
decode_compressed_type(struct RVInstruction *ins)
{
    int quad = ins->binary & 3;
    switch (quad)
    {
        case C_QUADRANT0:
        {
            decode_compressed_q0(ins);
            break;
        }
        case C_QUADRANT1:
        {
            decode_compressed_q1(ins);
            break;
        }
        case C_QUADRANT2:
        {
            decode_compressed_q2(ins);
            break;
        }
        default:
        {
            exit(1);
        }
    }
}

static void
set_op_fu(RVInstruction *i)
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
            case 1: /* mulh */
            case 2: /* mulhsu */
            case 3: /* mulhu */
                i->fu_type = FU_MUL;
                i->type = INS_TYPE_INT_MUL;
                break;
            case 4: /* div */
            case 5: /* divu */
            case 6: /* rem */
            case 7: /* remu */
                i->fu_type = FU_DIV;
                i->type = INS_TYPE_INT_DIV;
                break;
        }
    }
}

static int
chk_op_imm_exceptions(RVInstruction *i, uint32_t bit_size)
{
    uint32_t funct3 = (i->binary >> 12) & 7;

    switch (funct3)
    {
        case 1: /* slli */
            if ((i->imm & ~(bit_size - 1)) != 0)
            {
                return -1;
            }
            break;
        case 5: /* srli/srai */
            if ((i->imm & ~((bit_size - 1) | 0x400)) != 0)
            {
                return -1;
            }
    }
    return 0;
}

static int
chk_op_exceptions(RVInstruction *i)
{
    int32_t imm = i->binary >> 25;

    if (imm != 1)
    {
        if (imm & ~0x20)
        {
            return -1;
        }
    }
    return 0;
}

/**
 * @param  Encoded 32-bit instruction binary
 * @return Decoded RVInstruction
 */
void
decode_riscv_binary(struct RVInstruction *ins, uint32_t insn)
{
    ins->binary = insn;
    ins->fu_type = FU_ALU;
    ins->data_class = INS_CLASS_INT;
    ins->type = INS_TYPE_ARITMETIC;
    if ((ins->binary & 3) != 3)
    {
        /* Compressed Instruction */
        decode_compressed_type(ins);
    }
    else
    {
        /* 32-bit Instruction */
        ins->major_opcode = insn & 0x7f;
        ins->funct3 = (insn >> 12) & 7;
        ins->funct7 = (ins->binary & 0xfe000000) >> 25;
        ins->rd = (insn >> 7) & 0x1f;
        ins->rs1 = (insn >> 15) & 0x1f;
        ins->rs2 = (insn >> 20) & 0x1f;
        switch (ins->major_opcode)
        {
            case LOAD_MASK:
            {
                ins->is_load = TRUE;
                ins->has_src1 = TRUE;
                ins->has_dest = TRUE;
                ins->imm = (int32_t)insn >> 20;
                ins->type = INS_TYPE_LOAD;
                switch (ins->funct3)
                {
                    case 0x0: /* lb */
                    {
                        ins->bytes_to_rw = 1;
                        break;
                    }
                    case 0x1: /* lh */
                    {
                        ins->bytes_to_rw = 2;
                        break;
                    }
                    case 0x2: /* lw */
                    {
                        ins->bytes_to_rw = 4;
                        break;
                    }
#if defined(RV64)
                    case 0x3: /* ld */
                    {
                        ins->bytes_to_rw = 8;
                        break;
                    }
#endif
                    case 0x4: /* lbu */
                    {
                        ins->bytes_to_rw = 1;
                        ins->is_unsigned = TRUE;
                        break;
                    }
                    case 0x5: /* lhu */
                    {
                        ins->bytes_to_rw = 2;
                        ins->is_unsigned = TRUE;
                        break;
                    }
                    case 0x6: /* lwu */
                    {
                        ins->bytes_to_rw = 4;
                        ins->is_unsigned = TRUE;
                        break;
                    }
                }
                break;
            }
            case OP_IMM_MASK:
            case OP_IMM_32_MASK:
            {
                if (ins->major_opcode == OP_IMM_MASK)
                {
                    if (chk_op_imm_exceptions(ins, BIT_SIZE))
                    {
                        goto exception;
                    }
                }
                else
                {
                    if (chk_op_imm_exceptions(ins, 32))
                    {
                        goto exception;
                    }
                }

                ins->has_src1 = TRUE;
                ins->has_dest = TRUE;
                ins->imm = (int32_t)insn >> 20;
                break;
            }
            case OP_MASK:
            case OP_MASK_32:
            {
                if (chk_op_exceptions(ins))
                {
                    goto exception;
                }

                ins->has_src1 = TRUE;
                ins->has_src2 = TRUE;
                ins->has_dest = TRUE;
                /* set the functional units for mul and div */
                set_op_fu(ins);
                break;
            }
            case LUI_MASK:
            case AUIPC_MASK:
            {
                ins->has_dest = TRUE;
                ins->imm = (int32_t)(insn & 0xfffff000);
                break;
            }
            case STORE_MASK:
            {
                ins->is_store = TRUE;
                ins->has_src1 = TRUE;
                ins->has_src2 = TRUE;
                ins->imm = ins->rd | ((insn >> (25 - 5)) & 0xfe0);
                ins->imm = (ins->imm << 20) >> 20;
                ins->type = INS_TYPE_STORE;
                switch (ins->funct3)
                {
                    case 0x0: /* sb */
                    {
                        ins->bytes_to_rw = 1;
                        break;
                    }
                    case 0x1: /* sh */
                    {
                        ins->bytes_to_rw = 2;
                        break;
                    }
                    case 0x2: /* sw */
                    {
                        ins->bytes_to_rw = 4;
                        break;
                    }
#if defined(RV64)
                    case 0x3: /* sd */
                    {
                        ins->bytes_to_rw = 8;
                        break;
                    }
#endif
                }
                break;
            }
            case CSR_MASK:
            {
                ins->is_system = TRUE;
                ins->exception = TRUE;
                ins->type = INS_TYPE_SYSTEM;

                /* Complex Opcode */
                ins->exception_cause = SIM_COMPLEX_OPCODE;
                break;
            }
            case FENCE_MASK:
            {
                ins->is_system = TRUE;
                ins->exception = TRUE;
                ins->type = INS_TYPE_SYSTEM;

                /* Complex Opcode */
                ins->exception_cause = SIM_COMPLEX_OPCODE;
                break;
            }

            case JAL_MASK:
            {
                ins->is_branch = TRUE;
                ins->branch_type = BRANCH_UNCOND;
                ins->has_dest = TRUE;
                ins->imm = ((insn >> (31 - 20)) & (1 << 20))
                           | ((insn >> (21 - 1)) & 0x7fe)
                           | ((insn >> (20 - 11)) & (1 << 11))
                           | (insn & 0xff000);
                ins->imm = (ins->imm << 11) >> 11;
                ins->type = INS_TYPE_JAL;
                if (ins->rd == 1)
                {
                    ins->is_func_call = TRUE;
                }
                break;
            }
            case JALR_MASK:
            {
                ins->is_branch = TRUE;
                ins->branch_type = BRANCH_UNCOND;
                ins->has_src1 = TRUE;
                ins->has_dest = TRUE;
                ins->imm = (int32_t)insn >> 20;
                ins->type = INS_TYPE_JALR;
                if (ins->rd == 1)
                {
                    ins->is_func_call = TRUE;
                }
                if (ins->rs1 == 1)
                {
                    ins->is_func_ret = TRUE;
                }
                break;
            }
            case BRANCH_MASK:
            {
                ins->is_branch = TRUE;
                ins->branch_type = BRANCH_COND;
                ins->has_src1 = TRUE;
                ins->has_src2 = TRUE;
                ins->imm = ((insn >> (31 - 12)) & (1 << 12))
                           | ((insn >> (25 - 5)) & 0x7e0)
                           | ((insn >> (8 - 1)) & 0x1e)
                           | ((insn << (11 - 7)) & (1 << 11));
                ins->imm = (ins->imm << 19) >> 19;
                ins->type = INS_TYPE_COND_BRANCH;
                break;
            }
            case ATOMIC_MASK:
            {
                uint32_t funct3;

                ins->is_atomic = TRUE;
                ins->has_dest = TRUE;
                ins->type = INS_TYPE_ATOMIC;
                funct3 = (insn >> 12) & 7;
                switch (funct3)
                {
                    case 2:
#if BIT_SIZE >= 64
                    case 3:
#endif
                    {
                        funct3 = ins->binary >> 27;
                        switch (funct3)
                        {
                            case 2: /* lr.w */
                            {
                                if (ins->rs2 != 0)
                                    goto exception;
                                ins->has_src1 = TRUE;
                                ins->is_atomic_load = TRUE;
                                ins->bytes_to_rw = sizeof(target_ulong);
                                break;
                            }
                            case 3: /* sc.w */
                            {
                                ins->has_src1 = TRUE;
                                ins->has_src2 = TRUE;
                                ins->is_atomic_store = TRUE;
                                ins->bytes_to_rw = sizeof(target_ulong);
                                break;
                            }
                            case 1:    /* amiswap.w */
                            case 0:    /* amoadd.w */
                            case 4:    /* amoxor.w */
                            case 0xc:  /* amoand.w */
                            case 0x8:  /* amoor.w */
                            case 0x10: /* amomin.w */
                            case 0x14: /* amomax.w */
                            case 0x18: /* amominu.w */
                            case 0x1c: /* amomaxu.w */
                            {
                                ins->has_src1 = TRUE;
                                ins->has_src2 = TRUE;
                                ins->is_atomic_operate = TRUE;
                                ins->is_atomic_load = TRUE;
                                ins->is_atomic_store = TRUE;
                                ins->bytes_to_rw = sizeof(target_ulong);
                                break;
                            }
                            default:
                                goto exception;
                        }
                        break;
                    }
                    default:
                        goto exception;
                }
                break;
            }
            case FLOAD_MASK:
            {
                if (ins->current_fs == 0)
                {
                    goto exception;
                }
                ins->is_load = TRUE;
                ins->has_fp_dest = TRUE;
                ins->has_src1 = TRUE;
                ins->set_fs = TRUE;
                ins->imm = (int32_t)insn >> 20;
                ins->type = INS_TYPE_FP_LOAD;
                switch (ins->funct3)
                {
                    case 2: /* flw */
                    {
                        ins->bytes_to_rw = 4;
                        ins->f32_mask = TRUE;
                    }
                    break;
#if SIM_FLEN >= 64
                    case 3: /* fld */
                    {
                        ins->bytes_to_rw = 8;
                        ins->f64_mask = TRUE;
                    }
                    break;
#endif
                }
                break;
            }
            case FSTORE_MASK:
            {
                if (ins->current_fs == 0)
                {
                    goto exception;
                }
                ins->is_store = TRUE;
                ins->has_src1 = TRUE;
                ins->has_fp_src2 = TRUE;
                ins->imm = ins->rd | ((insn >> (25 - 5)) & 0xfe0);
                ins->imm = (ins->imm << 20) >> 20;
                ins->type = INS_TYPE_FP_STORE;
                switch (ins->funct3)
                {
                    case 2: /* fsw */
                        ins->bytes_to_rw = 4;
                        break;
#if SIM_FLEN >= 64
                    case 3: /* fsd */
#endif
                        ins->bytes_to_rw = 8;
                        break;
                }
                break;
            }
            case FMADD_MASK:
            {
                if ((ins->current_fs == 0) || (ins->rm < 0))
                {
                    goto exception;
                }
                ins->data_class = INS_CLASS_FP;
                ins->fu_type = FU_FPU_FMA;
                ins->type = INS_TYPE_FP_FMA;
                ins->has_fp_dest = TRUE;
                ins->has_fp_src1 = TRUE;
                ins->has_fp_src2 = TRUE;
                ins->has_fp_src3 = TRUE;
                ins->set_fs = TRUE;
                ins->funct3 = (ins->binary >> 25) & 3;
                ins->rs3 = ins->binary >> 27;
                break;
            }
            case FMSUB_MASK:
            {
                if ((ins->current_fs == 0) || (ins->rm < 0))
                {
                    goto exception;
                }
                ins->data_class = INS_CLASS_FP;
                ins->fu_type = FU_FPU_FMA;
                ins->type = INS_TYPE_FP_FMA;
                ins->has_fp_dest = TRUE;
                ins->has_fp_src1 = TRUE;
                ins->has_fp_src2 = TRUE;
                ins->has_fp_src3 = TRUE;
                ins->set_fs = TRUE;
                ins->funct3 = (ins->binary >> 25) & 3;
                ins->rs3 = ins->binary >> 27;
                break;
            }
            case FNMSUB_MASK:
            {
                if ((ins->current_fs == 0) || (ins->rm < 0))
                {
                    goto exception;
                }
                ins->data_class = INS_CLASS_FP;
                ins->fu_type = FU_FPU_FMA;
                ins->type = INS_TYPE_FP_FMA;
                ins->has_fp_dest = TRUE;
                ins->has_fp_src1 = TRUE;
                ins->has_fp_src2 = TRUE;
                ins->has_fp_src3 = TRUE;
                ins->set_fs = TRUE;
                ins->funct3 = (ins->binary >> 25) & 3;
                ins->rs3 = ins->binary >> 27;
                break;
            }
            case FNMADD_MASK:
            {
                if ((ins->current_fs == 0) || (ins->rm < 0))
                {
                    goto exception;
                }
                ins->data_class = INS_CLASS_FP;
                ins->fu_type = FU_FPU_FMA;
                ins->type = INS_TYPE_FP_FMA;
                ins->has_fp_dest = TRUE;
                ins->has_fp_src1 = TRUE;
                ins->has_fp_src2 = TRUE;
                ins->has_fp_src3 = TRUE;
                ins->set_fs = TRUE;
                ins->funct3 = (ins->binary >> 25) & 3;
                ins->rs3 = ins->binary >> 27;
                break;
            }
            case F_ARITHMETIC_MASK:
            {
                if (ins->current_fs == 0)
                {
                    goto exception;
                }
                ins->data_class = INS_CLASS_FP;
                ins->fu_type = FU_FPU_ALU;
                ins->rm = ins->funct3;
                ins->type = INS_TYPE_FP_MISC;
                switch (ins->funct7)
                {
#define F_SIZE 32
#include "decode_fpa_template.h"
#if SIM_FLEN >= 64
#define F_SIZE 64
#include "decode_fpa_template.h"
#endif
                    default:
                        goto exception;
                }
                break;
            }
            default:
            {
                goto exception;
            }
        }
    }

    /* Generate instruction string */
    if (ins->create_str)
    {
        get_riscv_ins_str(ins);
    }
    return;
exception:
    ins->exception = TRUE;
    ins->exception_cause = SIM_ILLEGAL_OPCODE;
}