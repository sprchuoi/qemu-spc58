/*
 * PowerPC VLE (Variable Length Encoding) translation helpers.
 *
 * This decoder covers the e200-focused VLE subset needed by the current
 * firmware bring-up.  It reuses translate.c helpers wherever possible so the
 * generated TCG matches the normal PPC translator behaviour for CR updates,
 * branches, memory accesses, and privileged return instructions.
 */

static bool decode_legacy(PowerPCCPU *cpu, DisasContext *ctx, uint32_t insn);

static inline int vle_se_reg(int nibble)
{
    return nibble < 8 ? nibble : nibble + 16;
}

static inline int vle_arf_reg(int nibble)
{
    return nibble + 8;
}

static inline bool vle_is_e_form(uint8_t byte0)
{
    switch (byte0 >> 2) {
    case 6:
    case 7:
    case 12:
    case 13:
    case 14:
    case 20:
    case 21:
    case 22:
    case 23:
    case 28:
    case 29:
    case 30:
    case 31:
        return true;
    default:
        return false;
    }
}

static inline uint32_t vle_mask32(int mb, int me)
{
    if (mb <= me) {
        return (0xffffffffu >> mb) & ~(0xffffffffu >> (me + 1));
    }
    return (0xffffffffu >> mb) | ~(0xffffffffu >> (me + 1));
}

static inline target_ulong vle_bit_mask(int bit)
{
    return (target_ulong)(uint32_t)(0x80000000u >> bit);
}

static inline int32_t vle_d16(uint32_t insn)
{
    return (int32_t)(int16_t)(insn & 0xffff);
}

static inline int32_t vle_d8(uint32_t insn)
{
    return (int32_t)(int8_t)(insn & 0xff);
}

static inline uint32_t vle_ui5(uint16_t insn)
{
    return (insn >> 4) & 0x1f;
}

static inline uint32_t vle_oimm(uint16_t insn)
{
    return vle_ui5(insn) + 1;
}

static inline uint32_t vle_e_i16l(uint32_t insn)
{
    return (((insn >> 16) & 0x1f) << 11) | (insn & 0x7ff);
}

static inline int32_t vle_e_i16a(uint32_t insn)
{
    uint32_t v = (((insn >> 21) & 0x1f) << 11) | (insn & 0x7ff);
    if (v & 0x8000) {
        v |= 0xffff0000u;
    }
    return (int32_t)v;
}

static inline int32_t vle_e_li20(uint32_t insn)
{
    uint32_t v = ((insn & 0x001f0000) >> 5)
               | ((insn & 0x00007800) << 5)
               | (insn & 0x000007ff);
    if (v & 0x00080000) {
        v |= 0xfff00000u;
    }
    return (int32_t)v;
}

static inline int32_t vle_e_bd24(uint32_t insn)
{
    uint32_t v = insn & 0x01fffffe;
    if (v & 0x01000000) {
        v |= 0xfe000000u;
    }
    return (int32_t)v;
}

static inline int32_t vle_e_bd15(uint32_t insn)
{
    uint32_t v = insn & 0x0000fffe;
    if (v & 0x00008000) {
        v |= 0xffff0000u;
    }
    return (int32_t)v;
}

static inline uint32_t vle_e_sci8(uint32_t insn)
{
    uint32_t ui8 = insn & 0xff;
    uint32_t scl = (insn >> 8) & 3;
    uint32_t f = (insn >> 10) & 1;

    switch (scl) {
    case 0:
        return ui8 | (f ? 0xffffff00u : 0);
    case 1:
        return (ui8 << 8) | (f ? 0xffff00ffu : 0);
    case 2:
        return (ui8 << 16) | (f ? 0xff00ffffu : 0);
    default:
        return (ui8 << 24) | (f ? 0x00ffffffu : 0);
    }
}

static inline TCGv vle_ea_disp(DisasContext *ctx, int ra, int32_t disp)
{
    return do_ea_calc(ctx, ra, tcg_constant_tl((target_long)disp));
}

static inline void vle_set_ra_from_ea(DisasContext *ctx, int ra, TCGv ea)
{
    if (ra != 0) {
        tcg_gen_mov_tl(cpu_gpr[ra], ea);
    }
}

static inline void vle_load(DisasContext *ctx, int rt, int ra, int32_t disp,
                            MemOp mop)
{
    TCGv ea = vle_ea_disp(ctx, ra, disp);

    gen_set_access_type(ctx, ACCESS_INT);
    switch (mop) {
    case MO_UB:
        gen_qemu_ld8u(ctx, cpu_gpr[rt], ea);
        break;
    case MO_UW:
        gen_qemu_ld16u(ctx, cpu_gpr[rt], ea);
        break;
    case MO_SW:
        gen_qemu_ld16s(ctx, cpu_gpr[rt], ea);
        break;
    case MO_UL:
        gen_qemu_ld32u(ctx, cpu_gpr[rt], ea);
        break;
    default:
        g_assert_not_reached();
    }
}

static inline void vle_store(DisasContext *ctx, int rs, int ra, int32_t disp,
                             MemOp mop)
{
    TCGv ea = vle_ea_disp(ctx, ra, disp);

    gen_set_access_type(ctx, ACCESS_INT);
    switch (mop) {
    case MO_UB:
        gen_qemu_st8(ctx, cpu_gpr[rs], ea);
        break;
    case MO_UW:
        gen_qemu_st16(ctx, cpu_gpr[rs], ea);
        break;
    case MO_UL:
        gen_qemu_st32(ctx, cpu_gpr[rs], ea);
        break;
    default:
        g_assert_not_reached();
    }
}

static inline void vle_load_update(DisasContext *ctx, int rt, int ra,
                                   int32_t disp, MemOp mop)
{
    TCGv ea;
    TCGv tmp = tcg_temp_new();

    if (ra == 0) {
        gen_inval_exception(ctx, POWERPC_EXCP_INVAL_INVAL);
        return;
    }

    ea = vle_ea_disp(ctx, ra, disp);
    gen_set_access_type(ctx, ACCESS_INT);
    switch (mop) {
    case MO_UB:
        gen_qemu_ld8u(ctx, tmp, ea);
        break;
    case MO_UW:
        gen_qemu_ld16u(ctx, tmp, ea);
        break;
    case MO_SW:
        gen_qemu_ld16s(ctx, tmp, ea);
        break;
    case MO_UL:
        gen_qemu_ld32u(ctx, tmp, ea);
        break;
    default:
        g_assert_not_reached();
    }
    vle_set_ra_from_ea(ctx, ra, ea);
    tcg_gen_mov_tl(cpu_gpr[rt], tmp);
}

static inline void vle_store_update(DisasContext *ctx, int rs, int ra,
                                    int32_t disp, MemOp mop)
{
    TCGv ea;

    if (ra == 0) {
        gen_inval_exception(ctx, POWERPC_EXCP_INVAL_INVAL);
        return;
    }

    ea = vle_ea_disp(ctx, ra, disp);
    gen_set_access_type(ctx, ACCESS_INT);
    switch (mop) {
    case MO_UB:
        gen_qemu_st8(ctx, cpu_gpr[rs], ea);
        break;
    case MO_UW:
        gen_qemu_st16(ctx, cpu_gpr[rs], ea);
        break;
    case MO_UL:
        gen_qemu_st32(ctx, cpu_gpr[rs], ea);
        break;
    default:
        g_assert_not_reached();
    }
    vle_set_ra_from_ea(ctx, ra, ea);
}

static inline void vle_branch_direct(DisasContext *ctx, target_ulong target,
                                     bool link)
{
    if (link) {
        gen_setlr(ctx, ctx->base.pc_next);
    }
    gen_update_cfar(ctx, ctx->cia);
    gen_goto_tb(ctx, 0, target);
    ctx->base.is_jmp = DISAS_NORETURN;
}

static inline void vle_branch_indirect(DisasContext *ctx, TCGv target,
                                       bool link)
{
    if (link) {
        gen_setlr(ctx, ctx->base.pc_next);
    }
    gen_update_cfar(ctx, ctx->cia);
    if (NARROW_MODE(ctx)) {
        tcg_gen_andi_tl(cpu_nip, target, (uint32_t)~1u);
    } else {
        tcg_gen_andi_tl(cpu_nip, target, (target_ulong)~1ull);
    }
    gen_lookup_and_goto_ptr(ctx);
    ctx->base.is_jmp = DISAS_NORETURN;
}

static inline bool vle_branch_cond16(DisasContext *ctx, uint16_t insn)
{
    int cond = (insn >> 8) & 0xf;
    int32_t disp = (int8_t)(insn & 0xff);
    target_ulong target = ctx->cia + (disp << 1);
    TCGLabel *l_not_taken = gen_new_label();
    TCGv_i32 t = tcg_temp_new_i32();
    uint32_t mask;
    bool want_set;

    switch (cond) {
    case 0x0: /* bge */
        mask = CRF_LT;
        want_set = false;
        break;
    case 0x1: /* ble */
        mask = CRF_GT;
        want_set = false;
        break;
    case 0x2: /* bne */
        mask = CRF_EQ;
        want_set = false;
        break;
    case 0x3: /* bns */
        mask = CRF_SO;
        want_set = false;
        break;
    case 0x4: /* blt */
        mask = CRF_LT;
        want_set = true;
        break;
    case 0x5: /* bgt */
        mask = CRF_GT;
        want_set = true;
        break;
    case 0x6: /* beq */
        mask = CRF_EQ;
        want_set = true;
        break;
    case 0x7: /* bso */
        mask = CRF_SO;
        want_set = true;
        break;
    default:
        return false;
    }

    tcg_gen_andi_i32(t, cpu_crf[0], mask);
    tcg_gen_brcondi_i32(want_set ? TCG_COND_EQ : TCG_COND_NE, t, 0, l_not_taken);
    gen_update_cfar(ctx, ctx->cia);
    gen_goto_tb(ctx, 0, target);
    gen_set_label(l_not_taken);
    gen_goto_tb(ctx, 1, ctx->base.pc_next);
    ctx->base.is_jmp = DISAS_NORETURN;
    return true;
}

static inline bool vle_branch_cond32(DisasContext *ctx, uint32_t insn)
{
    int32_t disp = vle_e_bd15(insn);
    target_ulong target = ctx->cia + disp;
    bool link = insn & 1;
    bool bit_l = (insn >> 15) & 1;
    int bo = (insn >> 20) & 0x3;

    if (bo >= 2) {
        TCGLabel *l_not_taken = gen_new_label();
        TCGv t = tcg_temp_new();

        if (link) {
            gen_setlr(ctx, ctx->base.pc_next);
        }
        tcg_gen_subi_tl(cpu_ctr, cpu_ctr, 1);
        if (NARROW_MODE(ctx)) {
            tcg_gen_ext32u_tl(t, cpu_ctr);
        } else {
            tcg_gen_mov_tl(t, cpu_ctr);
        }
        tcg_gen_brcondi_tl(bo == 2 ? TCG_COND_EQ : TCG_COND_NE, t, 0,
                           l_not_taken);
        gen_update_cfar(ctx, ctx->cia);
        gen_goto_tb(ctx, 0, target);
        gen_set_label(l_not_taken);
        gen_goto_tb(ctx, 1, ctx->base.pc_next);
        ctx->base.is_jmp = DISAS_NORETURN;
        return true;
    }

    {
        int crf = bit_l ? 0 : ((insn >> 18) & 0x7);
        int cc = (insn >> 16) & 0x3;
        bool want_set = bo & 1;
        uint32_t mask = 0x8 >> cc;
        TCGLabel *l_not_taken = gen_new_label();
        TCGv_i32 t = tcg_temp_new_i32();

        if (link) {
            gen_setlr(ctx, ctx->base.pc_next);
        }
        tcg_gen_andi_i32(t, cpu_crf[crf], mask);
        tcg_gen_brcondi_i32(want_set ? TCG_COND_EQ : TCG_COND_NE, t, 0,
                            l_not_taken);
        gen_update_cfar(ctx, ctx->cia);
        gen_goto_tb(ctx, 0, target);
        gen_set_label(l_not_taken);
        gen_goto_tb(ctx, 1, ctx->base.pc_next);
        ctx->base.is_jmp = DISAS_NORETURN;
        return true;
    }
}

static inline void vle_pack_cr(TCGv dst)
{
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_mov_i32(t, cpu_crf[0]);
    tcg_gen_shli_i32(t, t, 4);
    tcg_gen_or_i32(t, t, cpu_crf[1]);
    tcg_gen_shli_i32(t, t, 4);
    tcg_gen_or_i32(t, t, cpu_crf[2]);
    tcg_gen_shli_i32(t, t, 4);
    tcg_gen_or_i32(t, t, cpu_crf[3]);
    tcg_gen_shli_i32(t, t, 4);
    tcg_gen_or_i32(t, t, cpu_crf[4]);
    tcg_gen_shli_i32(t, t, 4);
    tcg_gen_or_i32(t, t, cpu_crf[5]);
    tcg_gen_shli_i32(t, t, 4);
    tcg_gen_or_i32(t, t, cpu_crf[6]);
    tcg_gen_shli_i32(t, t, 4);
    tcg_gen_or_i32(t, t, cpu_crf[7]);
    tcg_gen_extu_i32_tl(dst, t);
}

static inline void vle_unpack_cr(TCGv src)
{
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_trunc_tl_i32(t, src);
    for (int i = 0; i < 8; ++i) {
        tcg_gen_shri_i32(cpu_crf[i], t, (7 - i) * 4);
        tcg_gen_andi_i32(cpu_crf[i], cpu_crf[i], 0xf);
    }
}

static inline void vle_pack_xer(DisasContext *ctx, TCGv dst)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGv t2 = tcg_temp_new();

    tcg_gen_mov_tl(dst, cpu_xer);
    tcg_gen_shli_tl(t0, cpu_so, XER_SO);
    tcg_gen_shli_tl(t1, cpu_ov, XER_OV);
    tcg_gen_shli_tl(t2, cpu_ca, XER_CA);
    tcg_gen_or_tl(t0, t0, t1);
    tcg_gen_or_tl(dst, dst, t2);
    tcg_gen_or_tl(dst, dst, t0);
    if (is_isa300(ctx)) {
        tcg_gen_shli_tl(t0, cpu_ov32, XER_OV32);
        tcg_gen_or_tl(dst, dst, t0);
        tcg_gen_shli_tl(t0, cpu_ca32, XER_CA32);
        tcg_gen_or_tl(dst, dst, t0);
    }
}

static inline void vle_unpack_xer(TCGv src)
{
    tcg_gen_andi_tl(cpu_xer, src,
                    ~((1u << XER_SO) |
                      (1u << XER_OV) | (1u << XER_OV32) |
                      (1u << XER_CA) | (1u << XER_CA32)));
    tcg_gen_extract_tl(cpu_ov32, src, XER_OV32, 1);
    tcg_gen_extract_tl(cpu_ca32, src, XER_CA32, 1);
    tcg_gen_extract_tl(cpu_so, src, XER_SO, 1);
    tcg_gen_extract_tl(cpu_ov, src, XER_OV, 1);
    tcg_gen_extract_tl(cpu_ca, src, XER_CA, 1);
}

static bool vle_decode_se(DisasContext *ctx, uint16_t insn)
{
    int hi = (insn >> 8) & 0xff;
    int lo = insn & 0xff;
    int rD, rA, rB;

    switch (insn) {
    case 0x0000: /* se_illegal */
        gen_inval_exception(ctx, POWERPC_EXCP_INVAL_INVAL);
        return true;
    case 0x0004: /* se_blr */
    {
        TCGv target = tcg_temp_new();
        tcg_gen_mov_tl(target, cpu_lr);
        vle_branch_indirect(ctx, target, false);
        return true;
    }
    case 0x0005: /* se_blrl */
    {
        TCGv target = tcg_temp_new();
        tcg_gen_mov_tl(target, cpu_lr);
        vle_branch_indirect(ctx, target, true);
        return true;
    }
    case 0x0006: /* se_bctr */
    {
        TCGv target = tcg_temp_new();
        tcg_gen_mov_tl(target, cpu_ctr);
        vle_branch_indirect(ctx, target, false);
        return true;
    }
    case 0x0007: /* se_bctrl */
    {
        TCGv target = tcg_temp_new();
        tcg_gen_mov_tl(target, cpu_ctr);
        vle_branch_indirect(ctx, target, true);
        return true;
    }
    case 0x0010: /* se_rfi */
#if defined(CONFIG_USER_ONLY)
        gen_priv_opc(ctx);
        return true;
#else
        if (is_book3s_arch2x(ctx)) {
            gen_inval_exception(ctx, POWERPC_EXCP_INVAL_INVAL);
            return true;
        }
        if (unlikely(ctx->pr)) {
            gen_priv_opc(ctx);
            return true;
        }
        translator_io_start(&ctx->base);
        gen_update_cfar(ctx, ctx->cia);
        gen_helper_rfi(tcg_env);
        ctx->base.is_jmp = DISAS_EXIT;
#endif
        return true;
    case 0x0016: /* se_rfmci */
#if defined(CONFIG_USER_ONLY)
        gen_priv_opc(ctx);
        return true;
#else
        if (unlikely(ctx->pr)) {
            gen_priv_opc(ctx);
            return true;
        }
        gen_helper_rfmci(tcg_env);
        ctx->base.is_jmp = DISAS_EXIT;
#endif
        return true;
    default:
        break;
    }

    if ((insn & 0xfff0) == 0x0020) { /* se_not */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_not_tl(cpu_gpr[rD], cpu_gpr[rD]);
        tcg_gen_ext32u_tl(cpu_gpr[rD], cpu_gpr[rD]);
        return true;
    }
    if ((insn & 0xfff0) == 0x0030) { /* se_neg */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_neg_tl(cpu_gpr[rD], cpu_gpr[rD]);
        return true;
    }
    if ((insn & 0xfff0) == 0x0080) { /* se_mflr */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_mov_tl(cpu_gpr[rD], cpu_lr);
        return true;
    }
    if ((insn & 0xfff0) == 0x0090) { /* se_mtlr */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_mov_tl(cpu_lr, cpu_gpr[rD]);
        return true;
    }
    if ((insn & 0xfff0) == 0x00a0) { /* se_mfctr */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_mov_tl(cpu_gpr[rD], cpu_ctr);
        return true;
    }
    if ((insn & 0xfff0) == 0x00b0) { /* se_mtctr */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_mov_tl(cpu_ctr, cpu_gpr[rD]);
        return true;
    }
    if ((insn & 0xfff0) == 0x00c0) { /* se_extzb */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_andi_tl(cpu_gpr[rD], cpu_gpr[rD], 0xff);
        return true;
    }
    if ((insn & 0xfff0) == 0x00d0) { /* se_extsb */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_ext8s_tl(cpu_gpr[rD], cpu_gpr[rD]);
        return true;
    }
    if ((insn & 0xfff0) == 0x00e0) { /* se_extzh */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_andi_tl(cpu_gpr[rD], cpu_gpr[rD], 0xffff);
        return true;
    }
    if ((insn & 0xfff0) == 0x00f0) { /* se_extsh */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_ext16s_tl(cpu_gpr[rD], cpu_gpr[rD]);
        return true;
    }

    if (hi == 0x01) { /* se_mr */
        rD = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        tcg_gen_mov_tl(cpu_gpr[rD], cpu_gpr[rB]);
        return true;
    }
    if (hi == 0x02) { /* se_mtar */
        int arf = vle_arf_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        tcg_gen_mov_tl(cpu_gprh[arf - 8], cpu_gpr[rB]);
        return true;
    }
    if (hi == 0x03) { /* se_mfar */
        rD = vle_se_reg((lo >> 4) & 0xf);
        {
            int arf = vle_arf_reg(lo & 0xf);
            tcg_gen_mov_tl(cpu_gpr[rD], cpu_gprh[arf - 8]);
        }
        return true;
    }
    if (hi == 0x04) { /* se_add */
        rD = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        gen_op_arith_add(ctx, cpu_gpr[rD], cpu_gpr[rD], cpu_gpr[rB],
                         cpu_ca, cpu_ca32, false, false, false, false);
        return true;
    }
    if (hi == 0x05) { /* se_mullw */
        rD = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
#if defined(TARGET_PPC64)
        {
            TCGv_i64 t0 = tcg_temp_new_i64();
            TCGv_i64 t1 = tcg_temp_new_i64();
            tcg_gen_ext32s_tl(t0, cpu_gpr[rD]);
            tcg_gen_ext32s_tl(t1, cpu_gpr[rB]);
            tcg_gen_mul_i64(cpu_gpr[rD], t0, t1);
        }
#else
        tcg_gen_mul_tl(cpu_gpr[rD], cpu_gpr[rD], cpu_gpr[rB]);
#endif
        return true;
    }
    if (hi == 0x06) { /* se_sub */
        rD = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        gen_op_arith_subf(ctx, cpu_gpr[rD], cpu_gpr[rB], cpu_gpr[rD],
                          false, false, false, false);
        return true;
    }
    if (hi == 0x07) { /* se_subf */
        rD = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        gen_op_arith_subf(ctx, cpu_gpr[rD], cpu_gpr[rD], cpu_gpr[rB],
                          false, false, false, false);
        return true;
    }
    if (hi == 0x0c) { /* se_cmp */
        rA = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        gen_op_cmp32(cpu_gpr[rA], cpu_gpr[rB], 1, 0);
        return true;
    }
    if (hi == 0x0d) { /* se_cmpl */
        rA = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        gen_op_cmp32(cpu_gpr[rA], cpu_gpr[rB], 0, 0);
        return true;
    }

    if ((insn & 0xfe00) == 0x2000) { /* se_addi */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_addi_tl(cpu_gpr[rD], cpu_gpr[rD], vle_oimm(insn));
        return true;
    }
    if ((insn & 0xfe00) == 0x2200) { /* se_cmpli */
        rD = vle_se_reg(insn & 0xf);
        gen_op_cmpi32(cpu_gpr[rD], vle_oimm(insn), 0, 0);
        return true;
    }
    if ((insn & 0xfe00) == 0x2400) { /* se_subi */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_subi_tl(cpu_gpr[rD], cpu_gpr[rD], vle_oimm(insn));
        return true;
    }
    if ((insn & 0xfe00) == 0x2600) { /* se_subi. */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_subi_tl(cpu_gpr[rD], cpu_gpr[rD], vle_oimm(insn));
        gen_set_Rc0(ctx, cpu_gpr[rD]);
        return true;
    }
    if ((insn & 0xfe00) == 0x2a00) { /* se_cmpi */
        rD = vle_se_reg(insn & 0xf);
        gen_op_cmpi32(cpu_gpr[rD], vle_ui5(insn), 1, 0);
        return true;
    }
    if ((insn & 0xfe00) == 0x2c00) { /* se_bmaski */
        uint32_t ui5 = vle_ui5(insn);
        uint32_t mask = ui5 ? (0xffffffffu >> (32 - ui5)) : 0xffffffffu;
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_movi_tl(cpu_gpr[rD], mask);
        return true;
    }
    if ((insn & 0xfe00) == 0x2e00) { /* se_andi */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_andi_tl(cpu_gpr[rD], cpu_gpr[rD], vle_ui5(insn));
        return true;
    }

    if (hi == 0x40) { /* se_srw */
        rD = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        {
            TCGv sh = tcg_temp_new();
            TCGv src = tcg_temp_new();
            tcg_gen_andi_tl(sh, cpu_gpr[rB], 0x1f);
            tcg_gen_ext32u_tl(src, cpu_gpr[rD]);
            tcg_gen_shr_tl(cpu_gpr[rD], src, sh);
            tcg_gen_ext32u_tl(cpu_gpr[rD], cpu_gpr[rD]);
        }
        return true;
    }
    if (hi == 0x41) { /* se_sraw */
        rD = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        gen_helper_sraw(cpu_gpr[rD], tcg_env, cpu_gpr[rD], cpu_gpr[rB]);
        return true;
    }
    if (hi == 0x42) { /* se_slw */
        rD = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        {
            TCGv sh = tcg_temp_new();
            TCGv src = tcg_temp_new();
            tcg_gen_andi_tl(sh, cpu_gpr[rB], 0x1f);
            tcg_gen_ext32u_tl(src, cpu_gpr[rD]);
            tcg_gen_shl_tl(cpu_gpr[rD], src, sh);
            tcg_gen_ext32u_tl(cpu_gpr[rD], cpu_gpr[rD]);
        }
        return true;
    }
    if (hi == 0x44) { /* se_nop / se_or */
        rD = vle_se_reg(lo & 0xf);
        rB = vle_se_reg((lo >> 4) & 0xf);
        if (rD == 0 && rB == 0) {
            return true;
        }
        tcg_gen_or_tl(cpu_gpr[rD], cpu_gpr[rD], cpu_gpr[rB]);
        return true;
    }
    if (hi == 0x45) { /* se_andc */
        rD = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        tcg_gen_andc_tl(cpu_gpr[rD], cpu_gpr[rD], cpu_gpr[rB]);
        return true;
    }
    if (hi == 0x46) { /* se_and */
        rD = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        tcg_gen_and_tl(cpu_gpr[rD], cpu_gpr[rD], cpu_gpr[rB]);
        return true;
    }
    if (hi == 0x47) { /* se_and. */
        rD = vle_se_reg((lo >> 4) & 0xf);
        rB = vle_se_reg(lo & 0xf);
        tcg_gen_and_tl(cpu_gpr[rD], cpu_gpr[rD], cpu_gpr[rB]);
        gen_set_Rc0(ctx, cpu_gpr[rD]);
        return true;
    }

    if ((insn & 0xf800) == 0x4800) { /* se_li */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_movi_tl(cpu_gpr[rD], (insn >> 4) & 0x7f);
        return true;
    }

    if ((insn & 0xfe00) == 0x6000) { /* se_bclri */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_andi_tl(cpu_gpr[rD], cpu_gpr[rD], ~vle_bit_mask(vle_ui5(insn)));
        return true;
    }
    if ((insn & 0xfe00) == 0x6200) { /* se_bgeni */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_movi_tl(cpu_gpr[rD], vle_bit_mask(vle_ui5(insn)));
        return true;
    }
    if ((insn & 0xfe00) == 0x6400) { /* se_bseti */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_ori_tl(cpu_gpr[rD], cpu_gpr[rD], vle_bit_mask(vle_ui5(insn)));
        return true;
    }
    if ((insn & 0xfe00) == 0x6600) { /* se_btsti */
        TCGv t = tcg_temp_new();
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_andi_tl(t, cpu_gpr[rD], vle_bit_mask(vle_ui5(insn)));
        gen_op_cmpi32(t, 0, 1, 0);
        return true;
    }
    if ((insn & 0xfe00) == 0x6800) { /* se_srwi */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_ext32u_tl(cpu_gpr[rD], cpu_gpr[rD]);
        tcg_gen_shri_tl(cpu_gpr[rD], cpu_gpr[rD], vle_ui5(insn));
        tcg_gen_ext32u_tl(cpu_gpr[rD], cpu_gpr[rD]);
        return true;
    }
    if ((insn & 0xfe00) == 0x6a00) { /* se_srawi */
        int sh = vle_ui5(insn);
        TCGv dst;
        TCGv src;
        rD = vle_se_reg(insn & 0xf);
        dst = cpu_gpr[rD];
        src = cpu_gpr[rD];
        if (sh == 0) {
            tcg_gen_ext32s_tl(dst, src);
            tcg_gen_movi_tl(cpu_ca, 0);
            if (is_isa300(ctx)) {
                tcg_gen_movi_tl(cpu_ca32, 0);
            }
        } else {
            TCGv t0 = tcg_temp_new();
            tcg_gen_ext32s_tl(dst, src);
            tcg_gen_andi_tl(cpu_ca, dst, (1ull << sh) - 1);
            tcg_gen_sari_tl(t0, dst, TARGET_LONG_BITS - 1);
            tcg_gen_and_tl(cpu_ca, cpu_ca, t0);
            tcg_gen_setcondi_tl(TCG_COND_NE, cpu_ca, cpu_ca, 0);
            if (is_isa300(ctx)) {
                tcg_gen_mov_tl(cpu_ca32, cpu_ca);
            }
            tcg_gen_sari_tl(dst, dst, sh);
        }
        return true;
    }
    if ((insn & 0xfe00) == 0x6c00) { /* se_slwi */
        rD = vle_se_reg(insn & 0xf);
        tcg_gen_ext32u_tl(cpu_gpr[rD], cpu_gpr[rD]);
        tcg_gen_shli_tl(cpu_gpr[rD], cpu_gpr[rD], vle_ui5(insn));
        tcg_gen_ext32u_tl(cpu_gpr[rD], cpu_gpr[rD]);
        return true;
    }

    if ((insn & 0xf000) == 0x8000) { /* se_lbz */
        int sd4 = (insn >> 8) & 0xf;
        rD = vle_se_reg((insn >> 4) & 0xf);
        rA = vle_se_reg(insn & 0xf);
        vle_load(ctx, rD, rA, sd4, MO_UB);
        return true;
    }
    if ((insn & 0xf000) == 0x9000) { /* se_stb */
        int sd4 = (insn >> 8) & 0xf;
        rD = vle_se_reg((insn >> 4) & 0xf);
        rA = vle_se_reg(insn & 0xf);
        vle_store(ctx, rD, rA, sd4, MO_UB);
        return true;
    }
    if ((insn & 0xf000) == 0xa000) { /* se_lhz */
        int sd4 = ((insn >> 8) & 0xf) << 1;
        rD = vle_se_reg((insn >> 4) & 0xf);
        rA = vle_se_reg(insn & 0xf);
        vle_load(ctx, rD, rA, sd4, MO_UW);
        return true;
    }
    if ((insn & 0xf000) == 0xb000) { /* se_sth */
        int sd4 = ((insn >> 8) & 0xf) << 1;
        rD = vle_se_reg((insn >> 4) & 0xf);
        rA = vle_se_reg(insn & 0xf);
        vle_store(ctx, rD, rA, sd4, MO_UW);
        return true;
    }
    if ((insn & 0xf000) == 0xc000) { /* se_lwz */
        int sd4 = ((insn >> 8) & 0xf) << 2;
        rD = vle_se_reg((insn >> 4) & 0xf);
        rA = vle_se_reg(insn & 0xf);
        vle_load(ctx, rD, rA, sd4, MO_UL);
        return true;
    }
    if ((insn & 0xf000) == 0xd000) { /* se_stw */
        int sd4 = ((insn >> 8) & 0xf) << 2;
        rD = vle_se_reg((insn >> 4) & 0xf);
        rA = vle_se_reg(insn & 0xf);
        vle_store(ctx, rD, rA, sd4, MO_UL);
        return true;
    }

    if ((insn & 0xf000) == 0xe000) {
        if (hi == 0xe8) { /* se_b */
            vle_branch_direct(ctx, ctx->cia + (((int8_t)(insn & 0xff)) << 1), false);
            return true;
        }
        if (hi == 0xe9) { /* se_bl */
            vle_branch_direct(ctx, ctx->cia + (((int8_t)(insn & 0xff)) << 1), true);
            return true;
        }
        return vle_branch_cond16(ctx, insn);
    }

    return false;
}

static bool vle_decode_e_opc28(DisasContext *ctx, uint32_t insn)
{
    int rt = (insn >> 21) & 0x1f;
    int ra = (insn >> 16) & 0x1f;
    int subop = (insn >> 11) & 0x1f;

    if (((insn >> 15) & 1) == 0) {
        switch (subop) {
        case 17:
        case 18:
        case 19:
        case 20:
        case 22:
        case 24:
        case 25:
        case 26:
        case 28:
        case 29:
            break;
        default:
            tcg_gen_movi_tl(cpu_gpr[rt], (target_long)vle_e_li20(insn));
            return true;
        }
    }

    switch (subop) {
    case 17: /* e_add2i. */
        gen_op_arith_add(ctx, cpu_gpr[ra], cpu_gpr[ra],
                         tcg_constant_tl((target_long)vle_e_i16a(insn)),
                         cpu_ca, cpu_ca32, false, false, false, true);
        return true;
    case 18: /* e_add2is */
        gen_op_arith_add(ctx, cpu_gpr[ra], cpu_gpr[ra],
                         tcg_constant_tl((target_long)(vle_e_i16a(insn) << 16)),
                         cpu_ca, cpu_ca32, false, false, false, false);
        return true;
    case 19: /* e_cmp16i */
        gen_op_cmpi32(cpu_gpr[ra], (target_long)vle_e_i16a(insn), 1, 0);
        return true;
    case 21: /* e_cmpl16i */
        gen_op_cmpi32(cpu_gpr[ra], (uint16_t)vle_e_i16a(insn), 0, 0);
        return true;
    case 24: /* e_or2i */
        tcg_gen_ori_tl(cpu_gpr[rt], cpu_gpr[rt], vle_e_i16l(insn));
        return true;
    case 25: /* e_and2i. */
        tcg_gen_andi_tl(cpu_gpr[rt], cpu_gpr[rt], vle_e_i16l(insn));
        gen_set_Rc0(ctx, cpu_gpr[rt]);
        return true;
    case 26: /* e_or2is */
        tcg_gen_ori_tl(cpu_gpr[rt], cpu_gpr[rt], vle_e_i16l(insn) << 16);
        return true;
    case 28: /* e_lis */
        tcg_gen_movi_tl(cpu_gpr[rt], (target_long)(vle_e_i16l(insn) << 16));
        return true;
    case 29: /* e_and2is. */
        tcg_gen_andi_tl(cpu_gpr[rt], cpu_gpr[rt], vle_e_i16l(insn) << 16);
        gen_set_Rc0(ctx, cpu_gpr[rt]);
        return true;
    default:
        return false;
    }
}

static bool vle_decode_e_opc29(DisasContext *ctx, uint32_t insn)
{
    int rs = (insn >> 21) & 0x1f;
    int ra = (insn >> 16) & 0x1f;
    int sh = (insn >> 11) & 0x1f;
    int mb = (insn >> 6) & 0x1f;
    int me = (insn >> 1) & 0x1f;
    uint32_t mask = vle_mask32(mb, me);
    TCGv t = tcg_temp_new();

    tcg_gen_ext32u_tl(t, cpu_gpr[rs]);
    tcg_gen_rotli_tl(t, t, sh);
    if (insn & 1) { /* e_rlwinm */
        tcg_gen_andi_tl(cpu_gpr[ra], t, mask);
    } else { /* e_rlwimi */
        tcg_gen_andi_tl(t, t, mask);
        tcg_gen_andi_tl(cpu_gpr[ra], cpu_gpr[ra], ~((target_ulong)mask));
        tcg_gen_or_tl(cpu_gpr[ra], cpu_gpr[ra], t);
    }
    return true;
}

static bool vle_decode_e_opc31(DisasContext *ctx, PowerPCCPU *cpu, uint32_t insn)
{
    int xop = (insn >> 1) & 0x3ff;
    int rc = insn & 1;
    int rs = (insn >> 21) & 0x1f;
    int ra = (insn >> 16) & 0x1f;
    int sh = (insn >> 11) & 0x1f;

    switch (xop) {
    case 56: /* e_slwi / e_slwi. */
        tcg_gen_ext32u_tl(cpu_gpr[ra], cpu_gpr[rs]);
        tcg_gen_shli_tl(cpu_gpr[ra], cpu_gpr[ra], sh);
        tcg_gen_ext32u_tl(cpu_gpr[ra], cpu_gpr[ra]);
        if (rc) {
            gen_set_Rc0(ctx, cpu_gpr[ra]);
        }
        return true;
    case 568: /* e_srwi / e_srwi. */
        tcg_gen_ext32u_tl(cpu_gpr[ra], cpu_gpr[rs]);
        tcg_gen_shri_tl(cpu_gpr[ra], cpu_gpr[ra], sh);
        tcg_gen_ext32u_tl(cpu_gpr[ra], cpu_gpr[ra]);
        if (rc) {
            gen_set_Rc0(ctx, cpu_gpr[ra]);
        }
        return true;
    default:
        ctx->opcode = insn;
        return decode_insn32(ctx, insn) || decode_legacy(cpu, ctx, insn);
    }
}

static bool vle_decode_e_opc6(DisasContext *ctx, uint32_t insn)
{
    int rt = (insn >> 21) & 0x1f;
    int ra = (insn >> 16) & 0x1f;
    int xop8 = (insn >> 8) & 0xff;
    int xop11 = (insn >> 11) & 0x1f;
    uint32_t imm = vle_e_sci8(insn);
    int32_t disp = vle_d8(insn);

    switch (xop8) {
    case 0x00:
        vle_load_update(ctx, rt, ra, disp, MO_UB); /* e_lbzu */
        return true;
    case 0x01:
        vle_load_update(ctx, rt, ra, disp, MO_UW); /* e_lhzu */
        return true;
    case 0x02:
        vle_load_update(ctx, rt, ra, disp, MO_UL); /* e_lwzu */
        return true;
    case 0x03:
        vle_load_update(ctx, rt, ra, disp, MO_SW); /* e_lhau */
        return true;
    case 0x04:
        vle_store_update(ctx, rt, ra, disp, MO_UB); /* e_stbu */
        return true;
    case 0x05:
        vle_store_update(ctx, rt, ra, disp, MO_UW); /* e_sthu */
        return true;
    case 0x06:
        vle_store_update(ctx, rt, ra, disp, MO_UL); /* e_stwu */
        return true;
    case 0x08: /* e_lmw */
        for (int r = rt, off = disp; r < 32; ++r, off += 4) {
            vle_load(ctx, r, ra, off, MO_UL);
        }
        return true;
    case 0x09: /* e_stmw */
        for (int r = rt, off = disp; r < 32; ++r, off += 4) {
            vle_store(ctx, r, ra, off, MO_UL);
        }
        return true;
    case 0x10: /* e_lmvgprw / e_lmvsprw */
        switch (rt) {
        case 0: {
            TCGv ea = vle_ea_disp(ctx, ra, disp);
            gen_set_access_type(ctx, ACCESS_INT);
            for (int r = 0; r <= 12; ++r) {
                gen_qemu_ld32u(ctx, cpu_gpr[r], ea);
                if (r != 12) {
                    tcg_gen_addi_tl(ea, ea, 4);
                }
            }
            return true;
        }
        case 1: {
            TCGv ea = vle_ea_disp(ctx, ra, disp);
            TCGv tmp = tcg_temp_new();
            gen_set_access_type(ctx, ACCESS_INT);
            gen_qemu_ld32u(ctx, tmp, ea);
            vle_unpack_cr(tmp);
            tcg_gen_addi_tl(ea, ea, 4);
            gen_qemu_ld32u(ctx, cpu_lr, ea);
            tcg_gen_addi_tl(ea, ea, 4);
            gen_qemu_ld32u(ctx, cpu_ctr, ea);
            tcg_gen_addi_tl(ea, ea, 4);
            gen_qemu_ld32u(ctx, tmp, ea);
            vle_unpack_xer(tmp);
            return true;
        }
        default:
            return false;
        }
    case 0x11: /* e_stmvgprw / e_stmvsprw */
        switch (rt) {
        case 0: {
            TCGv ea = vle_ea_disp(ctx, ra, disp);
            gen_set_access_type(ctx, ACCESS_INT);
            for (int r = 0; r <= 12; ++r) {
                gen_qemu_st32(ctx, cpu_gpr[r], ea);
                if (r != 12) {
                    tcg_gen_addi_tl(ea, ea, 4);
                }
            }
            return true;
        }
        case 1: {
            TCGv ea = vle_ea_disp(ctx, ra, disp);
            TCGv tmp = tcg_temp_new();
            gen_set_access_type(ctx, ACCESS_INT);
            vle_pack_cr(tmp);
            gen_qemu_st32(ctx, tmp, ea);
            tcg_gen_addi_tl(ea, ea, 4);
            gen_qemu_st32(ctx, cpu_lr, ea);
            tcg_gen_addi_tl(ea, ea, 4);
            gen_qemu_st32(ctx, cpu_ctr, ea);
            tcg_gen_addi_tl(ea, ea, 4);
            vle_pack_xer(ctx, tmp);
            gen_qemu_st32(ctx, tmp, ea);
            return true;
        }
        default:
            return false;
        }
    default:
        break;
    }

    switch (xop11) {
    case 16: /* e_addi */
    case 17: /* e_addi. */
        gen_op_arith_add(ctx, cpu_gpr[rt], cpu_gpr[ra],
                         tcg_constant_tl((target_long)(int32_t)imm),
                         cpu_ca, cpu_ca32, false, false, false, xop11 == 17);
        return true;
    case 18: /* e_addic */
    case 19: /* e_addic. */
        gen_op_arith_add(ctx, cpu_gpr[rt], cpu_gpr[ra],
                         tcg_constant_tl((target_long)(int32_t)imm),
                         cpu_ca, cpu_ca32, false, true, false, xop11 == 19);
        return true;
    case 20: /* e_mulli */
        tcg_gen_muli_tl(cpu_gpr[rt], cpu_gpr[ra], (int32_t)imm);
        return true;
    case 21: /* e_cmpi */
        gen_op_cmpi32(cpu_gpr[ra], (target_long)(int32_t)imm, 1,
                      (insn >> 21) & 7);
        return true;
    case 22: /* e_cmpli */
        gen_op_cmpi32(cpu_gpr[ra], imm, 0, (insn >> 21) & 7);
        return true;
    case 24: /* e_andi */
    case 25: /* e_andi. */
        tcg_gen_andi_tl(cpu_gpr[rt], cpu_gpr[ra], imm);
        if (xop11 == 25) {
            gen_set_Rc0(ctx, cpu_gpr[rt]);
        }
        return true;
    case 26: /* e_ori */
    case 27: /* e_ori. */
        tcg_gen_ori_tl(cpu_gpr[rt], cpu_gpr[ra], imm);
        if (xop11 == 27) {
            gen_set_Rc0(ctx, cpu_gpr[rt]);
        }
        return true;
    case 28: /* e_xori */
    case 29: /* e_xori. */
        tcg_gen_xori_tl(cpu_gpr[rt], cpu_gpr[ra], imm);
        if (xop11 == 29) {
            gen_set_Rc0(ctx, cpu_gpr[rt]);
        }
        return true;
    default:
        return false;
    }
}

static bool vle_decode_e(DisasContext *ctx, CPUPPCState *env,
                         PowerPCCPU *cpu, uint32_t insn)
{
    int opc = insn >> 26;
    int rt = (insn >> 21) & 0x1f;
    int ra = (insn >> 16) & 0x1f;
    int32_t d16 = vle_d16(insn);

    switch (opc) {
    case 6:
        return vle_decode_e_opc6(ctx, insn);
    case 7: /* e_add16i */
        gen_op_arith_add(ctx, cpu_gpr[rt], cpu_gpr[ra],
                         tcg_constant_tl((target_long)d16),
                         cpu_ca, cpu_ca32, false, false, false, false);
        return true;
    case 12: /* e_lbz */
        vle_load(ctx, rt, ra, d16, MO_UB);
        return true;
    case 13: /* e_stb */
        vle_store(ctx, rt, ra, d16, MO_UB);
        return true;
    case 14: /* e_lha */
        vle_load(ctx, rt, ra, d16, MO_SW);
        return true;
    case 20: /* e_lwz */
        vle_load(ctx, rt, ra, d16, MO_UL);
        return true;
    case 21: /* e_stw */
        vle_store(ctx, rt, ra, d16, MO_UL);
        return true;
    case 22: /* e_lhz */
        vle_load(ctx, rt, ra, d16, MO_UW);
        return true;
    case 23: /* e_sth */
        vle_store(ctx, rt, ra, d16, MO_UW);
        return true;
    case 28:
        return vle_decode_e_opc28(ctx, insn);
    case 29:
        return vle_decode_e_opc29(ctx, insn);
    case 30:
        if (((insn >> 25) & 1) == 0) { /* e_b / e_bl */
            vle_branch_direct(ctx, ctx->cia + vle_e_bd24(insn), insn & 1);
            return true;
        }
        return vle_branch_cond32(ctx, insn);
    case 31:
        return vle_decode_e_opc31(ctx, cpu, insn);
    default:
        return false;
    }
}

static bool ppc_vle_translate_insn(DisasContext *ctx, CPUPPCState *env,
                                   PowerPCCPU *cpu)
{
    target_ulong pc = ctx->base.pc_next;
    bool ok;

    ctx->cia = pc;
    if (vle_is_e_form(translator_ldub(env, &ctx->base, pc))) {
        uint32_t insn = translator_ldl_swap(env, &ctx->base, pc,
                                            need_byteswap(ctx));
        ctx->base.pc_next = pc + 4;
        ok = vle_decode_e(ctx, env, cpu, insn);
    } else {
        uint16_t insn = translator_lduw_swap(env, &ctx->base, pc,
                                             need_byteswap(ctx));
        ctx->base.pc_next = pc + 2;
        ok = vle_decode_se(ctx, insn);
    }

    if (ctx->base.is_jmp == DISAS_NEXT &&
        !(ctx->base.pc_next & ~TARGET_PAGE_MASK)) {
        ctx->base.is_jmp = DISAS_TOO_MANY;
    }

    return ok;
}
