"""
Unit tests for ACI 318-19 concrete beam calc module.

All expected values are hand-computed from first principles so the user can
cross-check against textbook examples.  Tolerances are set to 0.5% or 1 significant
figure, whichever is looser, to allow for rounding in intermediate steps.

Reference section geometry for most tests:
  b = 12 in, h = 24 in, d = 21.5 in (approx; varies by bar)
  f'c = 4.0 ksi (4000 psi), fy = 60 ksi, NW concrete (λ=1.0)
"""
import math
import pytest

from src.calcs import aci318, rebar


# ---------------------------------------------------------------------------
# rebar.py
# ---------------------------------------------------------------------------

class TestRebar:
    def test_get_bar_hash_prefix(self):
        b = rebar.get_bar("#8")
        assert b["db_in"] == pytest.approx(1.000)
        assert b["Ab_in2"] == pytest.approx(0.79)

    def test_get_bar_no_hash(self):
        b = rebar.get_bar("5")
        assert b["Ab_in2"] == pytest.approx(0.31)

    def test_get_bar_invalid(self):
        with pytest.raises(ValueError):
            rebar.get_bar("#99")

    def test_parse_bar_string(self):
        n, d = rebar.parse_bar_string("4#8")
        assert n == 4 and d == "#8"

    def test_parse_bar_string_space(self):
        n, d = rebar.parse_bar_string("3 #10")
        assert n == 3 and d == "#10"


# ---------------------------------------------------------------------------
# Material helpers
# ---------------------------------------------------------------------------

class TestMaterialHelpers:
    def test_beta1_4ksi(self):
        # f'c = 4000 psi → β₁ = 0.85
        assert aci318.beta1(4.0) == pytest.approx(0.85)

    def test_beta1_6ksi(self):
        # f'c = 6000 psi → β₁ = 0.85 − 0.05*(6000−4000)/1000 = 0.75
        assert aci318.beta1(6.0) == pytest.approx(0.75)

    def test_beta1_min(self):
        # f'c = 12 ksi → formula gives 0.85−0.05*8 = 0.45 < 0.65 → clamped at 0.65
        assert aci318.beta1(12.0) == pytest.approx(0.65)

    def test_Ec_ksi_4ksi(self):
        # Ec = 33 × 145^1.5 × √4000 / 1000 ≈ 3645 ksi
        Ec = aci318.Ec_ksi(4.0)
        assert Ec == pytest.approx(3645, rel=0.005)

    def test_modular_ratio(self):
        # n = ceil(29000 / 3645) = ceil(7.95) = 8
        assert aci318.modular_ratio(4.0) == 8

    def test_min_As(self):
        # b=12, d=21.5, f'c=4ksi, fy=60ksi
        # As,min = max(3√4000/60000, 200/60000) × 12 × 21.5
        # = max(0.003162, 0.003333) × 258 = 0.003333 × 258 = 0.860 in²
        As_min = aci318.min_As(12.0, 21.5, 4.0, 60.0)
        assert As_min == pytest.approx(0.860, rel=0.005)


# ---------------------------------------------------------------------------
# Flexure
# ---------------------------------------------------------------------------

class TestFlexure:
    def test_flexure_4x8_bars(self):
        """4 #8 bars in 12×24 beam: hand check a, c, φMn."""
        # As = 4×0.79 = 3.16 in²
        # a = 3.16×60 / (0.85×4×12) = 189.6 / 40.8 = 4.647 in
        # c = 4.647 / 0.85 = 5.467 in
        # εt = 0.003×(21.5−5.467)/5.467 = 0.003×2.932 = 0.00880
        # Mn = 3.16×60×(21.5−4.647/2) = 189.6×19.177 = 3637 k-in
        # φMn = 0.90×3637/12 = 272.8 kip-ft
        res = aci318.flexure_check(12.0, 21.5, 3.16, 4.0, 60.0, 150.0)
        assert res.a_in     == pytest.approx(4.647, rel=0.005)
        assert res.c_in     == pytest.approx(5.467, rel=0.005)
        assert res.eps_t    == pytest.approx(0.00880, rel=0.01)
        assert res.tension_controlled is True
        assert res.phi_Mn_kip_ft == pytest.approx(272.8, rel=0.005)

    def test_flexure_DCR(self):
        res = aci318.flexure_check(12.0, 21.5, 3.16, 4.0, 60.0, 150.0)
        # DCR = 150 / 272.8 ≈ 0.550
        assert res.DCR == pytest.approx(150.0 / res.phi_Mn_kip_ft, rel=1e-6)

    def test_flexure_overstressed(self):
        res = aci318.flexure_check(12.0, 21.5, 3.16, 4.0, 60.0, 300.0)
        assert res.DCR > 1.0

    def test_required_As(self):
        """required_As should give As that yields φMn ≈ Mu."""
        As = aci318.required_As(12.0, 21.5, 4.0, 60.0, 200.0)
        res = aci318.flexure_check(12.0, 21.5, As, 4.0, 60.0, 200.0)
        assert res.phi_Mn_kip_ft == pytest.approx(200.0, rel=0.005)

    def test_min_As_flag(self):
        """Very small As triggers As < As,min note."""
        res = aci318.flexure_check(12.0, 21.5, 0.10, 4.0, 60.0, 5.0)
        assert any("As,min" in n for n in res.notes)


# ---------------------------------------------------------------------------
# Shear
# ---------------------------------------------------------------------------

class TestShear:
    def test_shear_with_stirrups(self):
        """12×24 beam, 4#8 main, #3@8 stirrups (2-leg), f'c=4ksi.

        Hand:
          ρw = 3.16/(12×21.5) = 0.01225
          Vc = [8×1.0×(0.01225)^(1/3)×√4000]×12×21.5 / 1000
             = [8×0.2310×63.25]×258 / 1000 = [116.9]×258/1000 = 30.16 kips
          Vs = 0.22×60×21.5/8 = 35.48 kips
          φVn = 0.75×(30.16+35.48) = 49.24 kips
        """
        As_in2 = 4 * rebar.get_bar("#8")["Ab_in2"]     # 3.16 in²
        Av_in2 = 2 * rebar.get_bar("#3")["Ab_in2"]     # 0.22 in²
        res = aci318.shear_check(
            b_in=12.0, d_in=21.5, As_in2=As_in2, h_in=24.0,
            fc_ksi=4.0, fy_ksi=60.0,
            Av_in2=Av_in2, s_in=8.0, fyt_ksi=60.0,
            Vu_kips=40.0,
        )
        assert res.Vc_kips   == pytest.approx(30.16, rel=0.01)
        assert res.Vs_kips   == pytest.approx(35.48, rel=0.01)
        assert res.phi_Vn_kips == pytest.approx(49.24, rel=0.01)
        assert res.DCR == pytest.approx(40.0 / 49.24, rel=0.01)

    def test_shear_has_min_Av(self):
        As_in2 = 4 * rebar.get_bar("#8")["Ab_in2"]
        Av_in2 = 2 * rebar.get_bar("#3")["Ab_in2"]
        res = aci318.shear_check(12.0, 21.5, As_in2, 24.0, 4.0, 60.0,
                                  Av_in2, 8.0, 60.0, 30.0)
        assert res.has_min_Av is True

    def test_shear_spacing_flag(self):
        """Stirrup spacing exceeding s_max should trigger a note."""
        As_in2 = 4 * rebar.get_bar("#8")["Ab_in2"]
        Av_in2 = 2 * rebar.get_bar("#3")["Ab_in2"]
        # d/2 = 10.75 in; pass s=18 in (>10.75) with low Vs
        res = aci318.shear_check(12.0, 21.5, As_in2, 24.0, 4.0, 60.0,
                                  Av_in2, 18.0, 60.0, 20.0)
        assert any("spacing" in n.lower() for n in res.notes)


# ---------------------------------------------------------------------------
# Deflection
# ---------------------------------------------------------------------------

class TestDeflection:
    def test_Ig(self):
        # Ig = 12×24³/12 = 13 824 in⁴
        b, h = 12.0, 24.0
        Ig_expected = b * h**3 / 12.0
        assert Ig_expected == pytest.approx(13824.0)

    def test_Mcr(self):
        # fr = 7.5×√4000/1000 = 0.4744 ksi; Ig=13824 in⁴; yt=12 in
        # Mcr = fr×Ig/yt = 0.4744×13824/12 = 546.5 kip-in = 45.5 kip-ft
        fc_ksi = 4.0
        fr = 7.5 * math.sqrt(fc_ksi * 1000) / 1000.0
        Ig = 12.0 * 24.0**3 / 12.0
        Mcr_kip_in = fr * Ig / 12.0     # yt = h/2 = 12 in
        Mcr_kip_ft = Mcr_kip_in / 12.0  # convert in → ft
        assert Mcr_kip_ft == pytest.approx(45.5, rel=0.01)

    def test_deflection_simply_supported(self):
        """Immediate deflections must be positive and L/360 limit meaningful."""
        res = aci318.deflection_check(
            b_in=12.0, h_in=24.0, d_in=21.5, As_in2=3.16,
            fc_ksi=4.0, wD_klf=1.0, wL_klf=0.5, span_ft=20.0,
        )
        assert res.delta_i_D_in > 0
        assert res.delta_i_L_in > 0
        assert res.delta_total_in > res.delta_i_DL_in   # LT adds to total
        assert res.lambda_delta == pytest.approx(2.0)
        assert res.limit_L_360_in == pytest.approx(20.0 * 12.0 / 360.0, rel=1e-6)

    def test_deflection_long_term_factor(self):
        """λΔ = 2.0 for singly reinforced (ρ'=0, ξ=2.0, 5-yr)."""
        res = aci318.deflection_check(12.0, 24.0, 21.5, 3.16, 4.0, 1.0, 0.5, 20.0)
        assert res.delta_lt_add_in == pytest.approx(2.0 * res.delta_i_D_in, rel=1e-6)


# ---------------------------------------------------------------------------
# Development lengths
# ---------------------------------------------------------------------------

class TestDevelopmentLengths:
    def test_straight_ld_conservative(self):
        """#8 bar, f'c=4 ksi, fy=60 ksi, cover=1.5, Ktr=0 (conservative).

        Hand:
          ψt=1.0, ψe=1.0, ψs=1.0 (#8>#6), ψg=1.0 (Gr60)
          cb = 1.5 + 1.0/2 = 2.0 in  →  cb/db = 2.0/1.0 = 2.0 ≤ 2.5
          ld = (3/40)×(60000/(1.0×√4000))×(1.0/2.0)×1.0 = (3/40)×948.7×0.5 = 35.58 in
        """
        res = aci318.development_length_straight("#8", 4.0, 60.0, cover_in=1.5)
        # Allow 2% tolerance (rounding differences in √4000)
        assert res.ld_in == pytest.approx(35.6, rel=0.02)
        assert res.end_type == "straight"

    def test_straight_minimum_12in(self):
        """Small bar at high f'c should never give ld < 12 in."""
        res = aci318.development_length_straight("#3", 8.0, 60.0, cover_in=2.0)
        assert res.ld_in >= 12.0

    def test_hook_90(self):
        """#8, f'c=4 ksi, fy=60 ksi, all factors = 1.0 (cover NOT ok → ψo=1.0).

        ψe=1.0, ψr=1.0, ψo=1.0 (cover_ok=False), ψc=1.0
        ldh = (60000×1.0)/(55×1.0×√4000) = 60000/3478.6 = 17.25 in  ≥ max(8×1, 6) = 8 in

        Note: default cover_ok=True gives ψo=0.8 (favorable cover → shorter ldh = 13.8 in).
        """
        res = aci318.development_length_hook(
            "#8", 4.0, 60.0, hook_angle=90, cover_ok=False
        )
        assert res.ld_in == pytest.approx(17.25, rel=0.02)
        assert res.end_type == "hook_90"

    def test_hook_minimum(self):
        res = aci318.development_length_hook("#3", 4.0, 60.0)
        # 8×0.375 = 3.0 in < 6 in → minimum governs at 6 in
        assert res.ld_in >= 6.0

    def test_thead_basic(self):
        """#8, f'c=4 ksi, fy=60 ksi, no reduction.

        ldt = (60000×1.0)/(75×1.0×√4000) = 60000/4743.4 = 12.65 in  ≥ max(8×1, 6) = 8 in
        """
        res = aci318.development_length_thead("#8", 4.0, 60.0)
        assert res.ld_in == pytest.approx(12.65, rel=0.02)
        assert res.end_type == "thead"

    def test_thead_invalid_bar(self):
        with pytest.raises(NotImplementedError):
            aci318.development_length_thead("#14", 4.0, 60.0)

    def test_top_bar_psi_t(self):
        """Top-bar factor ψt=1.3 increases ld vs. other bar."""
        ld_top   = aci318.development_length_straight("#8", 4.0, 60.0, bar_location="top")
        ld_other = aci318.development_length_straight("#8", 4.0, 60.0, bar_location="other")
        assert ld_top.ld_in > ld_other.ld_in


# ---------------------------------------------------------------------------
# Splices
# ---------------------------------------------------------------------------

class TestSplices:
    def test_class_B_is_1p3_ld(self):
        ls, dev = aci318.splice_length("#8", 4.0, 60.0, splice_class="B")
        assert ls == pytest.approx(1.3 * dev.ld_in, rel=1e-6)

    def test_class_A_is_1p0_ld(self):
        ls, dev = aci318.splice_length("#8", 4.0, 60.0, splice_class="A")
        assert ls == pytest.approx(1.0 * dev.ld_in, rel=1e-6)

    def test_class_B_longer_than_A(self):
        ls_B, _ = aci318.splice_length("#8", 4.0, 60.0, splice_class="B")
        ls_A, _ = aci318.splice_length("#8", 4.0, 60.0, splice_class="A")
        assert ls_B > ls_A


# ---------------------------------------------------------------------------
# Curtailment
# ---------------------------------------------------------------------------

class TestCurtailment:
    def test_theoretical_cutoff_positive(self):
        """Cut 2 of 4 #8 bars in 12×24 beam — use wu heavy enough to need all 4 bars.

        φMn_remain(2#8) ≈ 144.6 kip-ft.
        Need wu × 20² / 8 > 144.6  →  wu > 2.89 klf.  Use wu = 5 klf.
        Mu,max = 5×400/8 = 250 kip-ft > 144.6 → cutoff required.
        """
        res = aci318.bar_curtailment(
            b_in=12.0, d_in=21.5, n_total=4, n_cut=2, bar_desig="#8",
            fc_ksi=4.0, fy_ksi=60.0, wu_klf=5.0, span_ft=20.0,
        )
        assert res.x_theoretical_ft > 0
        assert res.x_theoretical_ft < 10.0   # must be less than L/2
        assert res.x_cutoff_ft > res.x_theoretical_ft   # cutoff extends past theoretical

    def test_extension_requirement(self):
        """Extension = max(d, 12db) = max(21.5, 12×1.0) = 21.5 in. (wu=5 klf triggers cutoff)"""
        res = aci318.bar_curtailment(12.0, 21.5, 4, 2, "#8", 4.0, 60.0, 5.0, 20.0)
        # db=1.0 → 12db=12 in; d=21.5 in → max = 21.5
        assert res.extension_req_in == pytest.approx(21.5, rel=1e-6)

    def test_no_cutoff_when_small_moment(self):
        """If remaining bars cover full span, no cutoff needed."""
        # Very light load → remaining 2 bars easily cover full span
        res = aci318.bar_curtailment(12.0, 21.5, 4, 2, "#8", 4.0, 60.0, 0.1, 20.0)
        assert "no cutoff" in " ".join(res.notes).lower()
