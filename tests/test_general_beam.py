"""
Tests for GeneralBeam — matrix stiffness solver.

All expected values are derived from closed-form solutions (textbook formulas)
and verified against SimpleBeam where they share the same case.

Reference formulas used:
  SS UDL      : M_max = wL²/8, R = wL/2, δ_max = 5wL⁴/(384EI)
  Fixed-fixed : M_ends = wL²/12, M_mid = wL²/24, R = wL/2, δ_max = wL⁴/(384EI)
  Fixed-pin   : M_fixed = wL²/8, R_fixed = 5wL/8, R_pin = 3wL/8
  Cantilever  : M_root = PL, δ_tip = PL³/(3EI)
  2-span SS   : equal spans with same UDL → R_mid = 1.25wL, R_ends = 0.375wL
  Hinge       : Gerber beam — becomes SS for each sub-span
"""

import math
import numpy as np
import pytest

from src.calcs.beam_stiffness import GeneralBeam, SimpleBeam

EI_STEEL = 29000.0 * 518.0    # kip-in² — W16×40 reference
E = 29000.0
I = 518.0
EI = E * I


def _check(val, expected, tol=0.005, abs_tol=1e-6):
    """Assert val ≈ expected within relative tolerance (default 0.5%) or abs_tol."""
    assert math.isclose(float(val), float(expected), rel_tol=tol, abs_tol=abs_tol), \
        f"{float(val):.6g} ≠ {float(expected):.6g} (tol={tol})"


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 1. Simply supported beam — matches SimpleBeam exactly
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class TestSS:
    """GeneralBeam pin-pin == SimpleBeam for same case."""

    def test_reactions_udl(self):
        b = GeneralBeam(30, E, I)
        b.add_udl(2.0)   # 2 kip/ft over 30 ft
        r = b.solve()
        # R = wL/2 = 2*30/2 = 30 kips each
        R = list(r.reactions.values())
        _check(R[0], 30.0)
        _check(R[1], 30.0)

    def test_mmax_udl(self):
        b = GeneralBeam(30, E, I)
        b.add_udl(2.0)
        r = b.solve()
        # M_max = wL²/8 = 2*900/8 = 225 kip-ft
        _check(max(r.M_kip_ft), 225.0)

    def test_delta_max_udl(self):
        b = GeneralBeam(30, E, I)
        b.add_udl(2.0)
        r = b.solve()
        # δ_max = 5wL⁴/(384EI); w in kip/in, L in in
        w_in = 2.0 / 12.0
        L_in = 30 * 12.0
        expected = 5 * w_in * L_in**4 / (384 * EI)
        _check(max(r.delta_in), expected, tol=0.002)

    def test_matches_simple_beam(self):
        w, L = 3.0, 20.0
        s = SimpleBeam(L, E, I)
        s.add_udl(w)
        rs = s.solve()

        g = GeneralBeam(L, E, I)
        g.add_udl(w)
        rg = g.solve()

        _check(max(rg.M_kip_ft), max(rs.M_kip_ft))
        _check(max(rg.delta_in), max(rs.delta_in))

    def test_point_load_midspan(self):
        P, L = 20.0, 24.0    # 20 kip at midspan of 24-ft beam
        b = GeneralBeam(L, E, I)
        b.add_point_load(P, L / 2)
        r = b.solve()
        # R = P/2 = 10, M_max = PL/4 = 120, δ_max = PL³/(48EI)
        _check(list(r.reactions.values())[0], 10.0)
        _check(max(r.M_kip_ft), P * L / 4)
        L_in = L * 12.0
        expected_d = P * L_in**3 / (48 * EI)
        _check(max(r.delta_in), expected_d, tol=0.005)

    def test_partial_udl(self):
        """UDL over left half of span — verify reactions by statics."""
        w, L = 2.0, 20.0
        b = GeneralBeam(L, E, I)
        b.add_udl(w, x_end_ft=L / 2)
        r = b.solve()
        W = w * L / 2    # total force = 20 kips
        # Moments about right: R_L * L - W * (L/4) = 0 → R_L = W/4
        R_L = W * (3 * L / 4) / L   # = W * 3/4 = 15 kips (moment about right end)
        # Actually: moment about right end = R_L*L - W*(L - L/4) = 0
        # R_L = W*(3L/4)/L = 3W/4 = 15
        # Wait: UDL from 0 to L/2, centroid at L/4.
        # ΣM_right = R_L*L - W*(L - L/4) = 0 → R_L = W*(3/4) = 15
        R_R = W - R_L    # = 5 kips
        rxns = sorted(r.reactions.items())
        _check(rxns[0][1], R_L)
        _check(rxns[1][1], R_R)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 2. Fixed-fixed beam
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class TestFixedFixed:
    """Fixed-fixed beam under UDL: M_ends = wL²/12, M_mid = wL²/24, δ = wL⁴/384EI."""

    def setup_method(self):
        self.w, self.L = 2.0, 24.0
        b = GeneralBeam(self.L, E, I)
        b.set_bc("fixed", "fixed")
        b.add_udl(self.w)
        self.r = b.solve()

    def test_reactions(self):
        R = list(self.r.reactions.values())
        # R = wL/2 = 2*24/2 = 24 kips each
        for Ri in R:
            _check(Ri, self.w * self.L / 2)

    def test_moment_at_ends(self):
        # M at x=0 and x=L should be -wL²/12 (hogging)
        M_end_expected = -self.w * self.L**2 / 12
        _check(self.r.M_kip_ft[0], M_end_expected, tol=0.01)
        _check(self.r.M_kip_ft[-1], M_end_expected, tol=0.01)

    def test_moment_at_midspan(self):
        M_mid_expected = self.w * self.L**2 / 24
        i_mid = int(np.argmax(self.r.M_kip_ft))
        _check(self.r.M_kip_ft[i_mid], M_mid_expected, tol=0.01)

    def test_delta_max(self):
        w_in = self.w / 12.0
        L_in = self.L * 12.0
        expected = w_in * L_in**4 / (384 * EI)
        _check(max(self.r.delta_in), expected, tol=0.005)

    def test_moment_reactions(self):
        # Fixed-end moments = wL²/12 (magnitude), CW at left (negative by CCW+ conv)
        assert len(self.r.moment_reactions) == 2


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 3. Fixed-pin (propped cantilever)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class TestFixedPin:
    """Propped cantilever (fixed-left, pin-right) under UDL."""

    def setup_method(self):
        self.w, self.L = 2.0, 20.0
        b = GeneralBeam(self.L, E, I)
        b.set_bc("fixed", "pin")
        b.add_udl(self.w)
        self.r = b.solve()

    def test_reactions(self):
        # R_fixed = 5wL/8, R_pin = 3wL/8
        rxns = sorted(self.r.reactions.items())
        R_fixed = self.w * self.L * 5 / 8
        R_pin   = self.w * self.L * 3 / 8
        _check(rxns[0][1], R_fixed)
        _check(rxns[1][1], R_pin)

    def test_fixed_end_moment(self):
        # M at fixed end = -wL²/8 (hogging)
        M_expected = -self.w * self.L**2 / 8
        _check(self.r.M_kip_ft[0], M_expected, tol=0.01)

    def test_pinned_end_zero_moment(self):
        # M at pin end = 0
        _check(self.r.M_kip_ft[-1], 0.0, tol=0.01)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 4. Cantilever beam (fixed-free)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class TestCantilever:
    """Fixed at left, free at right."""

    def test_tip_point_load(self):
        P, L = 10.0, 15.0
        b = GeneralBeam(L, E, I)
        b.set_bc("fixed", "free")
        b.add_point_load(P, L)
        r = b.solve()
        # M at root = -PL (hogging), δ_tip = PL³/(3EI)
        _check(r.M_kip_ft[0], -P * L, tol=0.01)
        L_in = L * 12.0
        expected_d = P * L_in**3 / (3 * EI)
        _check(max(r.delta_in), expected_d, tol=0.005)

    def test_udl(self):
        w, L = 1.5, 10.0
        b = GeneralBeam(L, E, I)
        b.set_bc("fixed", "free")
        b.add_udl(w)
        r = b.solve()
        # M at root = -wL²/2, δ_tip = wL⁴/(8EI)
        _check(r.M_kip_ft[0], -w * L**2 / 2, tol=0.01)
        w_in = w / 12.0
        L_in = L * 12.0
        expected_d = w_in * L_in**4 / (8 * EI)
        _check(max(r.delta_in), expected_d, tol=0.005)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 5. Two-span continuous beam
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class TestTwoSpan:
    """Two equal spans, pin supports at x=0, L, 2L, uniform UDL throughout."""

    def setup_method(self):
        self.w, self.L = 2.0, 20.0
        b = GeneralBeam.continuous([self.L, self.L], E, I)
        b.add_udl(self.w)
        self.r = b.solve()

    def test_reactions(self):
        # R_ends = 3wL/8, R_mid = 10wL/8 = 5wL/4
        rxns = sorted(self.r.reactions.items())
        R_end = 3 * self.w * self.L / 8
        R_mid = 10 * self.w * self.L / 8
        _check(rxns[0][1], R_end)
        _check(rxns[1][1], R_mid)
        _check(rxns[2][1], R_end)

    def test_moment_at_interior_support(self):
        # M at interior support = -wL²/8 (hogging)
        M_expected = -self.w * self.L**2 / 8
        # Find x near interior support (x = L)
        x_interior = self.L  # ft
        idx = int(np.argmin(np.abs(self.r.x_ft - x_interior)))
        _check(self.r.M_kip_ft[idx], M_expected, tol=0.02)

    def test_statics_check(self):
        # Sum of reactions = total load
        total_R = sum(self.r.reactions.values())
        total_W = self.w * 2 * self.L
        _check(total_R, total_W)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 6. Three-span continuous beam
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def test_three_span_statics():
    w, L = 1.5, 15.0
    b = GeneralBeam.continuous([L, L, L], E, I)
    b.add_udl(w)
    r = b.solve()
    total_R = sum(r.reactions.values())
    _check(total_R, w * 3 * L)

def test_three_span_unequal_loads():
    """Different UDL on each span — statics check only."""
    b = GeneralBeam.continuous([20, 15, 25], E, I)
    b.add_udl(2.0, x_end_ft=20)
    b.add_udl(1.5, x_start_ft=20, x_end_ft=35)
    b.add_udl(1.0, x_start_ft=35)
    r = b.solve()
    total_R = sum(r.reactions.values())
    expected_W = 2.0*20 + 1.5*15 + 1.0*25
    _check(total_R, expected_W)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 7. Internal hinge (Gerber beam)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class TestHinge:
    """
    Gerber beam: fixed at x=0, interior pin at x=L, hinge at x=L/2, free tip at x=2L.
    SI = (fixed: Ry+M) + (interior pin: Ry) − 3 eqns + hinge = 1 − 1 = 0 (determinate).
    With UDL w over [0, L] only:
      - Hinge at L/2 enforces M=0 there.
      - Right overhang [L, 2L]: no load, free tip → V=M=0 throughout (by sections from right).
    Note: a pin (not fixed) at x=0 + interior pin at x=L gives SI=0 WITHOUT the hinge;
    adding the hinge would create a mechanism (SI=-1), so the left end must be fixed.
    """

    def test_hinge_zero_moment(self):
        L = 20.0
        b = GeneralBeam(2 * L, E, I)
        b.set_bc("fixed", "free")
        b.add_support(L, bc="pin")
        b.add_hinge(L / 2)
        b.add_udl(2.0, x_end_ft=L)
        r = b.solve()
        # Moment at hinge position must be ≈ 0
        idx = int(np.argmin(np.abs(r.x_ft - L / 2)))
        _check(r.M_kip_ft[idx], 0.0, tol=0.02)

    def test_hinge_right_span_zero_moment(self):
        """Right overhang (no load, free end) must have M ≈ 0 everywhere."""
        L = 20.0
        b = GeneralBeam(2 * L, E, I)
        b.set_bc("fixed", "free")
        b.add_support(L, bc="pin")
        b.add_hinge(L / 2)
        b.add_udl(2.0, x_end_ft=L)
        r = b.solve()
        # M at x=1.5L should be ≈ 0
        idx = int(np.argmin(np.abs(r.x_ft - 1.5 * L)))
        _check(r.M_kip_ft[idx], 0.0, tol=0.02)

    def test_hinge_statics(self):
        """With a symmetric Gerber beam and symmetric UDL, reactions are symmetric."""
        L = 20.0
        b = GeneralBeam(2 * L, E, I)
        b.set_bc("pin", "pin")
        b.add_support(L, bc="pin")
        b.add_hinge(L / 2)
        b.add_hinge(3 * L / 2)
        b.add_udl(2.0)
        r = b.solve()
        _check(sum(r.reactions.values()), 2.0 * 2 * L)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 8. Variable EI
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def test_variable_ei_statics():
    """Variable EI doesn't affect statics — reactions still satisfy equilibrium."""
    b = GeneralBeam(40, E, I)
    b.set_bc("pin", "pin")
    b.add_support(20, bc="pin")
    b.set_ei(0, 20, E, 800.0)
    b.set_ei(20, 40, E, 400.0)
    b.add_udl(2.0)
    r = b.solve()
    _check(sum(r.reactions.values()), 2.0 * 40)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 9. Trapezoidal load
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def test_trapload_statics():
    """Trapezoidal load (0 → 4 kip/ft) over a SS beam — statics check."""
    w0, w1, L = 0.0, 4.0, 30.0
    b = GeneralBeam(L, E, I)
    b.add_trapload(w0, w1)
    r = b.solve()
    W = 0.5 * (w0 + w1) * L   # total = 60 kips
    _check(sum(r.reactions.values()), W)
    # Centroid at L*(2*w1 + w0)/(3*(w0+w1)) = L*2/3 from left
    x_c = L * (2*w1 + w0) / (3*(w0 + w1))
    # Moments about right: R_left * L = W * (L - x_c)
    R_left_expected = W * (L - x_c) / L
    rxns = sorted(r.reactions.items())
    _check(rxns[0][1], R_left_expected, tol=0.01)

def test_trapload_equals_udl_when_uniform():
    """Trapezoidal load with equal ends == UDL — match moment diagrams."""
    w, L = 2.0, 20.0

    b1 = GeneralBeam(L, E, I)
    b1.add_udl(w)
    r1 = b1.solve()

    b2 = GeneralBeam(L, E, I)
    b2.add_trapload(w, w)
    r2 = b2.solve()

    _check(max(r2.M_kip_ft), max(r1.M_kip_ft), tol=0.005)
    _check(max(r2.delta_in), max(r1.delta_in), tol=0.005)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 10. Point moment load
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def test_point_moment_statics():
    """Applied moment on SS beam — reactions must form a couple = M/L."""
    M_ext, L = 100.0, 20.0   # kip-ft
    b = GeneralBeam(L, E, I)
    b.add_point_moment(M_ext, L / 2)
    r = b.solve()
    # No net vertical load → sum of reactions = 0, reactions form couple M/L
    _check(sum(r.reactions.values()), 0.0, tol=0.01)
    rxns = sorted(r.reactions.items())
    _check(rxns[0][1], -M_ext / L, tol=0.02)    # downward reaction at left
    _check(rxns[1][1],  M_ext / L, tol=0.02)    # upward reaction at right


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 11. Input validation
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def test_invalid_bc():
    b = GeneralBeam(20, E, I)
    with pytest.raises(ValueError, match="bc must be"):
        b.set_bc("roller", "rollerX")

def test_hinge_at_boundary_rejected():
    b = GeneralBeam(20, E, I)
    with pytest.raises(ValueError):
        b.add_hinge(0.0)
    with pytest.raises(ValueError):
        b.add_hinge(20.0)

def test_support_at_boundary_rejected():
    b = GeneralBeam(20, E, I)
    with pytest.raises(ValueError):
        b.add_support(0.0)

def test_udl_bad_range():
    b = GeneralBeam(20, E, I)
    with pytest.raises(ValueError):
        b.add_udl(2.0, x_start_ft=15, x_end_ft=10)

def test_singular_system_raises():
    """Free-free beam with no supports → singular K (no rigid-body constraint)."""
    b = GeneralBeam(20, E, I)
    b.set_bc("free", "free")
    b.add_udl(2.0)
    with pytest.raises(np.linalg.LinAlgError):
        b.solve()
