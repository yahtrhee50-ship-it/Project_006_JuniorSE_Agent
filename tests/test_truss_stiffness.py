"""
Tests for truss_stiffness.PlaneTruss — verified against Hibbeler Structural Analysis 8th Ed.

CH14-M1 geometry (Problems 14-1, 14-2, 14-3):
  Units: length = in, force = kip
  Nodes: N1(0,0) pin, N2(0,72) pin, N3(48,36) free, N4(120,36) pin
  Members: M1(N1-N3, L=60), M2(N3-N4, L=72), M3(N2-N3, L=60)
  A = 0.5 in², E = 29000 ksi, EA = 14500 kip
  Direction cosines:
    M1: lx=0.8, ly=0.6   (N1→N3)
    M2: lx=1.0, ly=0.0   (N3→N4)
    M3: lx=0.8, ly=-0.6  (N2→N3)
"""
import pytest
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from src.calcs.truss_stiffness import PlaneTruss


# ---------------------------------------------------------------------------
# Fixture: CH14-M1 truss
# ---------------------------------------------------------------------------

def _ch14_m1() -> PlaneTruss:
    t = PlaneTruss()
    t.add_node("N1",   0,  0, ux_fixed=True, uy_fixed=True)
    t.add_node("N2",   0, 72, ux_fixed=True, uy_fixed=True)
    t.add_node("N3",  48, 36, ux_fixed=False, uy_fixed=False)
    t.add_node("N4", 120, 36, ux_fixed=True, uy_fixed=True)
    t.add_member("M1", "N1", "N3", A=0.5, E=29000)
    t.add_member("M2", "N3", "N4", A=0.5, E=29000)
    t.add_member("M3", "N2", "N3", A=0.5, E=29000)
    return t


# ---------------------------------------------------------------------------
# 14-1: Global stiffness matrix K
#
# Textbook DOF ordering (free node first, then supports):
#   [uN3x, uN3y, uN1x, uN1y, uN2x, uN2y, uN4x, uN4y]
#
# Our node insertion order:
#   [uN1x, uN1y, uN2x, uN2y, uN3x, uN3y, uN4x, uN4y]
#
# We extract the (N3x, N3x) block terms directly to verify the physics.
# ---------------------------------------------------------------------------

class TestCH14P1:
    def setup_method(self):
        t = _ch14_m1()
        self.K, self.order = t.assemble_K()
        # Indices in our ordering: N1=0, N2=1, N3=2, N4=3
        # DOF: uN1x=0, uN1y=1, uN2x=2, uN2y=3, uN3x=4, uN3y=5, uN4x=6, uN4y=7
        self.i3x = 4   # uN3x
        self.i3y = 5   # uN3y
        self.i1x = 0
        self.i1y = 1
        self.i2x = 2
        self.i2y = 3
        self.i4x = 6
        self.i4y = 7

    # Diagonal of the free-node sub-block (textbook K[0,0] and K[1,1])
    def test_K_N3x_N3x(self):
        # M1+M2+M3 contributions: 14500/60*0.64 + 14500/72*1 + 14500/60*0.64
        expected = 2 * (14500/60 * 0.64) + (14500/72 * 1.0)
        assert abs(self.K[self.i3x, self.i3x] - expected) < 0.1, \
            f"K[N3x,N3x] = {self.K[self.i3x,self.i3x]:.3f}, expected {expected:.3f}"

    def test_K_N3y_N3y(self):
        # M1 + M3 contributions (M2 has ly=0)
        expected = 2 * (14500/60 * 0.36)
        assert abs(self.K[self.i3y, self.i3y] - expected) < 0.1

    def test_K_N3x_N3y(self):
        # M1: +0.48*EA/L; M2: 0; M3: -0.48*EA/L → cancels
        assert abs(self.K[self.i3x, self.i3y]) < 0.1

    # Off-diagonal coupling to N1
    def test_K_N3x_N1x(self):
        expected = -(14500/60 * 0.64)
        assert abs(self.K[self.i3x, self.i1x] - expected) < 0.1

    def test_K_N3x_N1y(self):
        expected = -(14500/60 * 0.48)
        assert abs(self.K[self.i3x, self.i1y] - expected) < 0.1

    # Off-diagonal coupling to N4 (through M2, horizontal member)
    def test_K_N3x_N4x(self):
        expected = -(14500/72 * 1.0)
        assert abs(self.K[self.i3x, self.i4x] - expected) < 0.1

    def test_K_N3x_N4y(self):
        # M2 has ly=0 → no coupling
        assert abs(self.K[self.i3x, self.i4y]) < 1e-9

    # Off-diagonal coupling to N2 (through M3)
    def test_K_N3x_N2x(self):
        expected = -(14500/60 * 0.64)
        assert abs(self.K[self.i3x, self.i2x] - expected) < 0.1

    def test_K_N3x_N2y(self):
        # M3: lx=0.8, ly=-0.6; off-diag = -(EA/L)*lx*ly = +116
        expected = 14500/60 * 0.8 * 0.6
        assert abs(self.K[self.i3x, self.i2y] - expected) < 0.1

    # Symmetry
    def test_K_symmetry(self):
        assert np.allclose(self.K, self.K.T, atol=1e-10)

    # Textbook numeric spot checks (from textbook answer, kip/in)
    def test_K_N3x_N3x_textbook(self):
        assert abs(self.K[self.i3x, self.i3x] - 510.72) < 0.5  # textbook: 510.72

    def test_K_N3y_N3y_textbook(self):
        assert abs(self.K[self.i3y, self.i3y] - 174.0) < 0.5   # textbook: 174

    def test_K_N3x_N4x_textbook(self):
        assert abs(self.K[self.i3x, self.i4x] - (-201.39)) < 0.5  # textbook: -201.39


# ---------------------------------------------------------------------------
# 14-2: Displacements under 4-kip downward load at N3
# Textbook answers: ux_N3 = 0, uy_N3 = -0.02299 in
# ---------------------------------------------------------------------------

class TestCH14P2:
    def setup_method(self):
        t = _ch14_m1()
        t.add_load("N3", Fx=0.0, Fy=-4.0)   # 4 kip downward
        self.res = t.solve()

    def test_ux_N3_is_zero(self):
        ux, _ = self.res.displacements["N3"]
        assert abs(ux) < 1e-9, f"ux_N3 should be 0, got {ux}"

    def test_uy_N3(self):
        _, uy = self.res.displacements["N3"]
        # Textbook: -0.02299 in (downward)
        assert abs(uy - (-0.02299)) < 0.0002, \
            f"uy_N3 = {uy:.5f} in, expected -0.02299 in"

    def test_supports_have_zero_displacement(self):
        for nid in ("N1", "N2", "N4"):
            ux, uy = self.res.displacements[nid]
            assert abs(ux) < 1e-10 and abs(uy) < 1e-10, \
                f"Support {nid} displaced: ux={ux}, uy={uy}"

    def test_statics_Fx(self):
        total_Rx = sum(Rx for Rx, _ in self.res.reactions.values())
        assert abs(total_Rx - 0.0) < 1e-6

    def test_statics_Fy(self):
        total_Ry = sum(Ry for _, Ry in self.res.reactions.values())
        assert abs(total_Ry - 4.0) < 1e-6   # reactions balance 4 kip down


# ---------------------------------------------------------------------------
# 14-3: Member forces
# Textbook answers: M1=-3.33 kip (compression), M2=0, M3=+3.33 kip (tension)
# ---------------------------------------------------------------------------

class TestCH14P3:
    def setup_method(self):
        t = _ch14_m1()
        t.add_load("N3", Fx=0.0, Fy=-4.0)
        self.res = t.solve()

    def test_M1_compression(self):
        F = self.res.member_forces["M1"]
        assert abs(F - (-3.33)) < 0.05, f"M1 = {F:.3f} kip, expected -3.33 kip"

    def test_M2_zero(self):
        F = self.res.member_forces["M2"]
        assert abs(F) < 0.01, f"M2 = {F:.3f} kip, expected 0"

    def test_M3_tension(self):
        F = self.res.member_forces["M3"]
        assert abs(F - 3.33) < 0.05, f"M3 = {F:.3f} kip, expected +3.33 kip"


# ---------------------------------------------------------------------------
# Simple sanity: single horizontal member, unit load
# ---------------------------------------------------------------------------

class TestSingleMember:
    def test_axial_deformation(self):
        t = PlaneTruss()
        t.add_node("A", 0, 0, ux_fixed=True, uy_fixed=True)
        t.add_node("B", 120, 0, ux_fixed=False, uy_fixed=True)
        t.add_member("M1", "A", "B", A=1.0, E=29000)
        t.add_load("B", Fx=10.0, Fy=0.0)
        res = t.solve()
        # δ = PL/AE = 10*120/(1*29000) = 0.04138 in
        expected_ux = 10.0 * 120.0 / (1.0 * 29000.0)
        ux, uy = res.displacements["B"]
        assert abs(ux - expected_ux) < 1e-9
        assert abs(uy) < 1e-9

    def test_axial_force_tension(self):
        t = PlaneTruss()
        t.add_node("A", 0, 0, ux_fixed=True, uy_fixed=True)
        t.add_node("B", 120, 0, ux_fixed=False, uy_fixed=True)
        t.add_member("M1", "A", "B", A=1.0, E=29000)
        t.add_load("B", Fx=10.0, Fy=0.0)
        res = t.solve()
        assert abs(res.member_forces["M1"] - 10.0) < 1e-6

    def test_mechanism_raises(self):
        t = PlaneTruss()
        t.add_node("A", 0, 0, ux_fixed=False, uy_fixed=False)
        t.add_node("B", 120, 0, ux_fixed=False, uy_fixed=False)
        t.add_member("M1", "A", "B", A=1.0, E=29000)
        t.add_load("B", Fx=10.0)
        with pytest.raises(ValueError, match="[Ss]ingular|[Mm]echanism"):
            t.solve()


# ---------------------------------------------------------------------------
# Initial-strain (fabrication misfit / thermal) equivalent nodal loads
# ---------------------------------------------------------------------------

def _ch14_m2(delta_T_F: float = 0.0, alpha_per_F: float = 0.0) -> PlaneTruss:
    """CH14-M2 geometry (Problems 14-4, 14-5, 14-6). Units: in, lb (E in psi here,
    kept internally consistent — same ratio as ksi/kip)."""
    t = PlaneTruss()
    t.add_node("N1", 48, 48, ux_fixed=False, uy_fixed=False)
    t.add_node("N2",  0,  0, ux_fixed=True, uy_fixed=True)
    t.add_node("N3", 48,  0, ux_fixed=True, uy_fixed=True)
    t.add_node("N4", 84,  0, ux_fixed=True, uy_fixed=True)
    t.add_member("M1", "N1", "N2", A=0.75, E=29000)
    t.add_member("M2", "N1", "N3", A=0.75, E=29000,
                 delta_T_F=delta_T_F, alpha_per_F=alpha_per_F)
    t.add_member("M3", "N1", "N4", A=0.75, E=29000)
    return t


class TestCH14P6:
    """14-6: same load as 14-5 (Fx=-0.5 kip at N1) plus +100 F on M2.
    Textbook answer: M2 = -6.57 kip."""

    def setup_method(self):
        t = _ch14_m2(delta_T_F=100.0, alpha_per_F=6.5e-6)
        t.add_load("N1", Fx=-0.5, Fy=0.0)
        self.res = t.solve()

    def test_M2_with_temperature(self):
        F = self.res.member_forces["M2"]
        assert abs(F - (-6.57)) < 0.05, f"M2 = {F:.3f} kip, expected -6.57 kip"


class TestFabricationMisfit:
    """14-11: no applied load, member M6 fabricated 0.01 m too long (CH14-M4
    geometry, SI units). Textbook answer: uy_N2 = +0.01333 m."""

    def setup_method(self):
        t = PlaneTruss()
        t.add_node("N1", 8, 3, ux_fixed=False, uy_fixed=False)
        t.add_node("N2", 4, 3, ux_fixed=False, uy_fixed=False)
        t.add_node("N3", 4, 0, ux_fixed=False, uy_fixed=False)
        t.add_node("N4", 0, 3, ux_fixed=True, uy_fixed=True)
        t.add_node("N5", 0, 0, ux_fixed=True, uy_fixed=True)
        E = 200_000_000_000  # Pa
        t.add_member("M1", "N1", "N3", A=0.0015, E=E)
        t.add_member("M2", "N1", "N2", A=0.0015, E=E)
        t.add_member("M3", "N2", "N4", A=0.0015, E=E)
        t.add_member("M4", "N2", "N3", A=0.0015, E=E)
        t.add_member("M5", "N3", "N4", A=0.0015, E=E)
        t.add_member("M6", "N3", "N5", A=0.0015, E=E, delta_L0=0.01)
        t.add_member("M7", "N5", "N4", A=0.0015, E=E)
        self.res = t.solve()

    def test_uy_N2(self):
        _, uy = self.res.displacements["N2"]
        assert abs(uy - 0.01333) < 0.0005, f"uy_N2 = {uy:.5f} m, expected 0.01333 m"
