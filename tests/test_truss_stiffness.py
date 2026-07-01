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


# ---------------------------------------------------------------------------
# CH14-M3 geometry (Problems 14-7, 14-8)
#   Units: length = m, force = N
#   Nodes: N1(4,0) free, N2(2,0) free, N3(2,2) free, N4(0,2) pin, N5(0,0) pin
#   Members: M1(N2-N5), M2(N1-N2), M3(N1-N3), M4(N2-N3), M5(N3-N5), M6(N3-N4)
#   A = 0.0015 m^2, E = 200 GPa, EA = 3e8 N
# ---------------------------------------------------------------------------

def _ch14_m3() -> PlaneTruss:
    t = PlaneTruss()
    t.add_node("N1", 4, 0, ux_fixed=False, uy_fixed=False)
    t.add_node("N2", 2, 0, ux_fixed=False, uy_fixed=False)
    t.add_node("N3", 2, 2, ux_fixed=False, uy_fixed=False)
    t.add_node("N4", 0, 2, ux_fixed=True, uy_fixed=True)
    t.add_node("N5", 0, 0, ux_fixed=True, uy_fixed=True)
    E = 200_000_000_000  # Pa
    t.add_member("M1", "N2", "N5", A=0.0015, E=E)
    t.add_member("M2", "N1", "N2", A=0.0015, E=E)
    t.add_member("M3", "N1", "N3", A=0.0015, E=E)
    t.add_member("M4", "N2", "N3", A=0.0015, E=E)
    t.add_member("M5", "N3", "N5", A=0.0015, E=E)
    t.add_member("M6", "N3", "N4", A=0.0015, E=E)
    return t


# ---------------------------------------------------------------------------
# 14-7: Global stiffness matrix K
# No back-of-book numeric answer exists for this problem (assembly only), so
# checks are physics-based: each entry verified against the direct-stiffness
# formula k = EA/L * [[lx^2, lx*ly],[lx*ly, ly^2]] from the member's own
# direction cosines, plus overall symmetry.
# ---------------------------------------------------------------------------

class TestCH14P7:
    def setup_method(self):
        t = _ch14_m3()
        self.K, self.order = t.assemble_K()
        # Node insertion order: N1=0, N2=1, N3=2, N4=3, N5=4
        self.i1x, self.i1y = 0, 1
        self.i2x, self.i2y = 2, 3
        self.i3x, self.i3y = 4, 5
        self.i4x, self.i4y = 6, 7
        self.i5x, self.i5y = 8, 9
        self.EA_short = 300_000_000.0 / 2.0            # M1,M2,M4,M6 (L=2)
        self.EA_diag = 300_000_000.0 / (2 * 2 ** 0.5)  # M3,M5 (L=2*sqrt(2), 45 deg)

    def test_K_symmetry(self):
        assert np.allclose(self.K, self.K.T, atol=1e-3)

    def test_K_N3x_N3x(self):
        # M3 (N1-N3, lx=-0.7071,ly=0.7071): 0.5*EA_diag; M4 (N2-N3, lx=0): 0
        # M5 (N3-N5, lx=-0.7071,ly=-0.7071): 0.5*EA_diag; M6 (N3-N4, lx=-1): EA_short
        expected = self.EA_diag + self.EA_short
        assert abs(self.K[self.i3x, self.i3x] - expected) < 1.0

    def test_K_N3y_N3y(self):
        # M3: 0.5*EA_diag; M4: EA_short (ly=1); M5: 0.5*EA_diag; M6: 0 (ly=0)
        expected = self.EA_diag + self.EA_short
        assert abs(self.K[self.i3y, self.i3y] - expected) < 1.0

    def test_K_N1x_N1x(self):
        # M2 (N1-N2, lx=-1,ly=0): EA_short; M3 (N1-N3, lx=-0.7071,ly=0.7071): 0.5*EA_diag
        expected = self.EA_short + 0.5 * self.EA_diag
        assert abs(self.K[self.i1x, self.i1x] - expected) < 1.0

    def test_K_N2x_N5x_coupling(self):
        # M1 (N2-N5, lx=-1, ly=0): off-diag = -EA/L
        expected = -self.EA_short
        assert abs(self.K[self.i2x, self.i5x] - expected) < 1.0


# ---------------------------------------------------------------------------
# 14-8: Displacement and member force under 30 kN downward load at N1
# Textbook answers: uy_N2 = -0.00096569 m, M5 = -42400 N (compression)
# ---------------------------------------------------------------------------

class TestCH14P8:
    def setup_method(self):
        t = _ch14_m3()
        t.add_load("N1", Fx=0.0, Fy=-30000.0)
        self.res = t.solve()

    def test_uy_N2(self):
        _, uy = self.res.displacements["N2"]
        assert abs(uy - (-0.00096569)) < 0.00002, \
            f"uy_N2 = {uy:.8f} m, expected -0.00096569 m"

    def test_M5(self):
        F = self.res.member_forces["M5"]
        assert abs(F - (-42400.0)) < 200.0, f"M5 = {F:.1f} N, expected -42400 N"


# ---------------------------------------------------------------------------
# CH14-M4 geometry (Problems 14-9, 14-10, 14-11)
#   Units: length = m, force = N
#   Nodes: N1(8,3) free, N2(4,3) free, N3(4,0) free, N4(0,3) pin, N5(0,0) pin
#   Members: M1(N1-N3), M2(N1-N2), M3(N2-N4), M4(N2-N3), M5(N3-N4),
#            M6(N3-N5), M7(N5-N4)
#   A = 0.0015 m^2, E = 200 GPa, EA = 3e8 N
#   (14-11's fabrication-misfit case already covered by TestFabricationMisfit)
# ---------------------------------------------------------------------------

def _ch14_m4() -> PlaneTruss:
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
    t.add_member("M6", "N3", "N5", A=0.0015, E=E)
    t.add_member("M7", "N5", "N4", A=0.0015, E=E)
    return t


# ---------------------------------------------------------------------------
# 14-9: Global stiffness matrix K
# No back-of-book numeric answer exists for this problem (assembly only), so
# checks are physics-based: entries verified against the direct-stiffness
# formula k = EA/L * [[lx^2, lx*ly],[lx*ly, ly^2]] from each member's own
# direction cosines, plus overall symmetry.
# ---------------------------------------------------------------------------

class TestCH14P9:
    def setup_method(self):
        t = _ch14_m4()
        self.K, self.order = t.assemble_K()
        # Node insertion order: N1=0, N2=1, N3=2, N4=3, N5=4
        self.i1x, self.i1y = 0, 1
        self.i3x, self.i3y = 4, 5

    def test_K_symmetry(self):
        assert np.allclose(self.K, self.K.T, atol=1e-3)

    def test_K_N3x_N3x(self):
        # M1(N1-N3,lx=-0.8): 0.64*60e6; M4(N2-N3,lx=0): 0
        # M5(N3-N4,lx=-0.8): 0.64*60e6; M6(N3-N5,lx=-1): 75e6
        expected = 0.64 * 60e6 + 0.64 * 60e6 + 75e6
        assert abs(self.K[self.i3x, self.i3x] - expected) < 1.0

    def test_K_N3y_N3y(self):
        # M1(ly=-0.6): 0.36*60e6; M4(ly=-1): 100e6; M5(ly=0.6): 0.36*60e6; M6(ly=0): 0
        expected = 0.36 * 60e6 + 100e6 + 0.36 * 60e6
        assert abs(self.K[self.i3y, self.i3y] - expected) < 1.0

    def test_K_N1x_N1x(self):
        # M1(N1-N3,lx=-0.8,EA/L=60e6): 0.64*60e6; M2(N1-N2,lx=-1,EA/L=75e6): 75e6
        expected = 0.64 * 60e6 + 75e6
        assert abs(self.K[self.i1x, self.i1x] - expected) < 1.0


# ---------------------------------------------------------------------------
# 14-10: Member force under 20 kN downward load at N1
# Textbook answer: M5 = 33300 N (tension)
# ---------------------------------------------------------------------------

class TestCH14P10:
    def setup_method(self):
        t = _ch14_m4()
        t.add_load("N1", Fx=0.0, Fy=-20000.0)
        self.res = t.solve()

    def test_M5(self):
        F = self.res.member_forces["M5"]
        assert abs(F - 33300.0) < 200.0, f"M5 = {F:.1f} N, expected 33300 N"


# ---------------------------------------------------------------------------
# CH14-M5 geometry (Problems 14-12, 14-13, 14-14)
#   Units: length = in, force = kip
#   Nodes: N1(96,72) free, N2(96,0) roller (uy fixed, ux free),
#          N3(0,0) free, N4(0,72) pin
#   Members: M1(N1-N2), M2(N2-N3), M3(N1-N4), M4(N3-N4), M5(N1-N3), M6(N2-N4)
#   A = 2 in^2, E = 29000 ksi, EA = 58000 kip
# ---------------------------------------------------------------------------

def _ch14_m5(delta_L0_M3: float = 0.0) -> PlaneTruss:
    t = PlaneTruss()
    t.add_node("N1", 96, 72, ux_fixed=False, uy_fixed=False)
    t.add_node("N2", 96,  0, ux_fixed=False, uy_fixed=True)
    t.add_node("N3",  0,  0, ux_fixed=False, uy_fixed=False)
    t.add_node("N4",  0, 72, ux_fixed=True, uy_fixed=True)
    t.add_member("M1", "N1", "N2", A=2, E=29000)
    t.add_member("M2", "N2", "N3", A=2, E=29000)
    t.add_member("M3", "N1", "N4", A=2, E=29000, delta_L0=delta_L0_M3)
    t.add_member("M4", "N3", "N4", A=2, E=29000)
    t.add_member("M5", "N1", "N3", A=2, E=29000)
    t.add_member("M6", "N2", "N4", A=2, E=29000)
    return t


# ---------------------------------------------------------------------------
# 14-12: Global stiffness matrix K
# No back-of-book numeric answer exists for this problem (assembly only), so
# checks are physics-based: entries verified against the direct-stiffness
# formula k = EA/L * [[lx^2, lx*ly],[lx*ly, ly^2]] from each member's own
# direction cosines, plus overall symmetry.
# ---------------------------------------------------------------------------

class TestCH14P12:
    def setup_method(self):
        t = _ch14_m5()
        self.K, self.order = t.assemble_K()
        # Node insertion order: N1=0, N2=1, N3=2, N4=3
        self.i1x, self.i1y = 0, 1
        self.i3x, self.i3y = 4, 5

    def test_K_symmetry(self):
        assert np.allclose(self.K, self.K.T, atol=1e-3)

    def test_K_N1x_N1x(self):
        # M1(N1-N2,lx=0): 0; M3(N1-N4,lx=-1,EA/L=58000/96): full; M5(N1-N3,lx=-0.8,EA/L=58000/120): 0.64*
        expected = (58000 / 96) + 0.64 * (58000 / 120)
        assert abs(self.K[self.i1x, self.i1x] - expected) < 0.5

    def test_K_N3y_N3y(self):
        # M2(N2-N3,ly=0): 0; M4(N3-N4,ly=1,EA/L=58000/72): full; M5(N1-N3,ly=-0.6): 0.36*
        expected = (58000 / 72) + 0.36 * (58000 / 120)
        assert abs(self.K[self.i3y, self.i3y] - expected) < 0.5


# ---------------------------------------------------------------------------
# 14-13: Displacement and member force under 3-kip horizontal load at N3
# Textbook answers: ux_N2 = 0.005455 in, M5 = -1.64 kip (compression)
# ---------------------------------------------------------------------------

class TestCH14P13:
    def setup_method(self):
        t = _ch14_m5()
        t.add_load("N3", Fx=3.0, Fy=0.0)
        self.res = t.solve()

    def test_ux_N2(self):
        ux, _ = self.res.displacements["N2"]
        assert abs(ux - 0.005455) < 0.0001, f"ux_N2 = {ux:.6f} in, expected 0.005455 in"

    def test_M5(self):
        F = self.res.member_forces["M5"]
        assert abs(F - (-1.64)) < 0.05, f"M5 = {F:.3f} kip, expected -1.64 kip"


# ---------------------------------------------------------------------------
# 14-14: Same 3-kip load as 14-13, plus M3 fabricated 0.025 in too short
# Textbook answer: M3 = 3.55 kip (tension)
# ---------------------------------------------------------------------------

class TestCH14P14:
    def setup_method(self):
        t = _ch14_m5(delta_L0_M3=-0.025)
        t.add_load("N3", Fx=3.0, Fy=0.0)
        self.res = t.solve()

    def test_M3(self):
        F = self.res.member_forces["M3"]
        assert abs(F - 3.55) < 0.1, f"M3 = {F:.3f} kip, expected 3.55 kip"


# ---------------------------------------------------------------------------
# CH14-M6 geometry (Problems 14-15, 14-16) — inclined (skewed) roller support
#   Units: length = m, force = N
#   Nodes: N1(0,0) inclined roller (restrained normal to a 45-deg support
#          surface, free to slide tangent to it), N2(0,4) pin, N3(3,4) free
#   Members: M1(N1-N2, L=4 vertical), M2(N1-N3, L=5 diagonal),
#            M3(N2-N3, L=3 horizontal) — a 3-4-5 right triangle with the
#            right angle AT THE PIN (N2), not at N1.
#   Truss is statically determinate (3 members + 3 reactions = 6 DOF), so
#   member forces/reactions are independent of EA — the textbook expresses
#   uy_N3 symbolically as -29250/EA, so EA=1 is used here to compare directly.
#
#   Geometry note: reconstructed from Hibbeler's own intermediate member
#   direction-cosine calcs (not the simplified 3-node JSON, which had pin
#   and free joint swapped — that placement makes the roller's moment arm
#   about the pin vanish, forcing a zero reaction, contradicting the
#   textbook's 3182 N answer). With N2 at (0,4): moment equilibrium about
#   the pin gives Rx1=2250 N; a skew_angle_deg=45 roller (Rx1=Ry1) then
#   gives |R1|=2250*sqrt(2)=3182 N, matching the textbook exactly.
# ---------------------------------------------------------------------------

def _ch14_m6() -> PlaneTruss:
    t = PlaneTruss()
    t.add_node("N1", 0, 0, skew_angle_deg=45.0)
    t.add_node("N2", 0, 4, ux_fixed=True, uy_fixed=True)
    t.add_node("N3", 3, 4, ux_fixed=False, uy_fixed=False)
    t.add_member("M1", "N1", "N2", A=1.0, E=1.0)
    t.add_member("M2", "N1", "N3", A=1.0, E=1.0)
    t.add_member("M3", "N2", "N3", A=1.0, E=1.0)
    return t


# ---------------------------------------------------------------------------
# 14-15: Transformed global stiffness matrix (local-to-global DOF rotation
# at the inclined support). No back-of-book numeric answer for this
# assembly-only problem, so checks are physics-based: the transformed
# matrix must stay symmetric, and the N1 normal-normal entry must match
# EA/L * (l . n)^2 summed over members meeting at N1, where n=(cos45,sin45)
# is the restrained direction.
# ---------------------------------------------------------------------------

class TestCH14P15:
    def setup_method(self):
        t = _ch14_m6()
        self.Kt, self.T, self.order = t.assemble_K_transformed()
        # Node insertion order: N1=0, N2=1, N3=2; N1's local axes: tangent=0, normal=1
        self.i1_normal = 1

    def test_Kt_symmetry(self):
        assert np.allclose(self.Kt, self.Kt.T, atol=1e-9)

    def test_T_is_orthogonal(self):
        assert np.allclose(self.T @ self.T.T, np.eye(self.T.shape[0]), atol=1e-9)

    def test_N1_normal_normal_stiffness(self):
        # M1 (N1-N2): L=4, lx=0, ly=1; M2 (N1-N3): L=5, lx=0.6, ly=0.8
        # normal n=(cos45,sin45)=(0.7071,0.7071); component = EA/L*(l.n)^2
        # Textbook (page 516, code4-code4 entry): 0.321
        n = np.array([np.cos(np.radians(45)), np.sin(np.radians(45))])
        l_M1 = np.array([0.0, 1.0])
        l_M2 = np.array([0.6, 0.8])
        expected = (1.0/4) * (l_M1 @ n)**2 + (1.0/5) * (l_M2 @ n)**2
        assert abs(self.Kt[self.i1_normal, self.i1_normal] - expected) < 1e-6
        assert abs(expected - 0.321) < 0.001, f"expected {expected:.5f}, textbook 0.321"


# ---------------------------------------------------------------------------
# 14-16: 3 kN downward load at N3
# Textbook answers: uy_N3 = -29250/EA m, reaction_inclined = 3182 N,
# reaction_pin = (-2250, 750) N
# ---------------------------------------------------------------------------

class TestCH14P16:
    def setup_method(self):
        t = _ch14_m6()
        t.add_load("N3", Fx=0.0, Fy=-3000.0)
        self.res = t.solve()

    def test_uy_N3(self):
        _, uy = self.res.displacements["N3"]
        assert abs(uy - (-29250.0)) < 2.0, f"uy_N3 = {uy:.2f}/EA, expected -29250/EA"

    def test_reaction_pin(self):
        Rx, Ry = self.res.reactions["N2"]
        assert abs(Rx - (-2250.0)) < 5.0, f"Rx_N2 = {Rx:.1f} N, expected -2250 N"
        assert abs(Ry - 750.0) < 5.0, f"Ry_N2 = {Ry:.1f} N, expected 750 N"

    def test_reaction_inclined(self):
        Rx, Ry = self.res.reactions["N1"]
        magnitude = float(np.hypot(Rx, Ry))
        assert abs(magnitude - 3182.0) < 5.0, \
            f"|R_N1| = {magnitude:.1f} N, expected 3182 N"
        # Purely normal to the 45-deg incline: Rx == Ry
        assert abs(Rx - Ry) < 1.0, f"R_N1 not aligned to 45 deg: Rx={Rx:.1f}, Ry={Ry:.1f}"

    def test_statics_Fy(self):
        total_Ry = sum(Ry for _, Ry in self.res.reactions.values())
        assert abs(total_Ry - 3000.0) < 1e-6

    def test_statics_Fx(self):
        total_Rx = sum(Rx for Rx, _ in self.res.reactions.values())
        assert abs(total_Rx) < 1e-6
