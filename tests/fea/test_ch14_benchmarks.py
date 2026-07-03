"""
FEA-engine Phase 1 validation: Hibbeler Ch 14 truss benchmarks.

Two layers per model (geometry + published answers from
Reference/chapter_14_models.json, same tolerances as the PlaneTruss
oracle tests in tests/test_truss_stiffness.py):

  1. Published textbook answers (Hibbeler, Structural Analysis 8th Ed).
  2. Tier-4 regression: identical model through src/calcs/truss_stiffness
     PlaneTruss — displacements, reactions, member forces, and the free-DOF
     stiffness block must agree to near machine precision.

Assembly-only problems (14-1/4/7/9/12) have no back-of-book numbers; their
fea check is the free-DOF K block against the oracle's partitioned K (the
oracle's own entries are verified against the direct-stiffness formula in
tests/test_truss_stiffness.py), plus the 14-1 textbook spot value.

14-15/14-16 (inclined 45-deg roller): native skewed nodal axes arrive with
the Phase-2 transformation handler, so the fea model is validated by the
rotated-model equivalence — rotate the whole structure by -45 deg so the
roller's restrained normal lands on global x (an ordinary SP constraint),
solve, and rotate displacements/reactions back to the original frame.
Oracle: PlaneTruss's true skew_angle_deg solve on the unrotated model.
"""
import numpy as np
import pytest

from src.calcs.truss_stiffness import PlaneTruss
from src.fea.api import Model
from src.fea.analysis.static_analysis import StaticAnalysis

# spec rows: node = (tag, x, y, fix_x, fix_y); member = (tag, i, j, A, E)
# eleload = {member_tag: dict(delta_L0=..., delta_T=..., alpha=...)}


def _build_fea(nodes, members, loads=(), eleloads=None):
    m = Model(ndm=2, ndf=2)
    for tag, x, y, fx, fy in nodes:
        m.node(tag, float(x), float(y))
        if fx or fy:
            m.fix(tag, int(fx), int(fy))
    for tag, i, j, A, E in members:
        m.uniaxial_material("Elastic", tag, E=float(E))
        m.element("Truss", tag, i, j, A=float(A), mat=tag)
    if loads or eleloads:
        m.pattern("Plain", 1, "Linear")
        for tag, Fx, Fy in loads:
            m.load(tag, float(Fx), float(Fy))
        for tag, kw in (eleloads or {}).items():
            m.ele_load(tag, **kw)
    return m


def _build_oracle(nodes, members, loads=(), eleloads=None, skew=None):
    t = PlaneTruss()
    for tag, x, y, fx, fy in nodes:
        t.add_node(f"N{tag}", float(x), float(y),
                   ux_fixed=bool(fx), uy_fixed=bool(fy),
                   skew_angle_deg=(skew or {}).get(tag))
    for tag, i, j, A, E in members:
        kw = (eleloads or {}).get(tag, {})
        t.add_member(f"M{tag}", f"N{i}", f"N{j}", A=float(A), E=float(E),
                     delta_L0=kw.get("delta_L0", 0.0),
                     delta_T_F=kw.get("delta_T", 0.0),
                     alpha_per_F=kw.get("alpha", 0.0))
    for tag, Fx, Fy in loads:
        t.add_load(f"N{tag}", Fx=float(Fx), Fy=float(Fy))
    return t


def _solve_both(nodes, members, loads=(), eleloads=None):
    """Solve the same model through the fea engine and PlaneTruss; also
    return the fea analysis (for its assembled free-DOF K)."""
    m = _build_fea(nodes, members, loads, eleloads)
    analysis = StaticAnalysis(m.domain)
    analysis.analyze(1)
    res = _build_oracle(nodes, members, loads, eleloads).solve()
    return m, analysis, res


def _assert_matches_oracle(m, analysis, res, nodes, members,
                           rtol=1e-9, atol=1e-9):
    """Tier-4 regression: displacements, reactions, member forces, and the
    free-DOF stiffness block agree with PlaneTruss to near machine precision."""
    for tag, *_ in nodes:
        d = m.node_disp(tag)
        assert np.allclose(d, res.displacements[f"N{tag}"],
                           rtol=rtol, atol=atol), \
            f"N{tag} disp: fea {d} vs oracle {res.displacements[f'N{tag}']}"
        r = m.node_reaction(tag)
        assert np.allclose(r, res.reactions[f"N{tag}"],
                           rtol=rtol, atol=atol), \
            f"N{tag} reaction: fea {r} vs oracle {res.reactions[f'N{tag}']}"
    for tag, *_ in members:
        f_fea = m.ele_response(tag, "axial")
        f_ora = res.member_forces[f"M{tag}"]
        assert np.isclose(f_fea, f_ora, rtol=rtol, atol=atol), \
            f"M{tag} axial: fea {f_fea} vs oracle {f_ora}"
    # Free-DOF K block. Both orderings walk nodes in ascending tag order
    # (nodes are inserted sorted; PlainNumberer sorts tags), x then y.
    K, order = _build_oracle(nodes, members).assemble_K()
    free = []
    for k, (tag, x, y, fx, fy) in enumerate(nodes):
        if not fx:
            free.append(2 * k)
        if not fy:
            free.append(2 * k + 1)
    K_ff = K[np.ix_(free, free)]
    assert np.allclose(analysis.soe.A, K_ff, rtol=1e-12, atol=1e-9), \
        "fea assembled free-DOF K differs from PlaneTruss partition"


# ---------------------------------------------------------------------------
# CH14-M1 (problems 14-1, 14-2, 14-3) — units: kip, in
# ---------------------------------------------------------------------------

M1_NODES = [(1, 0, 0, 1, 1), (2, 0, 72, 1, 1), (3, 48, 36, 0, 0),
            (4, 120, 36, 1, 1)]
M1_MEMBERS = [(1, 1, 3, 0.5, 29000), (2, 3, 4, 0.5, 29000),
              (3, 2, 3, 0.5, 29000)]


class TestCH14M1:
    def setup_method(self):
        self.m, self.analysis, self.res = _solve_both(
            M1_NODES, M1_MEMBERS, loads=[(3, 0.0, -4.0)])

    def test_14_1_K_free_block_textbook(self):
        # Textbook K[N3x,N3x] = 510.72, K[N3y,N3y] = 174 kip/in
        A = self.analysis.soe.A
        assert abs(A[0, 0] - 510.72) < 0.5
        assert abs(A[1, 1] - 174.0) < 0.5

    def test_14_2_displacements(self):
        assert abs(self.m.node_disp(3, 0)) < 1e-9                    # ux = 0
        assert abs(self.m.node_disp(3, 1) - (-0.02299)) < 0.0001     # uy

    def test_14_3_member_forces(self):
        assert abs(self.m.ele_response(1, "axial") - (-3.33)) < 0.01
        assert abs(self.m.ele_response(2, "axial")) < 1e-9
        assert abs(self.m.ele_response(3, "axial") - 3.33) < 0.01

    def test_regression_vs_planetruss(self):
        _assert_matches_oracle(self.m, self.analysis, self.res,
                               M1_NODES, M1_MEMBERS)


# ---------------------------------------------------------------------------
# CH14-M2 (problems 14-4, 14-5, 14-6) — units: kip, in (JSON's lb scaled)
# ---------------------------------------------------------------------------

M2_NODES = [(1, 48, 48, 0, 0), (2, 0, 0, 1, 1), (3, 48, 0, 1, 1),
            (4, 84, 0, 1, 1)]
M2_MEMBERS = [(1, 1, 2, 0.75, 29000), (2, 1, 3, 0.75, 29000),
              (3, 1, 4, 0.75, 29000)]
M2_LOAD = [(1, -0.5, 0.0)]     # -500 lb


class TestCH14M2:
    def test_14_5_load_only(self):
        m, analysis, res = _solve_both(M2_NODES, M2_MEMBERS, loads=M2_LOAD)
        assert abs(m.node_disp(1, 0) - (-0.00172)) < 0.0001      # ux_N1, in
        assert abs(m.ele_response(2, "axial") - (-0.0127)) < 0.0005  # -12.7 lb
        _assert_matches_oracle(m, analysis, res, M2_NODES, M2_MEMBERS)

    def test_14_6_with_temperature(self):
        eleloads = {2: dict(delta_T=100.0, alpha=6.5e-6)}
        m, analysis, res = _solve_both(M2_NODES, M2_MEMBERS,
                                       loads=M2_LOAD, eleloads=eleloads)
        # Textbook: M2 = -6.57 kip
        assert abs(m.ele_response(2, "axial") - (-6.57)) < 0.05
        _assert_matches_oracle(m, analysis, res, M2_NODES, M2_MEMBERS)


# ---------------------------------------------------------------------------
# CH14-M3 (problems 14-7, 14-8) — units: N, m
# ---------------------------------------------------------------------------

_E = 200_000_000_000.0
M3_NODES = [(1, 4, 0, 0, 0), (2, 2, 0, 0, 0), (3, 2, 2, 0, 0),
            (4, 0, 2, 1, 1), (5, 0, 0, 1, 1)]
M3_MEMBERS = [(1, 2, 5, 0.0015, _E), (2, 1, 2, 0.0015, _E),
              (3, 1, 3, 0.0015, _E), (4, 2, 3, 0.0015, _E),
              (5, 3, 5, 0.0015, _E), (6, 3, 4, 0.0015, _E)]


class TestCH14M3:
    def setup_method(self):
        self.m, self.analysis, self.res = _solve_both(
            M3_NODES, M3_MEMBERS, loads=[(1, 0.0, -30000.0)])

    def test_14_8_answers(self):
        assert abs(self.m.node_disp(2, 1) - (-0.00096569)) < 0.00002
        assert abs(self.m.ele_response(5, "axial") - (-42400.0)) < 200.0

    def test_regression_vs_planetruss(self):
        _assert_matches_oracle(self.m, self.analysis, self.res,
                               M3_NODES, M3_MEMBERS)


# ---------------------------------------------------------------------------
# CH14-M4 (problems 14-9, 14-10, 14-11) — units: N, m
# ---------------------------------------------------------------------------

M4_NODES = [(1, 8, 3, 0, 0), (2, 4, 3, 0, 0), (3, 4, 0, 0, 0),
            (4, 0, 3, 1, 1), (5, 0, 0, 1, 1)]
M4_MEMBERS = [(1, 1, 3, 0.0015, _E), (2, 1, 2, 0.0015, _E),
              (3, 2, 4, 0.0015, _E), (4, 2, 3, 0.0015, _E),
              (5, 3, 4, 0.0015, _E), (6, 3, 5, 0.0015, _E),
              (7, 5, 4, 0.0015, _E)]


class TestCH14M4:
    def test_14_10_load(self):
        m, analysis, res = _solve_both(M4_NODES, M4_MEMBERS,
                                       loads=[(1, 0.0, -20000.0)])
        assert abs(m.ele_response(5, "axial") - 33300.0) < 200.0
        _assert_matches_oracle(m, analysis, res, M4_NODES, M4_MEMBERS)

    def test_14_11_fabrication_misfit_only(self):
        # M6 fabricated 0.01 m too long, no applied loads.
        eleloads = {6: dict(delta_L0=0.01)}
        m, analysis, res = _solve_both(M4_NODES, M4_MEMBERS,
                                       eleloads=eleloads)
        assert abs(m.node_disp(2, 1) - 0.01333) < 0.0005
        _assert_matches_oracle(m, analysis, res, M4_NODES, M4_MEMBERS)


# ---------------------------------------------------------------------------
# CH14-M5 (problems 14-12, 14-13, 14-14) — units: kip, in
# ---------------------------------------------------------------------------

M5_NODES = [(1, 96, 72, 0, 0), (2, 96, 0, 0, 1), (3, 0, 0, 0, 0),
            (4, 0, 72, 1, 1)]
M5_MEMBERS = [(1, 1, 2, 2, 29000), (2, 2, 3, 2, 29000),
              (3, 1, 4, 2, 29000), (4, 3, 4, 2, 29000),
              (5, 1, 3, 2, 29000), (6, 2, 4, 2, 29000)]
M5_LOAD = [(3, 3.0, 0.0)]


class TestCH14M5:
    def test_14_13_load_only(self):
        m, analysis, res = _solve_both(M5_NODES, M5_MEMBERS, loads=M5_LOAD)
        assert abs(m.node_disp(2, 0) - 0.005455) < 0.0001
        assert abs(m.ele_response(5, "axial") - (-1.64)) < 0.05
        _assert_matches_oracle(m, analysis, res, M5_NODES, M5_MEMBERS)

    def test_14_14_with_fabrication_misfit(self):
        # M3 fabricated 0.025 in too short + same 3-kip load.
        eleloads = {3: dict(delta_L0=-0.025)}
        m, analysis, res = _solve_both(M5_NODES, M5_MEMBERS,
                                       loads=M5_LOAD, eleloads=eleloads)
        assert abs(m.ele_response(3, "axial") - 3.55) < 0.1
        _assert_matches_oracle(m, analysis, res, M5_NODES, M5_MEMBERS)


# ---------------------------------------------------------------------------
# CH14-M6 (problems 14-15, 14-16) — inclined 45-deg roller; units: N, m
#
# Statically determinate (forces/reactions independent of EA); the textbook
# gives uy_N3 = -29250/EA, so EA = 1 is used to compare directly.
#
# fea has no skewed nodal axes until the Phase-2 transformation handler, so
# the fea model is solved in a frame rotated by -45 deg (roller normal
# (cos45, sin45) -> global +x, an ordinary ux restraint; the pin is
# rotation-invariant) and results are rotated back. The oracle is
# PlaneTruss's true skew_angle_deg=45 solve on the unrotated model.
# ---------------------------------------------------------------------------

class TestCH14M6:
    THETA = np.radians(-45.0)

    def setup_method(self):
        c, s = np.cos(self.THETA), np.sin(self.THETA)
        self.R = np.array([[c, -s], [s, c]])    # global -> rotated frame

        coords = {1: (0.0, 0.0), 2: (0.0, 4.0), 3: (3.0, 4.0)}
        load_N3 = np.array([0.0, -3000.0])

        m = Model(ndm=2, ndf=2)
        for tag, xy in coords.items():
            m.node(tag, *(self.R @ np.asarray(xy)))
        m.fix(1, 1, 0)   # rotated roller: normal is now global x
        m.fix(2, 1, 1)   # pin — invariant under rotation
        m.uniaxial_material("Elastic", 1, E=1.0)
        m.element("Truss", 1, 1, 2, A=1.0, mat=1)
        m.element("Truss", 2, 1, 3, A=1.0, mat=1)
        m.element("Truss", 3, 2, 3, A=1.0, mat=1)
        m.pattern("Plain", 1, "Linear")
        m.load(3, *(self.R @ load_N3))
        m.analyze(1)
        self.m = m

        # results rotated back to the original (global) frame
        self.disp = {t: self.R.T @ m.node_disp(t) for t in coords}
        self.reac = {t: self.R.T @ m.node_reaction(t) for t in coords}

        t = PlaneTruss()
        t.add_node("N1", 0, 0, skew_angle_deg=45.0)
        t.add_node("N2", 0, 4, ux_fixed=True, uy_fixed=True)
        t.add_node("N3", 3, 4)
        t.add_member("M1", "N1", "N2", A=1.0, E=1.0)
        t.add_member("M2", "N1", "N3", A=1.0, E=1.0)
        t.add_member("M3", "N2", "N3", A=1.0, E=1.0)
        t.add_load("N3", Fx=0.0, Fy=-3000.0)
        self.res = t.solve()

    def test_14_16_uy_N3(self):
        assert abs(self.disp[3][1] - (-29250.0)) < 2.0

    def test_14_16_reaction_pin(self):
        Rx, Ry = self.reac[2]
        assert abs(Rx - (-2250.0)) < 5.0
        assert abs(Ry - 750.0) < 5.0

    def test_14_16_reaction_inclined(self):
        Rx, Ry = self.reac[1]
        assert abs(float(np.hypot(Rx, Ry)) - 3182.0) < 5.0
        # Purely normal to the 45-deg incline: Rx == Ry in the global frame
        assert abs(Rx - Ry) < 1.0

    def test_statics(self):
        total = sum(self.reac[t] for t in (1, 2, 3))
        assert abs(total[0]) < 1e-6
        assert abs(total[1] - 3000.0) < 1e-6

    def test_regression_vs_planetruss_skew(self):
        for tag in (1, 2, 3):
            assert np.allclose(self.disp[tag],
                               self.res.displacements[f"N{tag}"],
                               rtol=1e-9, atol=1e-9)
            assert np.allclose(self.reac[tag],
                               self.res.reactions[f"N{tag}"],
                               rtol=1e-9, atol=1e-9)
        for tag in (1, 2, 3):
            assert np.isclose(self.m.ele_response(tag, "axial"),
                              self.res.member_forces[f"M{tag}"],
                              rtol=1e-9, atol=1e-9)
