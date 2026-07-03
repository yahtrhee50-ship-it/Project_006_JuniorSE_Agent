"""
FEA-engine Phase 1 validation: Hibbeler Ch 16 2D rigid-frame benchmarks.

Geometry, member properties, applied loads, and published textbook answers
are taken from Reference/hibbeler_chapter_16_node_member_model/
chapter_16_models.json (verified against the raw OCR in chapter_16.md,
per docs/sessions/2026-06-30_session.md). Six models, problems 16-1
through 16-12: each model's "assemble K" problem doesn't have a numeric
answer to check (Tier-1 assembly-only), and its paired analysis problem
has published free-DOF displacements/rotations and support reactions.

Every model also gets a Tier-3 statics sanity check (guiding-principle
gate from docs/fea_engine_plan.md): sum of reactions + sum of applied
nodal/member loads (force and moment about the origin) must vanish.

Node support flags: (tag, x, y, fix_ux, fix_uy, fix_rz).
Member spec: (tag, i, j, E, A, I).
"""
import numpy as np

from src.fea.api import Model

_DOF = {"ux": 0, "uy": 1, "rz": 2}


def _build(nodes, members, nodal_loads=(), uniform_loads=None, point_loads=None):
    """nodal_loads: [(tag, Fx, Fy, Mz), ...]
    uniform_loads: {member_tag: (wx, wy)}  (global components, full length)
    point_loads:   {member_tag: (Px, Py, distance_from_i)}  (global components)
    """
    m = Model(ndm=2, ndf=3)
    coords = {}
    for tag, x, y, fx, fy, frz in nodes:
        m.node(tag, float(x), float(y))
        coords[tag] = np.array([float(x), float(y)])
        if fx or fy or frz:
            m.fix(tag, int(fx), int(fy), int(frz))
    for tag, i, j, E, A, I in members:
        m.section("Elastic", tag, E=float(E), A=float(A), Iz=float(I))
        m.element("Frame", tag, i, j, section=tag)

    m.pattern("Plain", 1, "Linear")
    for tag, Fx, Fy, Mz in nodal_loads:
        m.load(tag, float(Fx), float(Fy), float(Mz))
    member_by_tag = {tag: (i, j) for tag, i, j, *_ in members}
    for tag, (wx, wy) in (uniform_loads or {}).items():
        m.ele_load(tag, wx=wx, wy=wy)
    for tag, (Px, Py, dist) in (point_loads or {}).items():
        m.ele_load(tag, Px=Px, Py=Py, distance_from_i=dist)

    m.analyze(1)
    return m, coords, member_by_tag


def _assert_close(actual, expected, rtol=1e-2, atol=1e-3, msg=""):
    scale = max(abs(expected), 1.0)
    assert abs(actual - expected) <= atol + rtol * scale, (
        f"{msg}: got {actual}, expected {expected}")


def _check_statics(m, coords, member_by_tag, nodal_loads, uniform_loads=None,
                    point_loads=None, force_atol=2.0, moment_atol=50.0):
    """Sum of (reactions + applied loads) == 0, force and moment (about
    origin, CCW+)."""
    Fx = Fy = Mz = 0.0
    for tag in coords:
        Rx, Ry, Rm = m.node_reaction(tag)
        Fx += Rx
        Fy += Ry
        Mz += Rm
        x, y = coords[tag]
        Mz += x * Ry - y * Rx
    for tag, fx, fy, mz in nodal_loads:
        x, y = coords[tag]
        Fx += fx
        Fy += fy
        Mz += mz + x * fy - y * fx
    for tag, (wx, wy) in (uniform_loads or {}).items():
        i, j = member_by_tag[tag]
        xi, xj = coords[i], coords[j]
        L = float(np.linalg.norm(xj - xi))
        mid = 0.5 * (xi + xj)
        Fx += wx * L
        Fy += wy * L
        Mz += mid[0] * (wy * L) - mid[1] * (wx * L)
    for tag, (Px, Py, dist) in (point_loads or {}).items():
        i, j = member_by_tag[tag]
        xi, xj = coords[i], coords[j]
        L = float(np.linalg.norm(xj - xi))
        pos = xi + (dist / L) * (xj - xi)
        Fx += Px
        Fy += Py
        Mz += pos[0] * Py - pos[1] * Px
    assert abs(Fx) < force_atol, f"Statics Fx = {Fx}"
    assert abs(Fy) < force_atol, f"Statics Fy = {Fy}"
    assert abs(Mz) < moment_atol, f"Statics Mz = {Mz}"


# ---------------------------------------------------------------------------
# CH16-M1 (problems 16-1, 16-2) — units: N, m
# ---------------------------------------------------------------------------

_E1 = 200_000_000_000.0
M1_NODES = [(1, 0, 4, 1, 1, 1), (2, 4, 4, 0, 0, 0), (3, 4, 0, 1, 1, 1)]
M1_MEMBERS = [(1, 1, 2, _E1, 0.01, 0.0003), (2, 2, 3, _E1, 0.01, 0.0003)]
M1_UNIFORM = {1: (0.0, -12000.0)}
M1_POINT = {2: (-10000.0, 0.0, 2.0)}


class TestCH16M1:
    def setup_method(self):
        self.m, self.coords, self.mbt = _build(
            M1_NODES, M1_MEMBERS, uniform_loads=M1_UNIFORM, point_loads=M1_POINT)

    def test_16_2_free_dofs(self):
        _assert_close(self.m.node_disp(2, 0), -1.357e-05, msg="N2.ux")
        _assert_close(self.m.node_disp(2, 1), -4.315e-05, msg="N2.uy")
        _assert_close(self.m.node_disp(2, 2), 8.612e-05, msg="N2.rz")

    def test_16_2_reactions(self):
        Rx3, Ry3, Mz3 = self.m.node_reaction(3)
        Rx1, Ry1, Mz1 = self.m.node_reaction(1)
        _assert_close(Rx3, 3214.0, msg="N3.Fx")
        _assert_close(Ry3, 21580.0, msg="N3.Fy")
        _assert_close(Mz3, -2722.0, msg="N3.Mz")
        _assert_close(Rx1, 6785.0, msg="N1.Fx")
        _assert_close(Ry1, 26420.0, msg="N1.Fy")
        _assert_close(Mz1, 19550.0, msg="N1.Mz")

    def test_statics(self):
        _check_statics(self.m, self.coords, self.mbt, [],
                       uniform_loads=M1_UNIFORM, point_loads=M1_POINT)


# ---------------------------------------------------------------------------
# CH16-M2 (problems 16-3, 16-4) — units: N, m
# Note (from JSON): printed E = 200 MPa is a typo; published calcs use
# E = 200 GPa, which is what's used here.
# ---------------------------------------------------------------------------

M2_NODES = [(1, 0, 0, 1, 1, 1), (2, 5, 0, 0, 0, 0), (3, 5, -4, 1, 1, 0)]
M2_MEMBERS = [(1, 1, 2, _E1, 0.021, 0.0003), (2, 2, 3, _E1, 0.021, 0.0003)]
M2_NODAL = [(2, 0.0, 0.0, 300000.0)]


class TestCH16M2:
    def setup_method(self):
        self.m, self.coords, self.mbt = _build(
            M2_NODES, M2_MEMBERS, nodal_loads=M2_NODAL)

    def test_16_4_free_dofs(self):
        _assert_close(self.m.node_disp(2, 0), -4.322e-05, msg="N2.ux")
        _assert_close(self.m.node_disp(2, 1), 4.417e-05, msg="N2.uy")
        _assert_close(self.m.node_disp(2, 2), 0.00323787, msg="N2.rz")
        _assert_close(self.m.node_disp(3, 2), -0.00160273, msg="N3.rz")

    def test_16_4_reactions(self):
        Rx3, Ry3, _ = self.m.node_reaction(3)
        Rx1, Ry1, Mz1 = self.m.node_reaction(1)
        _assert_close(Rx3, -36300.0, msg="N3.Fx")
        _assert_close(Ry3, -46400.0, msg="N3.Fy")
        _assert_close(Rx1, 36300.0, msg="N1.Fx")
        _assert_close(Ry1, 46400.0, msg="N1.Fy")
        _assert_close(Mz1, 77100.0, msg="N1.Mz")

    def test_statics(self):
        _check_statics(self.m, self.coords, self.mbt, M2_NODAL)


# ---------------------------------------------------------------------------
# CH16-M3 (problems 16-5, 16-6) — units: N, m
# ---------------------------------------------------------------------------

M3_NODES = [(1, 0, 4, 1, 1, 0), (2, 4, 4, 0, 0, 0), (3, 4, 0, 1, 1, 0)]
M3_MEMBERS = [(1, 1, 2, _E1, 0.015, 0.00035), (2, 2, 3, _E1, 0.015, 0.00035)]
M3_POINT = {1: (0.0, -60000.0, 2.0)}


class TestCH16M3:
    def setup_method(self):
        self.m, self.coords, self.mbt = _build(
            M3_NODES, M3_MEMBERS, point_loads=M3_POINT)

    def test_16_6_free_dofs(self):
        _assert_close(self.m.node_disp(2, 0), -7.3802e-06, msg="N2.ux")
        _assert_close(self.m.node_disp(2, 1), -4.73802e-05, msg="N2.uy")
        _assert_close(self.m.node_disp(2, 2), 0.0004235714, msg="N2.rz")
        _assert_close(self.m.node_disp(3, 2), -0.0002090181, msg="N3.rz")
        _assert_close(self.m.node_disp(1, 2), -0.0002295533, msg="N1.rz")

    def test_16_6_reactions(self):
        Rx3, Ry3, _ = self.m.node_reaction(3)
        Rx1, Ry1, _ = self.m.node_reaction(1)
        _assert_close(Rx3, -5535.0, msg="N3.Fx")
        _assert_close(Ry3, 35535.0, msg="N3.Fy")
        _assert_close(Rx1, 5535.0, msg="N1.Fx")
        _assert_close(Ry1, 24500.0, msg="N1.Fy")

    def test_statics(self):
        _check_statics(self.m, self.coords, self.mbt, [], point_loads=M3_POINT)


# ---------------------------------------------------------------------------
# CH16-M4 (problems 16-7, 16-8) — units: kip, in
# ---------------------------------------------------------------------------

M4_NODES = [(1, 0, 144, 0, 0, 0), (2, 120, 144, 0, 0, 0), (3, 120, 0, 1, 1, 1)]
M4_MEMBERS = [(1, 1, 2, 29000.0, 20.0, 650.0), (2, 2, 3, 29000.0, 20.0, 650.0)]
M4_NODAL = [(1, -4.0, -6.0, 0.0)]


class TestCH16M4:
    def setup_method(self):
        self.m, self.coords, self.mbt = _build(
            M4_NODES, M4_MEMBERS, nodal_loads=M4_NODAL)

    def test_16_8_displacements(self):
        _assert_close(self.m.node_disp(1, 0), -0.608, msg="N1.ux")
        _assert_close(self.m.node_disp(1, 1), -1.12, msg="N1.uy")
        _assert_close(self.m.node_disp(1, 2), 0.01, msg="N1.rz")
        _assert_close(self.m.node_disp(2, 0), -0.6076, msg="N2.ux")
        _assert_close(self.m.node_disp(2, 1), -0.00149, msg="N2.uy")
        _assert_close(self.m.node_disp(2, 2), 0.007705, msg="N2.rz")

    def test_statics(self):
        _check_statics(self.m, self.coords, self.mbt, M4_NODAL,
                       force_atol=0.05, moment_atol=1.0)


# ---------------------------------------------------------------------------
# CH16-M5 (problems 16-9, 16-10) — units: kip, in
# ---------------------------------------------------------------------------

M5_NODES = [(1, 0, 0, 1, 1, 0), (2, 0, 120, 0, 0, 0), (3, 240, 120, 0, 1, 0)]
M5_MEMBERS = [(1, 1, 2, 29000.0, 10.0, 300.0), (2, 2, 3, 29000.0, 10.0, 300.0)]
M5_UNIFORM = {2: (0.0, -1.0/6.0)}


class TestCH16M5:
    def setup_method(self):
        self.m, self.coords, self.mbt = _build(
            M5_NODES, M5_MEMBERS, uniform_loads=M5_UNIFORM)

    def test_16_10_free_dofs(self):
        _assert_close(self.m.node_disp(2, 0), 1.32, msg="N2.ux")
        _assert_close(self.m.node_disp(2, 1), -0.008276, msg="N2.uy")
        _assert_close(self.m.node_disp(2, 2), -0.011, msg="N2.rz")
        _assert_close(self.m.node_disp(3, 2), 0.005552, msg="N3.rz")
        _assert_close(self.m.node_disp(1, 2), -0.011, msg="N1.rz")
        _assert_close(self.m.node_disp(3, 0), 1.32, msg="N3.ux")

    def test_16_10_reactions(self):
        _, Ry3, _ = self.m.node_reaction(3)
        Rx1, Ry1, _ = self.m.node_reaction(1)
        _assert_close(Ry3, 20.0, msg="N3.Fy")
        _assert_close(Rx1, 0.0, atol=0.1, msg="N1.Fx")
        _assert_close(Ry1, 20.0, msg="N1.Fy")

    def test_statics(self):
        _check_statics(self.m, self.coords, self.mbt, [],
                       uniform_loads=M5_UNIFORM, force_atol=0.1, moment_atol=2.0)


# ---------------------------------------------------------------------------
# CH16-M6 (problems 16-11, 16-12) — units: kip, in
# ---------------------------------------------------------------------------

M6_NODES = [(1, 0, 0, 1, 1, 0), (2, 288, 0, 0, 0, 0), (3, 288, 192, 1, 1, 0)]
M6_MEMBERS = [(1, 1, 2, 29000.0, 20.0, 700.0), (2, 2, 3, 29000.0, 20.0, 700.0)]
M6_POINT = {1: (0.0, -20.0, 144.0)}


class TestCH16M6:
    def setup_method(self):
        self.m, self.coords, self.mbt = _build(
            M6_NODES, M6_MEMBERS, point_loads=M6_POINT)

    def test_16_12_free_dofs(self):
        _assert_close(self.m.node_disp(2, 0), 0.001668, msg="N2.ux")
        _assert_close(self.m.node_disp(2, 1), -0.004052, msg="N2.uy")
        _assert_close(self.m.node_disp(2, 2), 0.002043, msg="N2.rz")
        _assert_close(self.m.node_disp(3, 2), -0.001008, msg="N3.rz")
        _assert_close(self.m.node_disp(1, 2), -0.001042, msg="N1.rz")

    def test_16_12_reactions(self):
        Rx3, Ry3, _ = self.m.node_reaction(3)
        Rx1, Ry1, _ = self.m.node_reaction(1)
        _assert_close(Rx3, 3.36, msg="N3.Fx")
        _assert_close(Ry3, 12.2, msg="N3.Fy")
        _assert_close(Rx1, -3.36, msg="N1.Fx")
        _assert_close(Ry1, 7.76, msg="N1.Fy")

    def test_statics(self):
        _check_statics(self.m, self.coords, self.mbt, [],
                       point_loads=M6_POINT, force_atol=0.1, moment_atol=2.0)
