"""
FEA Phase 2 Step 4 validation: Frame3D (6 DOF/node) element.

Coverage per docs/fea_phase2_plan.md Step 4:
- closed-form cantilevers: PL/AE, PL^3/3EIz (local y), PL^3/3EIy (local z),
  tip torsion TL/GJ — including a vertical (global-Z) member and a
  rotated-model equivalence check on the orientation transform
- element loads (uniform + point) in both bending planes vs closed forms
- grillage (combined out-of-plane bending + torsion) vs unit-load closed form
- releases (3D pin: both bending moments) and offsets (in-plane equivalence
  vs the validated 2D Frame, plus an out-of-plane torsion-lever closed form)
- 2D regression: Hibbeler Ch 16 M1/M4 re-run through Frame3D with
  out-of-plane DOFs fixed == 2D Frame element to machine precision
- FD consistent tangent, rigid-body modes = 6

Closed-form tests use imperial units (kip, in, ksi) per the standing
directive.
"""
import math

import numpy as np
import pytest

from src.fea.api import Model
from src.fea.loads.pattern import FramePointLoad3D, FrameUniformLoad3D
from src.fea.testing import check_element_tangent
from tests.fea.test_ch16_benchmarks import (M1_MEMBERS, M1_NODES, M1_POINT,
                                            M1_UNIFORM, M4_MEMBERS, M4_NODES,
                                            M4_NODAL, _build)

# W12x50-ish imperial section with distinct Iy != Iz to catch axis mix-ups.
E = 29000.0        # ksi
G = 11200.0        # ksi
A = 14.6           # in^2
IZ = 391.0         # in^4
IY = 56.3          # in^4
J = 1.71           # in^4
L = 120.0          # in


def _sec(m, tag=1):
    m.section("Elastic", tag, E=E, A=A, Iz=IZ, Iy=IY, J=J, G=G)


def _cantilever_x(n_ele=1, **ele_kwargs):
    """Cantilever along global X, fixed at node 1, tip node = n_ele + 1."""
    m = Model(ndm=3, ndf=6)
    for k in range(n_ele + 1):
        m.node(k + 1, k * L / n_ele, 0.0, 0.0)
    m.fix(1, 1, 1, 1, 1, 1, 1)
    _sec(m)
    for k in range(n_ele):
        m.element("Frame3D", k + 1, k + 1, k + 2, section=1,
                  vecxz=(0.0, 0.0, 1.0), **ele_kwargs)
    m.pattern("Plain", 1, "Linear")
    return m, n_ele + 1


# -- closed-form cantilevers ------------------------------------------------

def test_tip_axial_PL_AE():
    m, tip = _cantilever_x()
    m.load(tip, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    m.analyze(1)
    assert m.node_disp(tip, 0) == pytest.approx(L / (A * E), rel=1e-12)


def test_tip_shear_y_PL3_3EIz():
    m, tip = _cantilever_x()
    m.load(tip, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0)
    m.analyze(1)
    assert m.node_disp(tip, 1) == pytest.approx(L**3 / (3 * E * IZ), rel=1e-12)
    assert m.node_disp(tip, 5) == pytest.approx(L**2 / (2 * E * IZ), rel=1e-12)


def test_tip_shear_z_PL3_3EIy():
    m, tip = _cantilever_x()
    m.load(tip, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    m.analyze(1)
    assert m.node_disp(tip, 2) == pytest.approx(L**3 / (3 * E * IY), rel=1e-12)
    # theta_y = -dw/dx for right-handed axes: tip rotation is NEGATIVE.
    assert m.node_disp(tip, 4) == pytest.approx(-L**2 / (2 * E * IY), rel=1e-12)


def test_tip_torsion_TL_GJ():
    m, tip = _cantilever_x()
    m.load(tip, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    m.analyze(1)
    assert m.node_disp(tip, 3) == pytest.approx(L / (G * J), rel=1e-12)


def test_vertical_member_orientation():
    """Column along global Z (vecxz = global X): local z -> global X, so a
    global-X tip load bends about the section's local y axis (EIy)."""
    m = Model(ndm=3, ndf=6)
    m.node(1, 0.0, 0.0, 0.0)
    m.node(2, 0.0, 0.0, L)
    m.fix(1, 1, 1, 1, 1, 1, 1)
    _sec(m)
    m.element("Frame3D", 1, 1, 2, section=1, vecxz=(1.0, 0.0, 0.0))
    m.pattern("Plain", 1, "Linear")
    m.load(2, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    m.analyze(1)
    assert m.node_disp(2, 0) == pytest.approx(L**3 / (3 * E * IY), rel=1e-12)
    # Global-Y tip load bends about local z (EIz).
    m2 = Model(ndm=3, ndf=6)
    m2.node(1, 0.0, 0.0, 0.0)
    m2.node(2, 0.0, 0.0, L)
    m2.fix(1, 1, 1, 1, 1, 1, 1)
    _sec(m2)
    m2.element("Frame3D", 1, 1, 2, section=1, vecxz=(1.0, 0.0, 0.0))
    m2.pattern("Plain", 1, "Linear")
    m2.load(2, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0)
    m2.analyze(1)
    assert m2.node_disp(2, 1) == pytest.approx(L**3 / (3 * E * IZ), rel=1e-12)


def test_rotated_model_equivalence():
    """Rotating the whole model (coords, vecxz, loads) by a rigid rotation
    R_g must rotate every displacement/rotation vector by R_g exactly."""
    def build(R_g):
        m = Model(ndm=3, ndf=6)
        for k, xyz in enumerate([(0.0, 0.0, 0.0), (L, 0.0, 0.0)]):
            m.node(k + 1, *(R_g @ np.array(xyz)))
        m.fix(1, 1, 1, 1, 1, 1, 1)
        _sec(m)
        m.element("Frame3D", 1, 1, 2, section=1,
                  vecxz=tuple(R_g @ np.array([0.0, 0.0, 1.0])))
        m.pattern("Plain", 1, "Linear")
        F = R_g @ np.array([0.3, 1.0, -0.7])
        M = R_g @ np.array([0.5, -0.2, 0.4])
        m.load(2, *F, *M)
        m.analyze(1)
        return m.node_disp(2)

    d0 = build(np.eye(3))
    # Rotation: 30 deg about z then 40 deg about x.
    cz, sz = math.cos(math.radians(30)), math.sin(math.radians(30))
    cx, sx = math.cos(math.radians(40)), math.sin(math.radians(40))
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    R_g = Rx @ Rz
    d1 = build(R_g)
    np.testing.assert_allclose(d1[:3], R_g @ d0[:3], rtol=1e-10, atol=1e-15)
    np.testing.assert_allclose(d1[3:], R_g @ d0[3:], rtol=1e-10, atol=1e-15)


# -- element loads ------------------------------------------------------------

def test_uniform_load_both_planes():
    """Cantilever, uniform global -y and -z line loads: tip w = wL^4/8EI,
    tip slope = wL^3/6EI in each plane (2-element mesh; Bernoulli consistent
    loads are nodally exact for uniform loads)."""
    m, tip = _cantilever_x(n_ele=2)
    w = 0.010  # kip/in
    for e in (1, 2):
        m.ele_load(e, wy=-w, wz=-w)
    m.analyze(1)
    assert m.node_disp(tip, 1) == pytest.approx(-w * L**4 / (8 * E * IZ), rel=1e-10)
    assert m.node_disp(tip, 2) == pytest.approx(-w * L**4 / (8 * E * IY), rel=1e-10)
    assert m.node_disp(tip, 5) == pytest.approx(-w * L**3 / (6 * E * IZ), rel=1e-10)
    # theta_y sign flip: sagging in the x-z plane is a POSITIVE y-rotation.
    assert m.node_disp(tip, 4) == pytest.approx(w * L**3 / (6 * E * IY), rel=1e-10)
    # Statics: base reactions carry the full load.
    R = m.node_reaction(1)
    assert R[1] == pytest.approx(w * L, rel=1e-12)
    assert R[2] == pytest.approx(w * L, rel=1e-12)
    assert R[5] == pytest.approx(w * L**2 / 2, rel=1e-12)   # Mz = +wL^2/2
    assert R[4] == pytest.approx(-w * L**2 / 2, rel=1e-12)  # My sign-flipped


def test_point_load_interior_both_planes():
    """Point load at a = 0.4L on a single-element cantilever:
    tip deflection = Pa^2(3L - a)/6EI in each plane."""
    m, tip = _cantilever_x()
    P, a = -2.0, 0.4 * L
    m.ele_load(1, Py=P, Pz=P, distance_from_i=a)
    m.analyze(1)
    d_exact = P * a**2 * (3 * L - a) / (6 * E * IZ)
    assert m.node_disp(tip, 1) == pytest.approx(d_exact, rel=1e-10)
    assert m.node_disp(tip, 2) == pytest.approx(
        P * a**2 * (3 * L - a) / (6 * E * IY), rel=1e-10)
    R = m.node_reaction(1)
    assert R[1] == pytest.approx(-P, rel=1e-12)
    assert R[2] == pytest.approx(-P, rel=1e-12)
    assert R[5] == pytest.approx(-P * a, rel=1e-12)
    assert R[4] == pytest.approx(P * a, rel=1e-12)


# -- grillage (combined bending + torsion) ------------------------------------

def test_grillage_bending_plus_torsion():
    """Horizontal L-shaped grillage, vertical tip load: unit-load method
    gives w_tip = P[(L1^3 + L2^3)/(3 E Iy) + L1 L2^2/(G J)] — out-of-plane
    bending uses EIy (vecxz = global Z), torsion lives in member 1 only."""
    L1, L2, P = 120.0, 72.0, -2.0
    m = Model(ndm=3, ndf=6)
    m.node(1, 0.0, 0.0, 0.0)
    m.node(2, L1, 0.0, 0.0)
    m.node(3, L1, L2, 0.0)
    m.fix(1, 1, 1, 1, 1, 1, 1)
    _sec(m)
    m.element("Frame3D", 1, 1, 2, section=1, vecxz=(0.0, 0.0, 1.0))
    m.element("Frame3D", 2, 2, 3, section=1, vecxz=(0.0, 0.0, 1.0))
    m.pattern("Plain", 1, "Linear")
    m.load(3, 0.0, 0.0, P, 0.0, 0.0, 0.0)
    m.analyze(1)
    w_exact = P * ((L1**3 + L2**3) / (3 * E * IY) + L1 * L2**2 / (G * J))
    assert m.node_disp(3, 2) == pytest.approx(w_exact, rel=1e-12)
    # Member 1 carries constant torsion of magnitude P * L2 (resisting
    # torque at the i end is -P*L2, at the j end +P*L2).
    assert m.ele_response(1, "T1") == pytest.approx(-P * L2, rel=1e-12)
    assert m.ele_response(1, "T2") == pytest.approx(P * L2, rel=1e-12)
    # Base reactions: force -P, moments balance the load at (L1, L2, 0).
    R = m.node_reaction(1)
    assert R[2] == pytest.approx(-P, rel=1e-12)
    assert R[3] == pytest.approx(-P * L2, rel=1e-12)   # Mx = -(y*Fz)
    assert R[4] == pytest.approx(P * L1, rel=1e-12)    # My = +(x*Fz)


# -- releases ------------------------------------------------------------------

def test_releases_both_ends_simple_span():
    """Two-element beam released at both OUTER ends between fully fixed
    supports behaves as a simple span for a load at the interior node, in
    BOTH bending planes: outer end moments are exactly zero and the support
    reactions are the simple-span values. (A released rotational DOF needs
    connectivity — hence the free interior node, same as the 2D gotcha.)"""
    xi = 0.3
    m = Model(ndm=3, ndf=6)
    m.node(1, 0.0, 0.0, 0.0)
    m.node(2, xi * L, 0.0, 0.0)
    m.node(3, L, 0.0, 0.0)
    m.fix(1, 1, 1, 1, 1, 1, 1)
    m.fix(3, 1, 1, 1, 1, 1, 1)
    _sec(m)
    m.element("Frame3D", 1, 1, 2, section=1, vecxz=(0.0, 0.0, 1.0),
              release_i=True)
    m.element("Frame3D", 2, 2, 3, section=1, vecxz=(0.0, 0.0, 1.0),
              release_j=True)
    m.pattern("Plain", 1, "Linear")
    P = -5.0
    m.load(2, 0.0, P, P, 0.0, 0.0, 0.0)
    m.analyze(1)
    for ele, names in ((1, ("My1", "Mz1")), (2, ("My2", "Mz2"))):
        for name in names:
            assert m.ele_response(ele, name) == pytest.approx(0.0, abs=1e-9)
    R1, R3 = m.node_reaction(1), m.node_reaction(3)
    for dof in (1, 2):
        assert R1[dof] == pytest.approx(-P * (1 - xi), rel=1e-12)
        assert R3[dof] == pytest.approx(-P * xi, rel=1e-12)
    # No element end moment -> zero rotational reaction despite fixity.
    for dof in (4, 5):
        assert R1[dof] == pytest.approx(0.0, abs=1e-9)
        assert R3[dof] == pytest.approx(0.0, abs=1e-9)


def test_release_i_propped_cantilever():
    """Two-element beam, release at the pin-support end, fixed far end,
    uniform -y load == propped cantilever: reactions 3wL/8 / 5wL/8 and
    fixed-end moment wL^2/8."""
    m = Model(ndm=3, ndf=6)
    m.node(1, 0.0, 0.0, 0.0)
    m.node(2, L / 2, 0.0, 0.0)
    m.node(3, L, 0.0, 0.0)
    m.fix(1, 1, 1, 1, 1, 1, 1)   # release makes the member end pinned
    m.fix(3, 1, 1, 1, 1, 1, 1)
    _sec(m)
    m.element("Frame3D", 1, 1, 2, section=1, vecxz=(0.0, 0.0, 1.0),
              release_i=True)
    m.element("Frame3D", 2, 2, 3, section=1, vecxz=(0.0, 0.0, 1.0))
    m.pattern("Plain", 1, "Linear")
    w = 0.010
    m.ele_load(1, wy=-w)
    m.ele_load(2, wy=-w)
    m.analyze(1)
    assert m.node_reaction(1, 1) == pytest.approx(3 * w * L / 8, rel=1e-12)
    assert m.node_reaction(3, 1) == pytest.approx(5 * w * L / 8, rel=1e-12)
    assert m.node_reaction(1, 5) == pytest.approx(0.0, abs=1e-9)
    assert m.node_reaction(3, 5) == pytest.approx(-w * L**2 / 8, rel=1e-12)


# -- offsets -------------------------------------------------------------------

def test_offset_in_plane_matches_2d_frame():
    """In-plane (z=0) offset model through Frame3D == the validated 2D
    Frame offset implementation to machine precision."""
    a, b = 6.0, 9.0
    m2 = Model(ndm=2, ndf=3)
    m2.node(1, 0.0, 0.0)
    m2.node(2, L, 0.0)
    m2.fix(1, 1, 1, 1)
    m2.section("Elastic", 1, E=E, A=A, Iz=IZ)
    m2.element("Frame", 1, 1, 2, section=1, offset_j=(a, b))
    m2.pattern("Plain", 1, "Linear")
    m2.load(2, 0.5, 1.0, 0.75)
    m2.analyze(1)

    m3 = Model(ndm=3, ndf=6)
    m3.node(1, 0.0, 0.0, 0.0)
    m3.node(2, L, 0.0, 0.0)
    m3.fix(1, 1, 1, 1, 1, 1, 1)
    m3.fix(2, 0, 0, 1, 1, 1, 0)   # out-of-plane DOFs fixed
    _sec(m3)
    m3.element("Frame3D", 1, 1, 2, section=1, vecxz=(0.0, 0.0, 1.0),
               offset_j=(a, b, 0.0))
    m3.pattern("Plain", 1, "Linear")
    m3.load(2, 0.5, 1.0, 0.0, 0.0, 0.0, 0.75)
    m3.analyze(1)

    for dof2, dof3 in ((0, 0), (1, 1), (2, 5)):
        assert m3.node_disp(2, dof3) == pytest.approx(
            m2.node_disp(2, dof2), rel=1e-12, abs=1e-15)
    R2, R3 = m2.node_reaction(1), m3.node_reaction(1)
    assert R3[0] == pytest.approx(R2[0], rel=1e-12)
    assert R3[1] == pytest.approx(R2[1], rel=1e-12)
    assert R3[5] == pytest.approx(R2[2], rel=1e-12)


def test_offset_out_of_plane_torsion_lever():
    """Member offset e in +y at both ends, tip load Pz at the node: the
    member carries torsion T = Pz*e and the node deflection picks up the
    twist through the rigid link: w = Pz[L^3/(3EIy) + e^2 L/(GJ)]."""
    e = 8.0
    m = Model(ndm=3, ndf=6)
    m.node(1, 0.0, 0.0, 0.0)
    m.node(2, L, 0.0, 0.0)
    m.fix(1, 1, 1, 1, 1, 1, 1)
    _sec(m)
    m.element("Frame3D", 1, 1, 2, section=1, vecxz=(0.0, 0.0, 1.0),
              offset_i=(0.0, e, 0.0), offset_j=(0.0, e, 0.0))
    m.pattern("Plain", 1, "Linear")
    Pz = 1.5
    m.load(2, 0.0, 0.0, Pz, 0.0, 0.0, 0.0)
    m.analyze(1)
    w_exact = Pz * (L**3 / (3 * E * IY) + e**2 * L / (G * J))
    assert m.node_disp(2, 2) == pytest.approx(w_exact, rel=1e-12)
    assert abs(m.ele_response(1, "T1")) == pytest.approx(Pz * e, rel=1e-12)


# -- 2D regression: Hibbeler Ch 16 through Frame3D -----------------------------

def _build_3d(nodes, members, nodal_loads=(), uniform_loads=None,
              point_loads=None):
    """Mirror of test_ch16_benchmarks._build with Frame3D elements in the
    z=0 plane, out-of-plane DOFs (uz, rx, ry) fixed at every node."""
    m = Model(ndm=3, ndf=6)
    for tag, x, y, fx, fy, frz in nodes:
        m.node(tag, float(x), float(y), 0.0)
        m.fix(tag, int(fx), int(fy), 1, 1, 1, int(frz))
    for tag, i, j, Em, Am, Im in members:
        # Iy/J/G are arbitrary positive values: out-of-plane DOFs are all
        # fixed, and the local stiffness blocks are uncoupled.
        m.section("Elastic", tag, E=float(Em), A=float(Am), Iz=float(Im),
                  Iy=float(Im), J=float(Im), G=float(Em) / 2.6)
        m.element("Frame3D", tag, i, j, section=tag, vecxz=(0.0, 0.0, 1.0))
    m.pattern("Plain", 1, "Linear")
    for tag, Fx, Fy, Mz in nodal_loads:
        m.load(tag, float(Fx), float(Fy), 0.0, 0.0, 0.0, float(Mz))
    for tag, (wx, wy) in (uniform_loads or {}).items():
        m.ele_load(tag, wx=wx, wy=wy)
    for tag, (Px, Py, dist) in (point_loads or {}).items():
        m.ele_load(tag, Px=Px, Py=Py, distance_from_i=dist)
    m.analyze(1)
    return m


@pytest.mark.parametrize("nodes,members,nodal,uniform,point", [
    (M1_NODES, M1_MEMBERS, (), M1_UNIFORM, M1_POINT),
    (M4_NODES, M4_MEMBERS, M4_NODAL, None, None),
], ids=["CH16-M1", "CH16-M4"])
def test_ch16_regression_3d_matches_2d(nodes, members, nodal, uniform, point):
    m2, _, _ = _build(nodes, members, nodal_loads=nodal,
                      uniform_loads=uniform, point_loads=point)
    m3 = _build_3d(nodes, members, nodal_loads=nodal,
                   uniform_loads=uniform, point_loads=point)
    for tag, *_ in nodes:
        d2 = m2.node_disp(tag)
        d3 = m3.node_disp(tag)
        np.testing.assert_allclose(
            [d3[0], d3[1], d3[5]], d2, rtol=1e-12, atol=1e-15,
            err_msg=f"node {tag} displacements")
        r2 = m2.node_reaction(tag)
        r3 = m3.node_reaction(tag)
        np.testing.assert_allclose(
            [r3[0], r3[1], r3[5]], r2, rtol=1e-12, atol=1e-9,
            err_msg=f"node {tag} reactions")


# -- consistency gates ----------------------------------------------------------

def _random_trial(m, scale=1e-2, seed=7):
    rng = np.random.default_rng(seed)
    for tag in m.domain.node_tags:
        node = m.domain.get_node(tag)
        node.set_trial_disp(rng.normal(0.0, scale, node.ndf))


def test_fd_tangent_plain():
    m, _ = _cantilever_x()
    _random_trial(m)
    check_element_tangent(m.domain.get_element(1))


def test_fd_tangent_skewed_with_releases_offsets_and_loads():
    m = Model(ndm=3, ndf=6)
    m.node(1, 1.0, 2.0, 3.0)
    m.node(2, 70.0, 55.0, -40.0)
    _sec(m)
    m.element("Frame3D", 1, 1, 2, section=1, vecxz=(0.2, 0.9, 1.0),
              release_i=True, offset_i=(2.0, -3.0, 1.0),
              offset_j=(-1.0, 2.0, 4.0))
    elem = m.domain.get_element(1)
    elem.add_load(FrameUniformLoad3D(1, wx=0.01, wy=-0.02, wz=0.015), 1.0)
    elem.add_load(FramePointLoad3D(1, Px=1.0, Py=-2.0, Pz=0.5,
                                   distance_from_i=30.0), 1.0)
    _random_trial(m, seed=11)
    check_element_tangent(elem)


def test_rigid_body_modes():
    m = Model(ndm=3, ndf=6)
    m.node(1, 0.0, 0.0, 0.0)
    m.node(2, 100.0, 30.0, 20.0)
    _sec(m)
    m.element("Frame3D", 1, 1, 2, section=1, vecxz=(0.0, 0.0, 1.0))
    K = m.domain.get_element(1).get_tangent_stiff()
    np.testing.assert_allclose(K, K.T, rtol=1e-12, atol=1e-9)
    lam = np.linalg.eigvalsh(K)
    tol = 1e-9 * max(abs(lam))
    assert sum(1 for x in lam if abs(x) < tol) == 6


# -- input validation -----------------------------------------------------------

def test_vecxz_required_and_not_parallel():
    m = Model(ndm=3, ndf=6)
    m.node(1, 0.0, 0.0, 0.0)
    m.node(2, L, 0.0, 0.0)
    _sec(m)
    with pytest.raises(ValueError, match="vecxz"):
        m.element("Frame3D", 1, 1, 2, section=1)
    with pytest.raises(ValueError, match="parallel"):
        m.element("Frame3D", 2, 1, 2, section=1, vecxz=(1.0, 0.0, 0.0))


def test_section_needs_3d_properties():
    m = Model(ndm=3, ndf=6)
    m.node(1, 0.0, 0.0, 0.0)
    m.node(2, L, 0.0, 0.0)
    m.section("Elastic", 1, E=E, A=A, Iz=IZ)   # no Iy/J/G
    with pytest.raises(ValueError, match="Iy"):
        m.element("Frame3D", 1, 1, 2, section=1, vecxz=(0.0, 0.0, 1.0))


def test_wz_on_2d_frame_raises():
    m = Model(ndm=2, ndf=3)
    m.node(1, 0.0, 0.0)
    m.node(2, L, 0.0)
    m.fix(1, 1, 1, 1)
    m.section("Elastic", 1, E=E, A=A, Iz=IZ)
    m.element("Frame", 1, 1, 2, section=1)
    m.pattern("Plain", 1, "Linear")
    with pytest.raises(ValueError, match="Frame3D"):
        m.ele_load(1, wz=-0.01)
