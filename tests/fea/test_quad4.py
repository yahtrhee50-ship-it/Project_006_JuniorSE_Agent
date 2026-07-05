"""
Phase 2 step 2 — isoparametric machinery + plane-stress/plane-strain
Quad4 (docs/fea_phase2_plan.md).

Covers the plan's verification list:
- isoparam kernels unit-tested directly: partition of unity, derivative
  sums, identity Jacobian on the parent square, Gauss rule exactness
- NDMaterial D-matrices (plane stress / plane strain / solid3d)
- PATCH TEST (Tier-2 gate, first appearance): 5-element distorted patch
  under a linear displacement field reproduces constant stress exactly
- rigid-body modes: one unconstrained Q4 has exactly 3 zero eigenvalues
- FD consistent tangent (with and without element loads); K symmetric
- exact constant-stress states: edge-traction tension, body-force weight
- cantilever convergence study vs beam theory + shear deformation
  (documents Q4's well-known bending over-stiffness at coarse mesh)
- NAFEMS LE1 elliptic membrane (path C): sigma_yy at D vs 92.7 MPa,
  hand-built graded mapped mesh (Reference/nafems_le1.json targets;
  measured ratio 1.0037 on the 32x12 mesh used here)
"""
import json
from pathlib import Path

import numpy as np
import pytest

from src.fea import Model
from src.fea.elements.isoparam import (Q4_CORNERS, b_matrix_2d, gauss_1d,
                                       gauss_2d, gauss_3d, jacobian, shape_q4)
from src.fea.loads.pattern import QuadBodyLoad, QuadEdgeLoad
from src.fea.materials.nd import ElasticIsotropic
from src.fea.testing import check_element_tangent

REPO = Path(__file__).resolve().parents[2]

E_KSI, NU = 29000.0, 0.3   # imperial closed-form tests: kip, in, ksi


# ---------------------------------------------------------------------------
# isoparametric machinery
# ---------------------------------------------------------------------------

def test_shape_q4_partition_of_unity():
    rng = np.random.default_rng(7)
    for xi, eta in rng.uniform(-1.0, 1.0, size=(20, 2)):
        N, dN = shape_q4(xi, eta)
        assert np.isclose(N.sum(), 1.0, atol=1e-14)
        assert np.allclose(dN.sum(axis=0), 0.0, atol=1e-14)


def test_shape_q4_kronecker_delta_at_corners():
    for a, (xi, eta) in enumerate(Q4_CORNERS):
        N, _ = shape_q4(xi, eta)
        expect = np.zeros(4)
        expect[a] = 1.0
        assert np.allclose(N, expect, atol=1e-15)


def test_jacobian_of_parent_square_is_identity():
    N, dN = shape_q4(0.3, -0.6)
    J, detJ, dN_dx = jacobian(Q4_CORNERS, dN)
    assert np.allclose(J, np.eye(2), atol=1e-15)
    assert np.isclose(detJ, 1.0, atol=1e-15)
    assert np.allclose(dN_dx, dN, atol=1e-15)


def test_gauss_rules_weights_and_exactness():
    for n, total in ((1, 2.0), (2, 2.0), (3, 2.0)):
        p, w = gauss_1d(n)
        assert np.isclose(w.sum(), total)
    # n-point Gauss integrates degree 2n-1 exactly: check x^2 and x^4
    p, w = gauss_1d(2)
    assert np.isclose((w * p**2).sum(), 2.0 / 3.0, atol=1e-14)
    p, w = gauss_1d(3)
    assert np.isclose((w * p**4).sum(), 2.0 / 5.0, atol=1e-14)
    pts, wts = gauss_2d(2)
    assert pts.shape == (4, 2) and np.isclose(wts.sum(), 4.0)
    # exact for xi^2 * eta^2 over the parent square: (2/3)^2
    assert np.isclose((wts * pts[:, 0]**2 * pts[:, 1]**2).sum(), 4.0 / 9.0,
                      atol=1e-14)
    pts3, wts3 = gauss_3d(2)
    assert pts3.shape == (8, 3) and np.isclose(wts3.sum(), 8.0)
    with pytest.raises(ValueError):
        gauss_1d(4)


def test_b_matrix_layout():
    dN_dx = np.array([[1.0, 2.0], [3.0, 4.0]])
    B = b_matrix_2d(dN_dx)
    assert B.shape == (3, 4)
    # node 0: eps_xx <- u0 * dN0/dx, eps_yy <- v0 * dN0/dy, gamma both
    assert B[0, 0] == 1.0 and B[1, 1] == 2.0
    assert B[2, 0] == 2.0 and B[2, 1] == 1.0


# ---------------------------------------------------------------------------
# NDMaterial
# ---------------------------------------------------------------------------

def test_elastic_isotropic_plane_stress_D():
    m = ElasticIsotropic(1, E=E_KSI, nu=NU, formulation="plane_stress")
    c = E_KSI / (1 - NU**2)
    D_expect = c * np.array([[1, NU, 0], [NU, 1, 0], [0, 0, (1 - NU) / 2]])
    assert np.allclose(m.get_tangent(), D_expect)
    eps = np.array([1e-3, -2e-4, 5e-4])
    m.set_trial_strain(eps)
    assert np.allclose(m.get_stress(), D_expect @ eps)


def test_elastic_isotropic_plane_strain_and_solid3d():
    ps = ElasticIsotropic(1, E=E_KSI, nu=NU, formulation="plane_strain")
    c = E_KSI / ((1 + NU) * (1 - 2 * NU))
    assert np.isclose(ps.get_tangent()[0, 0], c * (1 - NU))
    assert np.isclose(ps.get_tangent()[2, 2], c * (1 - 2 * NU) / 2)
    s3 = ElasticIsotropic(2, E=E_KSI, nu=NU, formulation="solid3d")
    D = s3.get_tangent()
    assert D.shape == (6, 6)
    assert np.allclose(D, D.T)
    assert np.all(np.linalg.eigvalsh(D) > 0.0)
    # uniaxial strain: sig_xx = c(1-nu)*eps, sig_yy = c*nu*eps
    s3.set_trial_strain([1e-3, 0, 0, 0, 0, 0])
    sig = s3.get_stress()
    assert np.isclose(sig[0], c * (1 - NU) * 1e-3)
    assert np.isclose(sig[1], c * NU * 1e-3)
    with pytest.raises(ValueError):
        ElasticIsotropic(3, E=E_KSI, nu=NU, formulation="axisym")


def test_elastic_isotropic_copy_is_independent():
    m = ElasticIsotropic(1, E=E_KSI, nu=NU)
    m2 = m.copy()
    m.set_trial_strain([1e-3, 0.0, 0.0])
    assert np.allclose(m2.get_stress(), 0.0)
    assert np.allclose(m2.get_tangent(), m.get_tangent())


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _rect_mesh_model(L, d, nx, ny, t=1.0, y0=None):
    """Structured quad grid on [0,L] x [y0, y0+d]; returns (model, tag)."""
    if y0 is None:
        y0 = -d / 2.0
    m = Model(ndm=2, ndf=2)
    m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=NU,
                  formulation="plane_stress")
    tag = {}
    k = 1
    for j in range(ny + 1):
        for i in range(nx + 1):
            m.node(k, L * i / nx, y0 + d * j / ny)
            tag[(i, j)] = k
            k += 1
    e = 1
    for j in range(ny):
        for i in range(nx):
            m.element("Quad4", e, tag[(i, j)], tag[(i + 1, j)],
                      tag[(i + 1, j + 1)], tag[(i, j + 1)], mat=1, t=t)
            e += 1
    return m, tag


# ---------------------------------------------------------------------------
# THE PATCH TEST (Tier-2 gate)
# ---------------------------------------------------------------------------

def test_patch_test_constant_stress():
    """Classic 5-element distorted patch (MacNeal geometry x100, imperial):
    prescribing a linear displacement field on the 4 exterior corners must
    reproduce the field at the interior nodes and CONSTANT stress at every
    Gauss point of every element, to machine precision."""
    a, b = 1e-3, 1e-3   # u = a(x + y/2), v = b(y + x/2)
    corners = {1: (0.0, 0.0), 2: (24.0, 0.0), 3: (24.0, 12.0), 4: (0.0, 12.0)}
    interior = {5: (4.0, 2.0), 6: (18.0, 3.0), 7: (16.0, 8.0), 8: (8.0, 8.0)}

    m = Model(ndm=2, ndf=2)
    m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=NU,
                  formulation="plane_stress")
    for tag, (x, y) in {**corners, **interior}.items():
        m.node(tag, x, y)
    for e, conn in enumerate([(1, 2, 6, 5), (2, 3, 7, 6), (3, 4, 8, 7),
                              (4, 1, 5, 8), (5, 6, 7, 8)], start=1):
        m.element("Quad4", e, *conn, mat=1, t=1.0)
    for tag, (x, y) in corners.items():
        m.sp(tag, 0, a * (x + y / 2.0))
        m.sp(tag, 1, b * (y + x / 2.0))
    m.analyze(1)

    for tag, (x, y) in interior.items():
        assert np.isclose(m.node_disp(tag, 0), a * (x + y / 2.0),
                          rtol=0, atol=1e-12)
        assert np.isclose(m.node_disp(tag, 1), b * (y + x / 2.0),
                          rtol=0, atol=1e-12)
    # eps = [a, b, a/2 + b/2]; sigma constant everywhere
    D = ElasticIsotropic(9, E=E_KSI, nu=NU).get_tangent()
    sig_expect = D @ np.array([a, b, (a + b) / 2.0])
    for e in range(1, 6):
        sig_gp = m.ele_response(e, "stress_gp")
        sig_nd = m.ele_response(e, "stress_nodes")
        assert np.allclose(sig_gp, sig_expect[None, :], rtol=1e-10)
        assert np.allclose(sig_nd, sig_expect[None, :], rtol=1e-10)


# ---------------------------------------------------------------------------
# element mechanics
# ---------------------------------------------------------------------------

def _single_distorted_quad():
    m = Model(ndm=2, ndf=2)
    m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=NU)
    m.node(1, 0.0, 0.0)
    m.node(2, 10.0, 1.0)
    m.node(3, 11.0, 9.0)
    m.node(4, -1.0, 8.0)
    m.element("Quad4", 1, 1, 2, 3, 4, mat=1, t=1.5)
    return m, m.domain.get_element(1)


def test_rigid_body_modes_exactly_three():
    _, elem = _single_distorted_quad()
    lam = np.linalg.eigvalsh(elem.get_tangent_stiff())
    scale = lam[-1]
    assert np.sum(np.abs(lam) < 1e-10 * scale) == 3


def test_tangent_is_fd_derivative_and_symmetric():
    m, elem = _single_distorted_quad()
    rng = np.random.default_rng(11)
    for tag in (1, 2, 3, 4):
        m.domain.get_node(tag).set_trial_disp(rng.uniform(-0.01, 0.01, 2))
    K = elem.get_tangent_stiff()
    assert np.allclose(K, K.T, rtol=1e-12)
    check_element_tangent(elem)
    # still the exact derivative with element loads applied (q0 is constant)
    elem.add_load(QuadBodyLoad(1, bx=0.2, by=-0.5), 1.3)
    elem.add_load(QuadEdgeLoad(1, edge=2, tx=1.0, pressure=2.0), 0.7)
    check_element_tangent(elem)


def test_clockwise_numbering_raises():
    m = Model(ndm=2, ndf=2)
    m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=NU)
    for tag, (x, y) in enumerate([(0, 0), (0, 10), (10, 10), (10, 0)], start=1):
        m.node(tag, float(x), float(y))
    with pytest.raises(ValueError, match="clockwise|inverted"):
        m.element("Quad4", 1, 1, 2, 3, 4, mat=1, t=1.0)


def test_uniaxial_tension_exact():
    """2x2 patch, uniform traction tx on the right edge: homogeneous plane
    stress state — sigma_xx = t_applied, u = sig*x/E, v = -nu*sig*y/E,
    machine precision (constant-stress states are exact for Q4)."""
    Lx, dy, t, sig = 40.0, 20.0, 2.0, 1.25
    m, tag = _rect_mesh_model(Lx, dy, 2, 2, t=t, y0=0.0)
    for j in range(3):
        m.fix(tag[(0, j)], 1, 0)
    m.fix(tag[(0, 0)], 0, 1)          # already ux-fixed; pin uy at one corner
    m.pattern("Plain", 1, "Linear")
    for j in range(2):
        m.ele_load(j * 2 + 2, edge=1, tx=sig)   # elements (i=1, j): tags 2, 4
    m.analyze(1)
    for (i, j), ntag in tag.items():
        x, y = Lx * i / 2, dy * j / 2
        assert np.isclose(m.node_disp(ntag, 0), sig * x / E_KSI, atol=1e-14)
        assert np.isclose(m.node_disp(ntag, 1), -NU * sig * y / E_KSI,
                          atol=1e-14)
    for e in (1, 2, 3, 4):
        assert np.allclose(m.ele_response(e, "stress_gp"),
                           np.array([sig, 0.0, 0.0])[None, :], atol=1e-12)


def test_outward_pressure_equals_tension_traction():
    """pressure = -p on the right edge of a square == tx = +p (sign
    convention: positive pressure pushes INWARD)."""
    def solve(**eload):
        m, tag = _rect_mesh_model(10.0, 10.0, 1, 1, t=1.0, y0=0.0)
        m.fix(tag[(0, 0)], 1, 1)
        m.fix(tag[(0, 1)], 1, 0)
        m.pattern("Plain", 1, "Linear")
        m.ele_load(1, edge=1, **eload)
        m.analyze(1)
        return m.node_disp(tag[(1, 0)], 0)
    u_trac = solve(tx=3.0)
    u_pres = solve(pressure=-3.0)
    assert np.isclose(u_trac, u_pres, rtol=1e-14)
    assert u_trac > 0.0   # tension pulls the free edge outward


def test_body_force_total_weight():
    """Self-weight body force: sum of vertical reactions == unit weight x
    volume, exactly (consistent nodal loads + statics)."""
    Lx, dy, t, gamma = 30.0, 12.0, 2.0, 0.49 / 1728.0   # kcf -> kip/in^3
    m, tag = _rect_mesh_model(Lx, dy, 3, 2, t=t, y0=0.0)
    for j in range(3):
        m.fix(tag[(0, j)], 1, 1)
    m.pattern("Plain", 1, "Linear")
    for e in range(1, 7):
        m.ele_load(e, by=-gamma)
    m.analyze(1)
    W = gamma * Lx * dy * t
    total = sum(m.node_reaction(tag[(0, j)], 1) for j in range(3))
    assert np.isclose(total, W, rtol=1e-12)


# ---------------------------------------------------------------------------
# cantilever convergence study (Tier-6 first appearance)
# ---------------------------------------------------------------------------

def test_cantilever_convergence_to_beam_theory():
    """Plane-stress cantilever (48x12x1 in, E=29000 ksi, nu=0.3, 40 kip
    tip shear as constant edge traction) vs beam theory including shear
    deformation (delta = PL^3/3EI + 1.2PL/GA). Fully-integrated Q4 is
    known to be over-stiff in bending at coarse mesh — the study must
    converge monotonically from below (measured ratios 0.879 / 0.961 /
    0.986 / 0.992)."""
    L, d, t, P = 48.0, 12.0, 1.0, 40.0
    I = t * d**3 / 12.0
    G = E_KSI / (2.0 * (1.0 + NU))
    delta = P * L**3 / (3.0 * E_KSI * I) + 1.2 * P * L / (G * d * t)

    ratios = []
    for nx, ny in [(8, 2), (16, 4), (32, 8), (64, 16)]:
        m, tag = _rect_mesh_model(L, d, nx, ny, t=t)
        for j in range(ny + 1):
            m.fix(tag[(0, j)], 1, 1)
        m.pattern("Plain", 1, "Linear")
        ty = P / (d * t)
        for j in range(ny):
            m.ele_load(j * nx + nx, edge=1, ty=ty)
        m.analyze(1)
        ratios.append(m.node_disp(tag[(nx, ny // 2)], 1) / delta)

    assert all(r2 > r1 for r1, r2 in zip(ratios, ratios[1:]))   # monotone
    assert 0.85 < ratios[0] < 0.95    # documented coarse-mesh stiffness
    assert 0.985 < ratios[-1] < 1.0   # converged from below


# ---------------------------------------------------------------------------
# NAFEMS LE1 — elliptic membrane (path C)
# ---------------------------------------------------------------------------

def test_nafems_le1_elliptic_membrane():
    """Hand-built graded mapped mesh (the Step 3 transfinite mesher will
    replace this construction): 32 circumferential x 12 radial Q4s, graded
    toward point D. sigma_yy extrapolated to node D vs the published
    92.7 MPa (measured ratio 1.0037 at this density; band +/-2%)."""
    ref = json.loads((REPO / "Reference" / "nafems_le1.json").read_text())
    ai, bi = ref["geometry"]["inner_ellipse_semi_axes"]
    ao, bo = ref["geometry"]["outer_ellipse_semi_axes"]
    t = ref["geometry"]["thickness"]
    E = ref["material"]["E_MPa"]
    nu = ref["material"]["nu"]
    target = ref["target"]["value_MPa"]
    load = 10.0                      # MPa, outward tension on the outer edge
    n_th, n_r, p, q = 32, 12, 2.5, 2.0

    m = Model(ndm=2, ndf=2)
    m.nd_material("ElasticIsotropic", 1, E=E, nu=nu,
                  formulation="plane_stress")
    tag = {}
    k = 1
    for i in range(n_th + 1):
        th = (np.pi / 2.0) * (i / n_th) ** p     # graded toward theta=0 (D)
        inner = np.array([ai * np.cos(th), bi * np.sin(th)])
        outer = np.array([ao * np.cos(th), bo * np.sin(th)])
        for j in range(n_r + 1):
            s = (j / n_r) ** q                   # graded toward the inner edge
            x, y = (1.0 - s) * inner + s * outer
            m.node(k, x, y)
            tag[(i, j)] = k
            k += 1
    e = 1
    for i in range(n_th):
        for j in range(n_r):
            m.element("Quad4", e, tag[(i, j)], tag[(i, j + 1)],
                      tag[(i + 1, j + 1)], tag[(i + 1, j)], mat=1, t=t)
            e += 1
    for j in range(n_r + 1):
        m.fix(tag[(0, j)], 0, 1)        # y = 0 symmetry edge: uy = 0
        m.fix(tag[(n_th, j)], 1, 0)     # x = 0 symmetry edge: ux = 0
    m.pattern("Plain", 1, "Linear")
    for i in range(n_th):
        # outermost ring: local edge 1 runs radially-outer between the two
        # theta stations; negative pressure = outward tension
        m.ele_load(i * n_r + n_r, edge=1, pressure=-load)
    m.analyze(1)

    # point D = (ai, 0) = local node 0 of element 1
    syy_D = m.ele_response(1, "stress_nodes")[0, 1]
    assert target * 0.98 < syy_D < target * 1.02
