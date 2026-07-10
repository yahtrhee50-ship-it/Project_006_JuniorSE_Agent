"""
Phase 2 step 5 — ShellMITC4 flat shell (docs/fea_phase2_plan.md).

Covers the plan's verification list:
- PATCH TESTS (Tier-2 gate): membrane constant stress AND bending
  constant curvature on the 5-element distorted patch
- rigid-body modes: one unconstrained shell has exactly 6 zero
  eigenvalues (no hourglass extras, no spurious drilling zeros — a
  uniform theta_z field alone carries energy, the consistent rigid
  rotation about the normal does not)
- FD consistent tangent (with and without element loads); K symmetric
- consistent pressure/self-weight loads: exact load totals, normal
  direction follows node ordering
- membrane part == Quad4 on the same mesh (drilling penalty made
  negligible), and orientation independence (same model built in the
  x-z plane)
- thin limit: hard-SS square plate vs Kirchhoff 0.00406*q*a^4/D,
  converging with mesh and NOT locking at a/t = 1000 (MITC4's
  raison d'etre), including a randomly distorted mesh
- thick 6 m / t = 0.15 m soft-SS plate vs the SAP2000-validated Mindlin
  number (7.336 mm, plate_ss_check.sdb, session 2026-07-04) in a
  [0.97, 1.03] band, with exact reaction statics
- warped element warns; validation errors raise
"""
import numpy as np
import pytest

from src.fea import Model
from src.fea.loads.pattern import ShellBodyLoad, ShellPressureLoad
from src.fea.materials.nd import ElasticIsotropic
from src.fea.testing import check_element_tangent

E_KSI, NU = 29000.0, 0.3   # imperial closed-form tests: kip, in, ksi


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _shell_model(E=E_KSI, nu=NU):
    m = Model(ndm=3, ndf=6)
    m.nd_material("ElasticIsotropic", 1, E=E, nu=nu,
                  formulation="plane_stress")
    return m


def _plate_grid(m, a, n, t, *, z=0.0, mat=1, drill_alpha=1.0e-3):
    """Uniform n x n ShellMITC4 grid on [0,a]^2 at height z. Returns the
    (i, j) -> node tag map."""
    tag = {}
    k = 1
    for j in range(n + 1):
        for i in range(n + 1):
            m.node(k, a * i / n, a * j / n, z)
            tag[(i, j)] = k
            k += 1
    e = 1
    for j in range(n):
        for i in range(n):
            m.element("ShellMITC4", e, tag[(i, j)], tag[(i + 1, j)],
                      tag[(i + 1, j + 1)], tag[(i, j + 1)], mat=mat, t=t,
                      drill_alpha=drill_alpha)
            e += 1
    return tag


def _fix_hard_ss(m, tag, n):
    """Hard simple support on all four edges: w = 0 everywhere on the
    boundary, plus the rotation the Kirchhoff SS condition implies
    (theta_x = 0 on x = const edges, theta_y = 0 on y = const edges);
    membrane u = v = 0 on the boundary."""
    for (i, j), t in tag.items():
        fx = i in (0, n)
        fy = j in (0, n)
        if fx or fy:
            m.fix(t, 1, 1, 1, 1 if fx else 0, 1 if fy else 0, 0)


def _solve_plate(m, q, *, center):
    m.pattern("Plain", 1, "Linear")
    for e in range(1, len(m.domain._elements) + 1):
        m.ele_load(e, pressure=-q)
    m.analyze(1)
    return -m.node_disp(center, 2)


# ---------------------------------------------------------------------------
# patch tests (Tier-2 gate)
# ---------------------------------------------------------------------------

_PATCH_CORNERS = {1: (0.0, 0.0), 2: (24.0, 0.0), 3: (24.0, 12.0),
                  4: (0.0, 12.0)}
_PATCH_INTERIOR = {5: (4.0, 2.0), 6: (18.0, 3.0), 7: (16.0, 8.0),
                   8: (8.0, 8.0)}
_PATCH_CONN = [(1, 2, 6, 5), (2, 3, 7, 6), (3, 4, 8, 7), (4, 1, 5, 8),
               (5, 6, 7, 8)]


def _patch_model(t=1.0):
    m = _shell_model()
    for tag, (x, y) in {**_PATCH_CORNERS, **_PATCH_INTERIOR}.items():
        m.node(tag, x, y, 0.0)
    for e, conn in enumerate(_PATCH_CONN, start=1):
        m.element("ShellMITC4", e, *conn, mat=1, t=t)
    return m


def _to_local_tensor(elem, s):
    """Rotate GLOBAL in-plane tensor components [sxx, syy, sxy] into the
    element's local axes (forces_gp reports in local axes, local x along
    edge 1-2 — the SAP2000 convention)."""
    Q = elem._R[:2, :2]
    S = np.array([[s[0], s[2]], [s[2], s[1]]])
    Sl = Q @ S @ Q.T
    return np.array([Sl[0, 0], Sl[1, 1], Sl[0, 1]])


def test_membrane_patch_constant_stress():
    """Linear in-plane field on the exterior corners -> exact field at the
    interior nodes and constant membrane forces at every Gauss point.
    Bending DOFs pinned at the corners only; theta_z left free (it must
    settle to the field's constant rotation with zero drilling energy)."""
    a, b = 1e-3, 1e-3   # u = a(x + y/2), v = b(y + x/2)
    t = 1.0
    m = _patch_model(t)
    for tag, (x, y) in _PATCH_CORNERS.items():
        m.sp(tag, 0, a * (x + y / 2.0))
        m.sp(tag, 1, b * (y + x / 2.0))
        m.fix(tag, 0, 0, 1, 1, 1, 0)
    m.analyze(1)

    for tag, (x, y) in _PATCH_INTERIOR.items():
        assert np.isclose(m.node_disp(tag, 0), a * (x + y / 2.0),
                          rtol=0, atol=1e-12)
        assert np.isclose(m.node_disp(tag, 1), b * (y + x / 2.0),
                          rtol=0, atol=1e-12)
    D = ElasticIsotropic(9, E=E_KSI, nu=NU).get_tangent()
    F_global = t * (D @ np.array([a, b, (a + b) / 2.0]))
    for e in range(1, 6):
        f = m.ele_response(e, "forces_gp")
        F_expect = _to_local_tensor(m.domain.get_element(e), F_global)
        assert np.allclose(f[:, 0:3], F_expect[None, :], rtol=1e-9)
        # no bending or shear develops
        assert np.max(np.abs(f[:, 3:8])) < 1e-9 * np.max(np.abs(F_global))


def test_bending_patch_constant_curvature():
    """Constant-curvature field w = (x^2 + x*y + y^2)/2 with the matching
    zero-shear rotations (theta_x = w,y; theta_y = -w,x) prescribed on the
    exterior corners -> exact nodal field at the interior nodes, constant
    moments and zero transverse shear at every Gauss point — the classic
    MITC4 distorted-mesh bending patch test."""
    t = 1.0
    m = _patch_model(t)
    c = 1e-3

    def field(x, y):
        w = 0.5 * c * (x * x + x * y + y * y)
        thx = c * (y + x / 2.0)      # = w,y
        thy = -c * (x + y / 2.0)     # = -w,x
        return w, thx, thy

    for tag, (x, y) in _PATCH_CORNERS.items():
        w, thx, thy = field(x, y)
        m.fix(tag, 1, 1, 0, 0, 0, 1)
        m.sp(tag, 2, w)
        m.sp(tag, 3, thx)
        m.sp(tag, 4, thy)
    m.analyze(1)

    for tag, (x, y) in _PATCH_INTERIOR.items():
        w, thx, thy = field(x, y)
        assert np.isclose(m.node_disp(tag, 2), w, rtol=0, atol=1e-12)
        assert np.isclose(m.node_disp(tag, 3), thx, rtol=0, atol=1e-12)
        assert np.isclose(m.node_disp(tag, 4), thy, rtol=0, atol=1e-12)
    D = ElasticIsotropic(9, E=E_KSI, nu=NU).get_tangent()
    kappa = c * np.array([-1.0, -1.0, -1.0])
    M_global = (t ** 3 / 12.0) * (D @ kappa)
    for e in range(1, 6):
        f = m.ele_response(e, "forces_gp")
        M_expect = _to_local_tensor(m.domain.get_element(e), M_global)
        assert np.allclose(f[:, 3:6], M_expect[None, :], rtol=1e-9)
        assert np.max(np.abs(f[:, 6:8])) < 1e-9 * np.max(np.abs(M_global))
        assert np.max(np.abs(f[:, 0:3])) < 1e-9 * np.max(np.abs(M_global))


# ---------------------------------------------------------------------------
# element mechanics
# ---------------------------------------------------------------------------

def _single_distorted_shell(**props):
    m = _shell_model()
    m.node(1, 0.0, 0.0, 0.0)
    m.node(2, 10.0, 1.0, 0.0)
    m.node(3, 11.0, 9.0, 0.0)
    m.node(4, -1.0, 8.0, 0.0)
    m.element("ShellMITC4", 1, 1, 2, 3, 4, mat=1, t=1.5, **props)
    return m, m.domain.get_element(1)


def test_rigid_body_modes_exactly_six():
    _, elem = _single_distorted_shell()
    lam = np.linalg.eigvalsh(elem.get_tangent_stiff())
    scale = lam[-1]
    assert np.sum(np.abs(lam) < 1e-10 * scale) == 6


def test_drilling_mode_energies():
    """Uniform theta_z alone must carry energy (no spurious drilling
    zeros); the CONSISTENT rigid rotation about the normal (u = -w*y,
    v = w*x, theta_z = w) must not."""
    m, elem = _single_distorted_shell()
    K = elem.get_tangent_stiff()
    scale = float(np.max(np.abs(K)))

    d_spur = np.zeros(24)
    for a in range(4):
        d_spur[6 * a + 5] = 1.0
    assert d_spur @ K @ d_spur > 1e-6 * scale

    d_rbm = np.zeros(24)
    for a, node in enumerate(elem.nodes):
        x, y = node.coords[0], node.coords[1]
        d_rbm[6 * a + 0] = -y
        d_rbm[6 * a + 1] = x
        d_rbm[6 * a + 5] = 1.0
    assert d_rbm @ K @ d_rbm < 1e-12 * scale


def test_tangent_is_fd_derivative_and_symmetric():
    m, elem = _single_distorted_shell()
    rng = np.random.default_rng(11)
    for tag in (1, 2, 3, 4):
        m.domain.get_node(tag).set_trial_disp(rng.uniform(-0.01, 0.01, 6))
    K = elem.get_tangent_stiff()
    assert np.allclose(K, K.T, rtol=1e-12)
    check_element_tangent(elem)
    # still the exact derivative with element loads applied (q0 constant)
    elem.add_load(ShellPressureLoad(1, p=2.0), 1.3)
    elem.add_load(ShellBodyLoad(1, bx=0.1, by=-0.2, bz=-0.5), 0.7)
    check_element_tangent(elem)


def test_pressure_totals_and_normal_follows_ordering():
    """Consistent pressure load sums to p * Area along the element normal;
    reversing the node order flips the normal, hence the load."""
    xy = np.array([[0, 0], [10, 1], [11, 9], [-1, 8]], dtype=float)
    area = 0.5 * abs(sum(xy[i, 0] * xy[(i + 1) % 4, 1]
                         - xy[(i + 1) % 4, 0] * xy[i, 1] for i in range(4)))

    m, elem = _single_distorted_shell()
    elem.add_load(ShellPressureLoad(1, p=3.0), 1.0)
    f = elem.get_resisting_force()          # = -equivalent applied load
    Fz = -sum(f[6 * a + 2] for a in range(4))
    assert np.isclose(Fz, 3.0 * area, rtol=1e-12)

    m2 = _shell_model()
    for tag, (x, y) in zip((1, 2, 3, 4),
                           [(0, 0), (-1, 8), (11, 9), (10, 1)]):
        m2.node(tag, float(x), float(y), 0.0)
    m2.element("ShellMITC4", 1, 1, 2, 3, 4, mat=1, t=1.5)
    e2 = m2.domain.get_element(1)
    e2.add_load(ShellPressureLoad(1, p=3.0), 1.0)
    f2 = e2.get_resisting_force()
    Fz2 = -sum(f2[6 * a + 2] for a in range(4))
    assert np.isclose(Fz2, -3.0 * area, rtol=1e-12)


def test_self_weight_statics():
    """Body force bz = -gamma on a supported plate: sum of vertical
    reactions == gamma * t * a^2 exactly (uniform rectangular mesh)."""
    a, t, n, gamma = 60.0, 0.5, 4, 0.15
    m = _shell_model()
    tag = _plate_grid(m, a, n, t)
    _fix_hard_ss(m, tag, n)
    m.pattern("Plain", 1, "Linear")
    for e in range(1, n * n + 1):
        m.ele_load(e, bz=-gamma)
    m.analyze(1)
    total = sum(m.node_reaction(tg, 2) for tg in tag.values())
    assert np.isclose(total, gamma * t * a * a, rtol=1e-10)


# ---------------------------------------------------------------------------
# membrane part vs Quad4 + orientation independence
# ---------------------------------------------------------------------------

def _membrane_cantilever_shell(plane="xy", drill_alpha=1.0e-6):
    """48 x 12 cantilever strip, 8x2 mesh, tip shear 10 kips split over the
    3 tip nodes, built either in the x-y or the x-z plane. Returns the tip
    in-plane deflection at the mid tip node."""
    L, d, nx, ny, t = 48.0, 12.0, 8, 2, 1.0
    m = _shell_model()
    tag = {}
    k = 1
    for j in range(ny + 1):
        for i in range(nx + 1):
            x, y = L * i / nx, -d / 2 + d * j / ny
            if plane == "xy":
                m.node(k, x, y, 0.0)
            else:
                m.node(k, x, 0.0, y)
            tag[(i, j)] = k
            k += 1
    e = 1
    for j in range(ny):
        for i in range(nx):
            m.element("ShellMITC4", e, tag[(i, j)], tag[(i + 1, j)],
                      tag[(i + 1, j + 1)], tag[(i, j + 1)], mat=1, t=t,
                      drill_alpha=drill_alpha)
            e += 1
    # in-plane = (ux, uy) for xy / (ux, uz) for xz; suppress the
    # out-of-plane plate DOFs everywhere (pure membrane problem)
    for (i, j), tg in tag.items():
        if plane == "xy":
            m.fix(tg, 1 if i == 0 else 0, 1 if i == 0 else 0, 1, 1, 1, 0)
        else:
            m.fix(tg, 1 if i == 0 else 0, 1, 1 if i == 0 else 0, 1, 0, 1)
    m.pattern("Plain", 1, "Linear")
    P = 10.0 / (ny + 1)
    for j in range(ny + 1):
        if plane == "xy":
            m.load(tag[(nx, j)], 0.0, -P, 0.0, 0.0, 0.0, 0.0)
        else:
            m.load(tag[(nx, j)], 0.0, 0.0, -P, 0.0, 0.0, 0.0)
    m.analyze(1)
    dof = 1 if plane == "xy" else 2
    return m.node_disp(tag[(nx, ny // 2)], dof)


def test_membrane_matches_quad4_cantilever():
    """With the drilling penalty made negligible, the shell's membrane
    response must match the plain Quad4 element on the same mesh."""
    L, d, nx, ny, t = 48.0, 12.0, 8, 2, 1.0
    m = Model(ndm=2, ndf=2)
    m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=NU,
                  formulation="plane_stress")
    tag = {}
    k = 1
    for j in range(ny + 1):
        for i in range(nx + 1):
            m.node(k, L * i / nx, -d / 2 + d * j / ny)
            tag[(i, j)] = k
            k += 1
    e = 1
    for j in range(ny):
        for i in range(nx):
            m.element("Quad4", e, tag[(i, j)], tag[(i + 1, j)],
                      tag[(i + 1, j + 1)], tag[(i, j + 1)], mat=1, t=t)
            e += 1
    for j in range(ny + 1):
        m.fix(tag[(0, j)], 1, 1)
    m.pattern("Plain", 1, "Linear")
    P = 10.0 / (ny + 1)
    for j in range(ny + 1):
        m.load(tag[(nx, j)], 0.0, -P)
    m.analyze(1)
    v_quad = m.node_disp(tag[(nx, ny // 2)], 1)

    v_shell = _membrane_cantilever_shell("xy")
    assert np.isclose(v_shell, v_quad, rtol=1e-4)


def test_membrane_orientation_independence():
    """The same cantilever built in the x-z plane gives the same in-plane
    answer — exercises the local-frame transformation."""
    v_xy = _membrane_cantilever_shell("xy")
    v_xz = _membrane_cantilever_shell("xz")
    assert np.isclose(v_xz, v_xy, rtol=1e-10)


# ---------------------------------------------------------------------------
# plate bending: thin limit (Kirchhoff), no locking, thick (SAP2000)
# ---------------------------------------------------------------------------

def _kirchhoff_w(q, a, t, E, nu):
    D = E * t ** 3 / (12.0 * (1.0 - nu ** 2))
    return 0.00406 * q * a ** 4 / D


def test_thin_plate_converges_to_kirchhoff():
    """Hard-SS square plate, uniform load, a/t = 100: center deflection
    converges to 0.00406*q*a^4/D with mesh refinement."""
    a, t, q = 100.0, 1.0, 0.01
    w_ref = _kirchhoff_w(q, a, t, E_KSI, NU)
    ratios = []
    for n in (4, 8, 16):
        m = _shell_model()
        tag = _plate_grid(m, a, n, t)
        _fix_hard_ss(m, tag, n)
        w = _solve_plate(m, q, center=tag[(n // 2, n // 2)])
        ratios.append(w / w_ref)
    errs = [abs(r - 1.0) for r in ratios]
    assert errs[1] < errs[0] and errs[2] < errs[1]   # self-convergence
    assert 0.985 <= ratios[-1] <= 1.015


def test_no_shear_locking_at_extreme_slenderness():
    """a/t = 1000: a locking element collapses (ratio -> ~0); MITC4 must
    stay at the Kirchhoff value."""
    a, t, q, n = 100.0, 0.1, 1e-5, 12
    m = _shell_model()
    tag = _plate_grid(m, a, n, t)
    _fix_hard_ss(m, tag, n)
    w = _solve_plate(m, q, center=tag[(n // 2, n // 2)])
    ratio = w / _kirchhoff_w(q, a, t, E_KSI, NU)
    assert 0.97 <= ratio <= 1.02


def test_thin_plate_distorted_mesh_still_converges():
    """Randomly distorted interior nodes (up to ~25% of the spacing) at
    a/t = 1000 — the MITC4 tying must keep the answer near Kirchhoff."""
    a, t, q, n = 100.0, 0.1, 1e-5, 12
    h = a / n
    rng = np.random.default_rng(42)
    m = _shell_model()
    tag = {}
    k = 1
    for j in range(n + 1):
        for i in range(n + 1):
            x, y = a * i / n, a * j / n
            if 0 < i < n and 0 < j < n:
                x += rng.uniform(-0.25, 0.25) * h
                y += rng.uniform(-0.25, 0.25) * h
            m.node(k, x, y, 0.0)
            tag[(i, j)] = k
            k += 1
    e = 1
    for j in range(n):
        for i in range(n):
            m.element("ShellMITC4", e, tag[(i, j)], tag[(i + 1, j)],
                      tag[(i + 1, j + 1)], tag[(i, j + 1)], mat=1, t=t)
            e += 1
    _fix_hard_ss(m, tag, n)
    w = _solve_plate(m, q, center=tag[(n // 2, n // 2)])
    ratio = w / _kirchhoff_w(q, a, t, E_KSI, NU)
    assert 0.90 <= ratio <= 1.05


def test_thick_plate_matches_sap2000_mindlin():
    """The SAP2000-validated scenario (plate_ss_check.sdb, 18/18 sweep of
    2026-07-04): 6 m simply supported square concrete plate, t = 0.15 m,
    q = 10 kN/m2, E = 4700*sqrt(28) MPa, nu = 0.2, 12x12 mesh, perimeter
    pins (translations only — soft SS, matching the SAP model). SAP2000
    thick-shell center deflection: 7.336 mm. Band [0.97, 1.03] per the
    plan. Also: exact vertical statics (sum R = q*a^2 = 360 kN)."""
    a, t, q, n = 6.0, 0.15, 10.0, 12
    E = 4700.0 * np.sqrt(28.0) * 1000.0     # kPa
    m = _shell_model(E=E, nu=0.2)
    tag = _plate_grid(m, a, n, t)
    for (i, j), tg in tag.items():
        if i in (0, n) or j in (0, n):
            m.fix(tg, 1, 1, 1, 0, 0, 0)
    w = _solve_plate(m, q, center=tag[(n // 2, n // 2)])

    total = sum(m.node_reaction(tg, 2) for tg in tag.values())
    assert np.isclose(total, q * a * a, rtol=1e-10)

    ratio = (w * 1000.0) / 7.336
    assert 0.97 <= ratio <= 1.03, f"w = {w*1000:.3f} mm vs SAP 7.336 mm"


# ---------------------------------------------------------------------------
# geometry checks and validation
# ---------------------------------------------------------------------------

def test_warped_element_warns():
    m = _shell_model()
    m.node(1, 0.0, 0.0, 0.0)
    m.node(2, 10.0, 0.0, 0.0)
    m.node(3, 10.0, 10.0, 4.0)   # lifted corner -> warp ~0.066 > 0.05
    # (the mean-plane projection splits the lift into +/- z offsets)
    m.node(4, 0.0, 10.0, 0.0)
    with pytest.warns(UserWarning, match="warp"):
        m.element("ShellMITC4", 1, 1, 2, 3, 4, mat=1, t=1.0)


def test_validation_errors():
    m = _shell_model()
    m.nd_material("ElasticIsotropic", 2, E=E_KSI, nu=NU,
                  formulation="plane_strain")
    for tag, (x, y) in enumerate([(0, 0), (10, 0), (10, 10), (0, 10)],
                                 start=1):
        m.node(tag, float(x), float(y), 0.0)
    with pytest.raises(ValueError, match="4 nodes"):
        m.element("ShellMITC4", 1, 1, 2, 3, mat=1, t=1.0)
    with pytest.raises(ValueError, match="plane_stress"):
        m.element("ShellMITC4", 1, 1, 2, 3, 4, mat=2, t=1.0)
    with pytest.raises(ValueError, match="thickness"):
        m.element("ShellMITC4", 1, 1, 2, 3, 4, mat=1, t=0.0)
    # degenerate geometry: diagonals parallel
    m.node(11, 0.0, 0.0, 0.0)
    m.node(12, 10.0, 0.0, 0.0)
    m.node(13, 20.0, 0.0, 0.0)
    m.node(14, 30.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="degenerate"):
        m.element("ShellMITC4", 2, 11, 12, 13, 14, mat=1, t=1.0)


def test_2d_node_rejected():
    m = Model(ndm=2, ndf=6)
    m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=NU)
    for tag, (x, y) in enumerate([(0, 0), (10, 0), (10, 10), (0, 10)],
                                 start=1):
        m.node(tag, float(x), float(y))
    with pytest.raises(ValueError, match="3D"):
        m.element("ShellMITC4", 1, 1, 2, 3, 4, mat=1, t=1.0)
