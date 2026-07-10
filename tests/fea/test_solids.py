"""
Phase 2 step 6 — Hex8 + Tet4 solid elements (docs/fea_phase2_plan.md).

Covers the plan's verification list:
- b_matrix_3d kernel unit-tested directly (linear field -> exact Voigt strain)
- 3D PATCH TESTS: distorted 2x2x2 Hex8 patch and a 48-tet Kuhn assembly,
  linear displacement field prescribed on every exterior node reproduces
  the field at the interior node and CONSTANT stress everywhere
- rigid-body modes = 6 for a single unconstrained Hex8 and Tet4
- FD consistent tangent (with element loads applied); K symmetric
- uniaxial bar of hexes == PL/AE exactly (nodal load AND face pressure)
- body-force reactions == total weight (Hex8 and Tet4)
- Hex8 cantilever convergence study vs beam theory (+ shear deformation)
- Tet4 documented-stiff vs Hex8 on the same cantilever (the plan's
  "documented-stiff behavior" gate — constant-strain tets lock in bending)
- mesher box_grid integration via Model.mesh_box (+ quality gate)
- error paths: wrong material formulation, inverted node order, bad face
  index, empty solid ele_load
"""
import numpy as np
import pytest

from src.fea import Model
from src.fea.elements.isoparam import b_matrix_3d, shape_h8
from src.fea.elements.solid import _HEX_FACES, _TET_FACES, Hex8, Tet4
from src.fea.loads.pattern import SolidBodyLoad, SolidFaceLoad
from src.fea.materials.nd import ElasticIsotropic
from src.fea.mesh import MeshQualityError, box_grid
from src.fea.testing import check_element_tangent

E_KSI, NU = 29000.0, 0.3   # imperial closed-form tests: kip, in, ksi

# Linear test field u = A @ x + c and its engineering Voigt strain.
_A = np.array([[2.0, 1.0, 0.5],
               [0.8, -1.5, 0.6],
               [0.3, 0.4, 1.2]]) * 1e-3
_C = np.array([1.0, -2.0, 3.0]) * 1e-3
_EPS = np.array([_A[0, 0], _A[1, 1], _A[2, 2],
                 _A[0, 1] + _A[1, 0], _A[1, 2] + _A[2, 1],
                 _A[2, 0] + _A[0, 2]])


def _field(x):
    return _A @ np.asarray(x) + _C


def _solid_model():
    m = Model(ndm=3, ndf=3)
    m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=NU,
                  formulation="solid3d")
    return m


# Kuhn 6-tet decomposition of one hex (local H8 corner indices); every
# tet contains the main diagonal 0-6.
_KUHN = ((0, 1, 2, 6), (0, 2, 3, 6), (0, 3, 7, 6),
         (0, 7, 4, 6), (0, 4, 5, 6), (0, 5, 1, 6))


def _tet_conn(conn, coords_of):
    """Orientation-fix a tet connectivity: swap the last two nodes if the
    signed volume is negative (Kuhn tets alternate handedness)."""
    p = [np.asarray(coords_of(t), dtype=float) for t in conn]
    six_v = float(np.dot(np.cross(p[1] - p[0], p[2] - p[0]), p[3] - p[0]))
    return conn if six_v > 0.0 else (conn[0], conn[1], conn[3], conn[2])


# ---------------------------------------------------------------------------
# kernels
# ---------------------------------------------------------------------------

def test_b_matrix_3d_reproduces_linear_field_strain():
    coords = np.array([[0.0, 0.0, 0.0], [2.1, 0.1, -0.2], [2.3, 1.9, 0.1],
                       [-0.1, 2.2, 0.2], [0.2, -0.1, 1.8], [2.0, 0.2, 2.1],
                       [1.9, 2.1, 2.2], [0.1, 1.8, 1.9]])
    u = np.concatenate([_field(x) for x in coords])
    from src.fea.elements.isoparam import jacobian
    for xi, eta, ze in ((0.0, 0.0, 0.0), (0.5, -0.3, 0.7), (-0.9, 0.9, -0.5)):
        _, dN = shape_h8(xi, eta, ze)
        _, detJ, dN_dx = jacobian(coords, dN)
        assert detJ > 0.0
        B = b_matrix_3d(dN_dx)
        assert B.shape == (6, 24)
        assert np.allclose(B @ u, _EPS, rtol=0, atol=1e-15)


def test_hex_and_tet_face_tables_point_outward():
    """On the parent cube / canonical tet, each face's tangent cross
    product must point away from the element centroid."""
    from src.fea.elements.isoparam import H8_CORNERS
    for fn in _HEX_FACES:
        fc = H8_CORNERS[list(fn)]
        t1 = fc[1] - fc[0]
        t2 = fc[3] - fc[0]
        n = np.cross(t1, t2)
        centroid = fc.mean(axis=0)      # centroid of face; cube centre is 0
        assert np.dot(n, centroid) > 0.0
    tet = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    cen = tet.mean(axis=0)
    for fn in _TET_FACES:
        fc = tet[list(fn)]
        n = np.cross(fc[1] - fc[0], fc[2] - fc[0])
        assert np.dot(n, fc.mean(axis=0) - cen) > 0.0


# ---------------------------------------------------------------------------
# patch tests (Tier-2 gate)
# ---------------------------------------------------------------------------

def test_hex8_patch_test_distorted():
    """2x2x2 hex patch on [0,2]^3 with the interior node AND several face
    nodes perturbed: linear field on the 26 exterior nodes reproduces the
    field at the interior node and constant stress at every Gauss point
    and corner of all 8 elements, to machine precision."""
    mesh = box_grid(2.0, 2.0, 2.0, 2, 2, 2)
    coords = {t: np.array(c, dtype=float) for t, c in mesh.nodes.items()}
    interior = 14                     # node (1,1,1): 1 + 1 + 3*1 + 9*1
    assert np.allclose(coords[interior], (1.0, 1.0, 1.0))
    coords[interior] += (0.13, -0.09, 0.11)
    # distort mid-face/mid-edge exterior nodes IN their boundary planes
    coords[5] += (0.08, -0.06, 0.0)      # (1,1,0) on zmin
    coords[11] += (0.0, -0.07, 0.09)     # (1,0,1) on ymin
    coords[13] += (0.0, 0.06, -0.08)     # (0,1,1) on xmin
    coords[17] += (0.0, 0.05, 0.07)      # (2,1,1) on xmax
    coords[23] += (-0.06, 0.0, 0.0)      # (1,1,2) on zmax

    m = _solid_model()
    for t, c in coords.items():
        m.node(t, *c)
    for e, conn in mesh.elements.items():
        m.element("Hex8", e, *conn, mat=1)
    for t, c in coords.items():
        if t == interior:
            continue
        u = _field(c)
        for d in range(3):
            m.sp(t, d, u[d])
    m.analyze(1)

    assert np.allclose(m.node_disp(interior), _field(coords[interior]),
                       rtol=0, atol=1e-12)
    D = ElasticIsotropic(9, E=E_KSI, nu=NU, formulation="solid3d").get_tangent()
    sig_expect = D @ _EPS
    for e in mesh.elements:
        assert np.allclose(m.ele_response(e, "stress_gp"),
                           sig_expect[None, :], rtol=1e-9)
        assert np.allclose(m.ele_response(e, "stress_nodes"),
                           sig_expect[None, :], rtol=1e-9)


def test_tet4_patch_test_assembly():
    """2x2x2 cube grid, each cube split into 6 Kuhn tets (48 tets total,
    interior node free): the linear field and constant stress must be
    exact — the constant-strain tet passes the patch test by construction,
    this guards the B-matrix/assembly plumbing."""
    mesh = box_grid(2.0, 2.0, 2.0, 2, 2, 2)
    coords = {t: np.array(c, dtype=float) for t, c in mesh.nodes.items()}
    interior = 14
    coords[interior] += (0.11, 0.07, -0.09)

    m = _solid_model()
    for t, c in coords.items():
        m.node(t, *c)
    e = 1
    vol = 0.0
    for conn in mesh.elements.values():
        for loc in _KUHN:
            tconn = _tet_conn(tuple(conn[i] for i in loc),
                              lambda t: coords[t])
            m.element("Tet4", e, *tconn, mat=1)
            vol += m.domain.get_element(e).volume
            e += 1
    assert np.isclose(vol, 8.0, rtol=1e-12)   # tets tile the box exactly

    for t, c in coords.items():
        if t == interior:
            continue
        u = _field(c)
        for d in range(3):
            m.sp(t, d, u[d])
    m.analyze(1)

    assert np.allclose(m.node_disp(interior), _field(coords[interior]),
                       rtol=0, atol=1e-12)
    D = ElasticIsotropic(9, E=E_KSI, nu=NU, formulation="solid3d").get_tangent()
    sig_expect = D @ _EPS
    for tag in range(1, e):
        assert np.allclose(m.ele_response(tag, "stress_gp"),
                           sig_expect[None, :], rtol=1e-9)
        assert np.allclose(m.ele_response(tag, "stress_nodes"),
                           sig_expect[None, :], rtol=1e-9)


# ---------------------------------------------------------------------------
# element mechanics
# ---------------------------------------------------------------------------

def _single_distorted_hex():
    m = _solid_model()
    coords = [(0.0, 0.0, 0.0), (2.1, 0.1, -0.2), (2.3, 1.9, 0.1),
              (-0.1, 2.2, 0.2), (0.2, -0.1, 1.8), (2.0, 0.2, 2.1),
              (1.9, 2.1, 2.2), (0.1, 1.8, 1.9)]
    for t, c in enumerate(coords, start=1):
        m.node(t, *c)
    m.element("Hex8", 1, *range(1, 9), mat=1)
    return m, m.domain.get_element(1)


def _single_tet():
    m = _solid_model()
    for t, c in enumerate([(0.0, 0.0, 0.0), (1.1, 0.1, 0.0),
                           (-0.1, 1.2, 0.1), (0.1, 0.2, 0.9)], start=1):
        m.node(t, *c)
    m.element("Tet4", 1, 1, 2, 3, 4, mat=1)
    return m, m.domain.get_element(1)


def test_rigid_body_modes_exactly_six():
    for maker in (_single_distorted_hex, _single_tet):
        _, elem = maker()
        lam = np.linalg.eigvalsh(elem.get_tangent_stiff())
        scale = lam[-1]
        assert int(np.sum(np.abs(lam) < 1e-10 * scale)) == 6, \
            f"{type(elem).__name__}: wrong zero-energy mode count"


def test_tangent_symmetric_and_fd_consistent_with_loads():
    rng = np.random.default_rng(3)
    for maker in (_single_distorted_hex, _single_tet):
        _, elem = maker()
        elem.add_load(SolidBodyLoad(1, bx=0.2, by=-0.5, bz=0.9), 1.3)
        elem.add_load(SolidFaceLoad(1, face=1, tx=0.4, ty=0.1, tz=-0.2,
                                    pressure=2.5), 0.7)
        for node in elem.nodes:
            for d in range(3):
                node.set_trial_disp_component(d, rng.uniform(-1e-3, 1e-3))
        K = elem.get_tangent_stiff()
        assert np.allclose(K, K.T, rtol=1e-12)
        check_element_tangent(elem)


def test_hex8_bar_pl_over_ae_exact():
    """4-hex bar, nu = 0: tip displacement = PL/AE at machine precision,
    reactions balance — via nodal loads AND via outward face pressure."""
    L, bw, hh, P = 10.0, 1.0, 1.0, 5.0
    for how in ("nodal", "pressure"):
        m = Model(ndm=3, ndf=3)
        m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=0.0,
                      formulation="solid3d")
        mesh = m.mesh_box(L, bw, hh, 4, 1, 1, mat=1)
        for t in mesh.groups["xmin"]:
            m.fix(t, 1, 1, 1)
        m.pattern("Plain", 1)
        tip = mesh.groups["xmax"]
        if how == "nodal":
            for t in tip:
                m.load(t, P / len(tip), 0.0, 0.0)
        else:
            # outward (tension) pressure on the last element's xi=+1 face
            last = max(mesh.elements)
            m.ele_load(last, face=3, pressure=-P / (bw * hh))
        m.analyze(1)
        expect = P * L / (E_KSI * bw * hh)
        for t in tip:
            assert np.isclose(m.node_disp(t, 0), expect, rtol=1e-12)
        R = sum(m.node_reaction(t)[0] for t in mesh.groups["xmin"])
        assert np.isclose(R, -P, rtol=1e-12)


def test_body_force_reactions_equal_total_weight():
    gamma = 0.15
    # Hex8 box
    m = _solid_model()
    mesh = m.mesh_box(2.0, 3.0, 4.0, 2, 2, 2, mat=1)
    for t in mesh.groups["zmin"]:
        m.fix(t, 1, 1, 1)
    m.pattern("Plain", 1)
    for e in mesh.elements:
        m.ele_load(e, bz=-gamma)
    m.analyze(1)
    Rz = sum(m.node_reaction(t)[2] for t in mesh.groups["zmin"])
    assert np.isclose(Rz, gamma * 24.0, rtol=1e-12)
    # Tet4: one cube split into 6 Kuhn tets
    m2 = _solid_model()
    mesh2 = box_grid(1.0, 1.0, 1.0, 1, 1, 1)
    for t, c in mesh2.nodes.items():
        m2.node(t, *c)
    for t in mesh2.groups["zmin"]:
        m2.fix(t, 1, 1, 1)
    conn = mesh2.elements[1]
    for e, loc in enumerate(_KUHN, start=1):
        tconn = _tet_conn(tuple(conn[i] for i in loc),
                          lambda t: mesh2.nodes[t])
        m2.element("Tet4", e, *tconn, mat=1)
    m2.pattern("Plain", 1)
    for e in range(1, 7):
        m2.ele_load(e, bz=-gamma)
    m2.analyze(1)
    Rz2 = sum(m2.node_reaction(t)[2] for t in mesh2.groups["zmin"])
    assert np.isclose(Rz2, gamma * 1.0, rtol=1e-12)


# ---------------------------------------------------------------------------
# cantilever: Hex8 convergence + Tet4 documented-stiff
# ---------------------------------------------------------------------------

_CANT = dict(L=48.0, d=12.0, t=1.0, P=4.0)   # in, kips (Quad4 test geometry)


def _cant_reference():
    """Tip deflection: Euler beam + first-order shear correction (the same
    reference the Quad4 convergence test uses)."""
    L, d, t, P = _CANT["L"], _CANT["d"], _CANT["t"], _CANT["P"]
    I = t * d ** 3 / 12.0
    G = E_KSI / (2.0 * (1.0 + NU))
    return P * L ** 3 / (3.0 * E_KSI * I) + 1.2 * P * L / (G * d * t)


def _hex_cantilever_tip(nx, ny):
    """Cantilever slab (1 element through thickness), tip shear as a
    consistent face traction; returns mean tip uy."""
    L, d, t, P = _CANT["L"], _CANT["d"], _CANT["t"], _CANT["P"]
    m = _solid_model()
    mesh = m.mesh_box(L, d, t, nx, ny, 1, mat=1,
                      limits={"max_aspect": 20.0})
    for tn in mesh.groups["xmin"]:
        m.fix(tn, 1, 1, 1)
    m.pattern("Plain", 1)
    tau = P / (d * t)
    for k in range(1):
        for j in range(ny):
            e = 1 + (nx - 1) + nx * (j + ny * k)   # i = nx-1 column
            m.ele_load(e, face=3, ty=-tau)
    m.analyze(1)
    return -np.mean([m.node_disp(tn, 1) for tn in mesh.groups["xmax"]])


def test_hex8_cantilever_converges_to_beam_theory():
    ref = _cant_reference()
    ratios = [_hex_cantilever_tip(nx, ny) / ref
              for nx, ny in ((8, 2), (16, 4), (32, 8))]
    # trilinear hex is over-stiff in bending at coarse mesh and converges
    # monotonically from below (same signature as Quad4)
    assert ratios[0] < ratios[1] < ratios[2]
    assert ratios[0] > 0.75
    assert 0.95 < ratios[2] < 1.02


def _tet_cantilever_tip(nx, ny):
    """Same cantilever, each hex split into 6 Kuhn tets, tip load as equal
    nodal forces (comparison uses identical nodal loading on both models)."""
    L, d, t, P = _CANT["L"], _CANT["d"], _CANT["t"], _CANT["P"]
    mesh = box_grid(L, d, t, nx, ny, 1)
    m = _solid_model()
    for tn, c in mesh.nodes.items():
        m.node(tn, *c)
    e = 1
    for conn in mesh.elements.values():
        for loc in _KUHN:
            tconn = _tet_conn(tuple(conn[i] for i in loc),
                              lambda tg: mesh.nodes[tg])
            m.element("Tet4", e, *tconn, mat=1)
            e += 1
    for tn in mesh.groups["xmin"]:
        m.fix(tn, 1, 1, 1)
    m.pattern("Plain", 1)
    tip = mesh.groups["xmax"]
    for tn in tip:
        m.load(tn, 0.0, -P / len(tip), 0.0)
    m.analyze(1)
    return -np.mean([m.node_disp(tn, 1) for tn in tip])


def _hex_cantilever_tip_nodal(nx, ny):
    L, d, t, P = _CANT["L"], _CANT["d"], _CANT["t"], _CANT["P"]
    m = _solid_model()
    mesh = m.mesh_box(L, d, t, nx, ny, 1, mat=1,
                      limits={"max_aspect": 20.0})
    for tn in mesh.groups["xmin"]:
        m.fix(tn, 1, 1, 1)
    m.pattern("Plain", 1)
    tip = mesh.groups["xmax"]
    for tn in tip:
        m.load(tn, 0.0, -P / len(tip), 0.0)
    m.analyze(1)
    return -np.mean([m.node_disp(tn, 1) for tn in tip])


def test_tet4_documented_stiff_vs_hex8():
    """The plan's 'documented-stiff' gate: on the same 16x4x1 grid with
    identical nodal loads, the constant-strain tet mesh must deflect
    MEANINGFULLY less than the hex mesh (bending locking), while still
    being on the flexible side of nothing (sanity floor)."""
    d_hex = _hex_cantilever_tip_nodal(16, 4)
    d_tet = _tet_cantilever_tip(16, 4)
    assert d_tet < 0.90 * d_hex     # documented over-stiffness
    assert d_tet > 0.30 * d_hex     # but not pathological


# ---------------------------------------------------------------------------
# mesher integration + error paths
# ---------------------------------------------------------------------------

def test_mesh_box_emits_hex8_with_face_groups():
    m = _solid_model()
    mesh = m.mesh_box(2.0, 1.0, 1.0, 4, 2, 2, mat=1)
    assert len(mesh.elements) == 16
    assert len(mesh.nodes) == 5 * 3 * 3
    assert len(mesh.groups["xmin"]) == 9
    assert len(mesh.groups["zmax"]) == 15
    assert isinstance(m.domain.get_element(1), Hex8)


def test_mesh_box_quality_gate_blocks_bad_aspect():
    m = _solid_model()
    with pytest.raises(MeshQualityError):
        m.mesh_box(100.0, 1.0, 1.0, 1, 1, 1, mat=1)   # edge aspect 100
    # gate ran BEFORE emitting: domain untouched
    assert not m.domain.node_tags and not m.domain.element_tags
    m.mesh_box(100.0, 1.0, 1.0, 1, 1, 1, mat=1,
               limits={"max_aspect": 200.0})
    assert len(m.domain.element_tags) == 1


def test_solid_error_paths():
    plane = ElasticIsotropic(1, E=E_KSI, nu=NU)   # 3-component
    with pytest.raises(ValueError, match="solid3d"):
        Hex8(1, range(1, 9), material=plane)
    with pytest.raises(ValueError, match="solid3d"):
        Tet4(1, range(1, 5), material=plane)
    with pytest.raises(ValueError, match="exactly 8"):
        Hex8(1, range(1, 8),
             material=ElasticIsotropic(2, E=1.0, nu=0.0,
                                       formulation="solid3d"))
    # inverted tet raises at set_domain
    m = _solid_model()
    for t, c in enumerate([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
                           (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)], start=1):
        m.node(t, *c)
    with pytest.raises(ValueError, match="non-positive volume"):
        m.element("Tet4", 1, 1, 3, 2, 4, mat=1)
    # bad face index
    _, hexe = _single_distorted_hex()
    with pytest.raises(ValueError, match="face index"):
        hexe.add_load(SolidFaceLoad(1, face=6, pressure=1.0), 1.0)
    _, tete = _single_tet()
    with pytest.raises(ValueError, match="face index"):
        tete.add_load(SolidFaceLoad(1, face=4, pressure=1.0), 1.0)
    # empty solid ele_load via api
    m2, _ = _single_distorted_hex()
    m2.pattern("Plain", 1)
    with pytest.raises(ValueError, match="body"):
        m2.ele_load(1)
    # unknown response
    with pytest.raises(ValueError, match="unknown response"):
        hexe.get_response("bogus")
