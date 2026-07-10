"""
Phase 2 step 3 — structured conforming mesher + quality gates + edge
stitching (docs/fea_phase2_plan.md).

Covers the plan's verification list:
- conformity: every shared edge/face is node-identical; deterministic
  re-run gives identical numbering (rect, transfinite, box)
- transfinite patch on straight edges == rect_grid to machine precision;
  corner-mismatch and bad-grading inputs raise
- quality gate rejects an inverted quad (negative Jacobian), a sliver
  (angle), an over-stretched quad (aspect), a warped 3D quad, and an
  inverted hex — naming the element and metric; Model.mesh_* hooks refuse
  to emit a failing mesh (domain stays empty)
- H8 trilinear shapes: partition of unity, Kronecker delta, identity
  Jacobian on the parent cube
- edge stitching: fine patch tied to coarse patch passes the constant-
  stress patch test across the hanging-node interface; graded-mesh
  cantilever stays within tolerance of the conforming answer; statics
- NAFEMS LE1 upgraded from the step-2 hand mesh to the mapped
  (transfinite) mesher, with a mesh-convergence sequence
- wall archetype: base statics for lateral + gravity loading
"""
import json
from pathlib import Path

import numpy as np
import pytest

from src.fea import Model
from src.fea.archetypes import wall
from src.fea.elements.isoparam import H8_CORNERS, jacobian, shape_h8
from src.fea.mesh import (Mesh, MeshQualityError, box_grid, check_quality,
                          line_mesh, rect_grid, stitch_edge,
                          transfinite_patch)

REPO = Path(__file__).resolve().parents[2]

E_KSI, NU = 29000.0, 0.3   # imperial closed-form tests: kip, in, ksi


# ---------------------------------------------------------------------------
# H8 shape functions (added to isoparam for the hex Jacobian gate; Step 6
# solids consume them)
# ---------------------------------------------------------------------------

def test_shape_h8_partition_of_unity_and_kronecker():
    rng = np.random.default_rng(3)
    for xi, eta, ze in rng.uniform(-1.0, 1.0, size=(20, 3)):
        N, dN = shape_h8(xi, eta, ze)
        assert np.isclose(N.sum(), 1.0, atol=1e-14)
        assert np.allclose(dN.sum(axis=0), 0.0, atol=1e-14)
    for a, (xi, eta, ze) in enumerate(H8_CORNERS):
        N, _ = shape_h8(xi, eta, ze)
        expect = np.zeros(8)
        expect[a] = 1.0
        assert np.allclose(N, expect, atol=1e-15)


def test_shape_h8_identity_jacobian_on_parent_cube():
    _, dN = shape_h8(0.2, -0.5, 0.7)
    J, detJ, dN_dx = jacobian(H8_CORNERS, dN)
    assert np.allclose(J, np.eye(3), atol=1e-15)
    assert np.isclose(detJ, 1.0, atol=1e-15)
    assert np.allclose(dN_dx, dN, atol=1e-15)


# ---------------------------------------------------------------------------
# generators: determinism + conformity
# ---------------------------------------------------------------------------

def _edge_counts(mesh):
    counts = {}
    for conn in mesh.elements.values():
        n = len(conn)
        for a in range(n):
            key = frozenset((conn[a], conn[(a + 1) % n]))
            counts[key] = counts.get(key, 0) + 1
    return counts


def test_line_mesh_groups_and_determinism():
    m1 = line_mesh((0.0, 0.0), (30.0, 0.0), 5, first_node=10, first_ele=20)
    m2 = line_mesh((0.0, 0.0), (30.0, 0.0), 5, first_node=10, first_ele=20)
    assert list(m1.nodes.items()) == list(m2.nodes.items())
    assert list(m1.elements.items()) == list(m2.elements.items())
    assert m1.groups["start"] == (10,) and m1.groups["end"] == (15,)
    assert m1.nodes[12] == (12.0, 0.0)
    assert m1.elements[20] == (10, 11)


def test_rect_grid_conformity_and_determinism():
    m1 = rect_grid(40.0, 20.0, 4, 2)
    m2 = rect_grid(40.0, 20.0, 4, 2)
    assert list(m1.nodes.items()) == list(m2.nodes.items())
    assert list(m1.elements.items()) == list(m2.elements.items())
    # conformity: interior edges shared by exactly 2 elements, boundary by 1
    counts = _edge_counts(m1)
    n_boundary = sum(1 for c in counts.values() if c == 1)
    assert set(counts.values()) <= {1, 2}
    assert n_boundary == 2 * (4 + 2)
    # ordered boundary groups + matching traction sides
    assert m1.groups["bottom"] == (1, 2, 3, 4, 5)
    assert m1.groups["top"] == (11, 12, 13, 14, 15)
    assert len(m1.sides["top"]) == 4 and m1.sides["top"][0][1] == 2
    assert len(m1.sides["right"]) == 2 and m1.sides["right"][0][1] == 1
    check_quality(m1)   # a clean grid passes the gate


def test_transfinite_straight_edges_equals_rect_grid():
    Lx, Ly, nx, ny = 36.0, 18.0, 6, 3
    r = rect_grid(Lx, Ly, nx, ny)
    t = transfinite_patch(lambda u: (Lx * u, 0.0),
                          lambda v: (Lx, Ly * v),
                          lambda u: (Lx * u, Ly),
                          lambda v: (0.0, Ly * v), nx, ny)
    assert list(t.elements.items()) == list(r.elements.items())
    for tag in r.nodes:
        assert np.allclose(t.nodes[tag], r.nodes[tag], atol=1e-13)
    t2 = transfinite_patch(lambda u: (Lx * u, 0.0),
                           lambda v: (Lx, Ly * v),
                           lambda u: (Lx * u, Ly),
                           lambda v: (0.0, Ly * v), nx, ny)
    assert list(t.nodes.items()) == list(t2.nodes.items())


def test_transfinite_bad_inputs_raise():
    with pytest.raises(ValueError, match="corner mismatch"):
        transfinite_patch(lambda u: (u, 0.0), lambda v: (1.0, v),
                          lambda u: (u, 1.0), lambda v: (0.5, v), 2, 2)
    good = dict(bottom=lambda u: (u, 0.0), right=lambda v: (1.0, v),
                top=lambda u: (u, 1.0), left=lambda v: (0.0, v))
    with pytest.raises(ValueError, match="grading"):
        transfinite_patch(*good.values(), 2, 2, grade_u=lambda u: u + 0.1)
    with pytest.raises(ValueError, match="strictly increasing"):
        transfinite_patch(*good.values(), 2, 2,
                          grade_v=lambda v: v + 0.5 * np.sin(2 * np.pi * v))


def test_box_grid_conformity_and_determinism():
    m1 = box_grid(3.0, 2.0, 1.0, 3, 2, 1)
    m2 = box_grid(3.0, 2.0, 1.0, 3, 2, 1)
    assert list(m1.nodes.items()) == list(m2.nodes.items())
    assert list(m1.elements.items()) == list(m2.elements.items())
    assert len(m1.nodes) == 4 * 3 * 2 and len(m1.elements) == 6
    # conformity: shared faces node-identical (interior faces appear twice)
    face_counts = {}
    local_faces = ((0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
                   (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
    for conn in m1.elements.values():
        for f in local_faces:
            key = frozenset(conn[a] for a in f)
            face_counts[key] = face_counts.get(key, 0) + 1
    assert set(face_counts.values()) <= {1, 2}
    n_boundary = sum(1 for c in face_counts.values() if c == 1)
    assert n_boundary == 2 * (3 * 2 + 3 * 1 + 2 * 1)
    assert len(m1.groups["zmin"]) == 4 * 3
    check_quality(m1)


# ---------------------------------------------------------------------------
# quality gate
# ---------------------------------------------------------------------------

def _one_quad(coords, ndm=2):
    return Mesh(ndm=ndm, kind="quad",
                nodes={i + 1: tuple(c) for i, c in enumerate(coords)},
                elements={7: (1, 2, 3, 4)})


def test_gate_rejects_inverted_quad():
    # clockwise numbering
    with pytest.raises(MeshQualityError, match="element 7.*jacobian"):
        check_quality(_one_quad([(0, 0), (0, 10), (10, 10), (10, 0)]))
    # bowtie (self-intersecting): detJ changes sign inside
    with pytest.raises(MeshQualityError, match="jacobian"):
        check_quality(_one_quad([(0, 0), (10, 0), (0, 10), (10, 10)]))


def test_gate_rejects_aspect_and_sliver_angle():
    with pytest.raises(MeshQualityError, match="aspect") as ei:
        check_quality(_one_quad([(0, 0), (20, 0), (20, 1), (0, 1)]))
    assert ei.value.ele_tag == 7 and ei.value.value > 10.0
    # sheared parallelogram: corner angle ~18 deg < 20 (aspect only ~2.1)
    with pytest.raises(MeshQualityError, match="min_angle"):
        check_quality(_one_quad([(0, 0), (2, 0), (2.9, 0.3), (0.9, 0.3)]))


def test_gate_rejects_warped_3d_quad_and_reports_worst_metrics():
    warped = _one_quad([(0, 0, 0), (10, 0, 0), (10, 10, 4), (0, 10, 0)],
                       ndm=3)
    with pytest.raises(MeshQualityError, match="warp"):
        check_quality(warped)
    flat = _one_quad([(0, 0, 0), (10, 0, 0), (10, 10, 0), (0, 10, 0)], ndm=3)
    worst = check_quality(flat)
    assert np.isclose(worst["warp"], 0.0, atol=1e-14)
    worst2 = check_quality(rect_grid(30.0, 10.0, 3, 2))
    assert worst2["min_detJ"] > 0.0
    assert np.isclose(worst2["aspect"], 2.0)


def test_gate_rejects_inverted_hex():
    good = box_grid(1.0, 1.0, 1.0, 1, 1, 1)
    conn = good.elements[1]
    bad = Mesh(ndm=3, kind="hex", nodes=dict(good.nodes),
               elements={1: conn[4:] + conn[:4]})   # top/bottom swapped
    with pytest.raises(MeshQualityError, match="jacobian"):
        check_quality(bad)


def test_model_hook_refuses_failing_mesh_and_emits_nothing():
    m = Model(ndm=2, ndf=2)
    m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=NU)
    with pytest.raises(MeshQualityError, match="aspect"):
        m.mesh_rect(100.0, 1.0, 2, 1, mat=1, t=1.0)
    assert m.domain.node_tags == [] and m.domain.element_tags == []
    # same geometry passes with an explicit (relaxed) limit override
    mesh = m.mesh_rect(100.0, 1.0, 2, 1, mat=1, t=1.0,
                       limits={"max_aspect": 100.0})
    assert len(m.domain.element_tags) == 2 and len(mesh.nodes) == 6


def test_mesh_line_hook_cantilever_closed_form():
    L, E, A, I, P = 120.0, 29000.0, 10.0, 100.0, 5.0
    m = Model(ndm=2, ndf=3)
    m.section("Elastic", 1, E=E, A=A, Iz=I)
    mesh = m.mesh_line((0.0, 0.0), (L, 0.0), 8, section=1)
    m.fix(mesh.groups["start"][0], 1, 1, 1)
    m.pattern("Plain", 1, "Linear")
    m.load(mesh.groups["end"][0], 0.0, -P, 0.0)
    m.analyze(1)
    assert np.isclose(m.node_disp(mesh.groups["end"][0], 1),
                      -P * L**3 / (3 * E * I), rtol=1e-12)


# ---------------------------------------------------------------------------
# edge stitching (hanging nodes -> MP constraints)
# ---------------------------------------------------------------------------

def _stitched_two_patch_model():
    """Coarse 1x1 quad on [0,1]^2 + fine 2x2 patch on [1,2]x[0,1], tied
    along x=1 (2 coincident nodes -> equalDOF, 1 hanging node at y=0.5)."""
    m = Model(ndm=2, ndf=2)
    m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=NU,
                  formulation="plane_stress")
    coarse = m.mesh_rect(1.0, 1.0, 1, 1, mat=1, t=1.0)
    fine = m.mesh_rect(1.0, 1.0, 2, 2, mat=1, t=1.0, x0=1.0,
                       first_node=101, first_ele=101)
    n_con = m.stitch_edge(coarse.groups["right"], fine.groups["left"])
    assert n_con == 3
    return m, coarse, fine


def test_stitched_patch_test_constant_stress():
    """Tier-2 gate across a tied interface: a linear displacement field
    prescribed on the (non-slaved) outer boundary must reproduce constant
    stress in every element of both patches, and the hanging node must
    land exactly on the interpolated field."""
    a, b = 1e-3, 1e-3   # u = a(x + y/2), v = b(y + x/2)
    m, coarse, fine = _stitched_two_patch_model()
    slaved = set(fine.groups["left"])
    boundary = (set(coarse.nodes) |
                {t for t in fine.nodes
                 if any(np.isclose(fine.nodes[t][k], lim)
                        for k, lims in ((0, (2.0,)), (1, (0.0, 1.0)))
                        for lim in lims)}) - slaved
    for tag in sorted(boundary):
        x, y = m.domain.get_node(tag).coords
        m.sp(tag, 0, a * (x + y / 2.0))
        m.sp(tag, 1, b * (y + x / 2.0))
    m.analyze(1)

    for tag in list(coarse.nodes) + list(fine.nodes):
        x, y = m.domain.get_node(tag).coords
        assert np.isclose(m.node_disp(tag, 0), a * (x + y / 2.0),
                          rtol=0, atol=1e-12)
        assert np.isclose(m.node_disp(tag, 1), b * (y + x / 2.0),
                          rtol=0, atol=1e-12)
    from src.fea.materials.nd import ElasticIsotropic
    D = ElasticIsotropic(9, E=E_KSI, nu=NU).get_tangent()
    sig_expect = D @ np.array([a, b, (a + b) / 2.0])
    for e in list(coarse.elements) + list(fine.elements):
        assert np.allclose(m.ele_response(e, "stress_gp"),
                           sig_expect[None, :], rtol=1e-10)


def test_stitch_rejects_node_off_the_interface():
    m = Model(ndm=2, ndf=2)
    m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=NU)
    coarse = m.mesh_rect(1.0, 1.0, 1, 1, mat=1, t=1.0)
    m.node(50, 1.3, 0.5)     # not on x=1
    with pytest.raises(ValueError, match="does not lie"):
        m.stitch_edge(coarse.groups["right"], [50])


def test_stitched_cantilever_close_to_conforming():
    """Graded mesh: fine root patch (bending-critical) stitched to a
    coarse tip patch, vs the conforming uniformly-fine mesh. Tip shear via
    edge traction; statics gate on the reactions."""
    L, d, t, P = 48.0, 12.0, 1.0, 40.0

    def tip_defl(build):
        m = Model(ndm=2, ndf=2)
        m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=NU,
                      formulation="plane_stress")
        root, tip = build(m)
        for tag in root.groups["left"]:
            m.fix(tag, 1, 1)
        m.pattern("Plain", 1, "Linear")
        ty = P / (d * t)
        for ele, edge in tip.sides["right"]:
            m.ele_load(ele, edge=edge, ty=ty)
        m.analyze(1)
        # statics: vertical reactions balance the applied shear
        R = sum(m.node_reaction(tag, 1) for tag in root.groups["left"])
        assert np.isclose(R, -P, rtol=1e-10)
        tip_mid = tip.groups["right"][len(tip.groups["right"]) // 2]
        return m.node_disp(tip_mid, 1)

    def stitched(m):
        root = m.mesh_rect(24.0, d, 4, 4, mat=1, t=t)
        tip = m.mesh_rect(24.0, d, 2, 2, mat=1, t=t, x0=24.0,
                          first_node=1001, first_ele=1001)
        n_con = m.stitch_edge(tip.groups["left"], root.groups["right"])
        assert n_con == 5   # 3 coincident + 2 hanging fine nodes
        return root, tip

    def conforming(m):
        mesh = m.mesh_rect(L, d, 8, 4, mat=1, t=t)
        return mesh, mesh

    d_stitched = tip_defl(stitched)
    d_conform = tip_defl(conforming)
    assert d_stitched > 0.0 and d_conform > 0.0   # +y traction lifts the tip
    assert 0.90 < d_stitched / d_conform < 1.005


# ---------------------------------------------------------------------------
# NAFEMS LE1 with the mapped mesher (upgrades the step-2 hand mesh)
# ---------------------------------------------------------------------------

def test_nafems_le1_transfinite_mesh_convergence():
    """LE1 elliptic membrane meshed by transfinite_patch with the same
    grading the hand mesh used (radial q=2 toward the inner edge, theta
    p=2.5 toward point D). sigma_yy at D vs the published 92.7 MPa:
    convergence sequence tightens to within the +/-2% band."""
    ref = json.loads((REPO / "Reference" / "nafems_le1.json").read_text())
    ai, bi = ref["geometry"]["inner_ellipse_semi_axes"]
    ao, bo = ref["geometry"]["outer_ellipse_semi_axes"]
    t = ref["geometry"]["thickness"]
    target = ref["target"]["value_MPa"]

    def solve(n_r, n_th):
        m = Model(ndm=2, ndf=2)
        m.nd_material("ElasticIsotropic", 1, E=ref["material"]["E_MPa"],
                      nu=ref["material"]["nu"], formulation="plane_stress")
        # u = radial (inner -> outer), v = theta (0 at D -> pi/2): CCW quads
        def on_ellipse(a_, b_):
            return lambda v: (a_ * np.cos(v * np.pi / 2.0),
                              b_ * np.sin(v * np.pi / 2.0))
        mesh = m.mesh_transfinite(
            lambda u: ((1 - u) * ai + u * ao, 0.0),          # theta = 0
            on_ellipse(ao, bo),                               # outer edge
            lambda u: (0.0, (1 - u) * bi + u * bo),           # theta = pi/2
            on_ellipse(ai, bi),                               # inner edge
            n_r, n_th, mat=1, t=t, grade_u=lambda u: u**2.0,
            grade_v=lambda v: v**2.5,
            # deliberate override: the strong theta grading this benchmark
            # needs makes the theta=0 column extremely anisotropic (aspect
            # up to ~270 at 12x32) — legitimate for a graded stress-
            # concentration mesh, far past the production default of 10
            limits={"max_aspect": 400.0, "min_angle_deg": 15.0})
        for tag in mesh.groups["bottom"]:
            m.fix(tag, 0, 1)     # x-axis symmetry edge: uy = 0
        for tag in mesh.groups["top"]:
            m.fix(tag, 1, 0)     # y-axis symmetry edge: ux = 0
        m.pattern("Plain", 1, "Linear")
        for ele, edge in mesh.sides["right"]:
            m.ele_load(ele, edge=edge, pressure=-10.0)   # outward tension
        m.analyze(1)
        # point D = (ai, 0) = local node 0 of the first element
        first_ele = next(iter(mesh.elements))
        return m.ele_response(first_ele, "stress_nodes")[0, 1]

    syy = [solve(n_r, n_th) for n_r, n_th in [(3, 8), (6, 16), (12, 32)]]
    # Self-convergence: successive refinements tighten (the published
    # 92.7 is a rounded benchmark value — the coarsest mesh happens to
    # CROSS it, so error-vs-target is not the right monotonicity check;
    # measured sequence converges to ~93.04, +0.37% like the step-2
    # hand mesh).
    assert abs(syy[2] - syy[1]) < abs(syy[1] - syy[0])
    assert abs(syy[-1] / target - 1.0) < 0.02   # finest in the +/-2% band


# ---------------------------------------------------------------------------
# wall archetype (Tier-2)
# ---------------------------------------------------------------------------

def test_wall_archetype_statics():
    """10x8 ft concrete wall, 8 in thick (imperial, inches): total base
    reactions balance the applied lateral shear and self-weight exactly;
    lateral load drifts the top edge in +x."""
    W, H, t = 120.0, 96.0, 8.0
    gamma = 0.150 / 1728.0          # 150 pcf in kip/in^3
    V = 25.0                        # kips, +x at the top
    a = wall(W, H, t, E=3600.0, nu=0.2, nx=6, ny=5,
             gravity_weight=gamma, lateral_top=V)
    a.model.analyze(1)
    Rx = sum(a.model.node_reaction(tag, 0) for tag in a.support_nodes)
    Ry = sum(a.model.node_reaction(tag, 1) for tag in a.support_nodes)
    assert np.isclose(Rx, -V, rtol=1e-12)
    assert np.isclose(Ry, gamma * W * H * t, rtol=1e-12)
    assert a.model.node_disp(a.key_nodes["top_mid"], 0) > 0.0
    assert a.key_nodes["base_left"] != a.key_nodes["base_right"]
    assert a.node_near(W / 2.0, H) == a.key_nodes["top_mid"]
