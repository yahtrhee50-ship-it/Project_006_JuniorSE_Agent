"""
FEA-engine Phase 1: Frame member end moment releases (hinges) and rigid
end offsets.

Closed-form oracles (all exact for linear Euler-Bernoulli, so tight
tolerances):

- Propped cantilever via release: fixed-fixed single element with
  release_i under UDL w must give the propped-cantilever reaction set
  R_pin = 3wL/8, R_fix = 5wL/8, M_fix = wL^2/8, M_pin = 0 (exactly).
- Double release + UDL = simple beam: R = wL/2 each end, both element
  end moments exactly zero.
- Gerber (fixed - internal hinge - roller) is statically determinate, so
  reactions are independent of EI and exact by statics.
- Rigid zone at the fixed end of a cantilever: tip load P deflects only
  the flexible length Lf: v = P*Lf^3/3EI; support moment reaction is
  still P*L (full lever arm through the rigid link).
- Rigid zone at the loaded tip: v_node = P*(Lf^3/3 + b*Lf^2 + b^2*Lf)/EI
  (cantilever tip force + tip moment P*b, plus rigid-link rotation b*th).

Plus FD consistent-tangent and symmetry checks on released/offset
elements (src/fea/testing.py), including with element loads applied.
"""
import numpy as np

from src.fea.api import Model
from src.fea.domain import Domain, Node
from src.fea.elements.frame import Frame
from src.fea.loads.pattern import FramePointLoad, FrameUniformLoad
from src.fea.sections.section import ElasticSection
from src.fea.testing import check_element_tangent

E, A, I = 29000.0, 20.0, 800.0   # ksi, in^2, in^4 (values arbitrary)


def _frame_model(nodes, elements, ndf=3):
    """nodes: [(tag, x, y, fix_ux, fix_uy, fix_rz)]
    elements: [(tag, i, j, kwargs_dict)]"""
    m = Model(ndm=2, ndf=ndf)
    for tag, x, y, fx, fy, frz in nodes:
        m.node(tag, float(x), float(y))
        if fx or fy or frz:
            m.fix(tag, int(fx), int(fy), int(frz))
    m.section("Elastic", 1, E=E, A=A, Iz=I)
    for tag, i, j, kw in elements:
        m.element("Frame", tag, i, j, section=1, **kw)
    m.pattern("Plain", 1, "Linear")
    return m


# ---------------------------------------------------------------------------
# Moment releases
# ---------------------------------------------------------------------------

def test_release_i_udl_matches_propped_cantilever():
    """Two elements with a free midspan node (a fully fixed 2-node model has
    no free DOFs and the engine refuses to run); release_i at the N1 end
    turns the fixed support there into an effective pin."""
    L, w = 120.0, 0.8
    m = _frame_model(
        nodes=[(1, 0.0, 0.0, 1, 1, 1), (2, L / 2.0, 0.0, 0, 0, 0),
               (3, L, 0.0, 1, 1, 1)],
        elements=[(10, 1, 2, {"release_i": True}), (11, 2, 3, {})])
    m.ele_load(10, wy=-w)
    m.ele_load(11, wy=-w)
    m.analyze(1)
    _, R1y, M1 = m.node_reaction(1)
    _, R3y, M3 = m.node_reaction(3)
    assert abs(M1) < 1e-9, "released end must carry zero moment reaction"
    assert np.isclose(R1y, 3.0 * w * L / 8.0, rtol=1e-9)
    assert np.isclose(R3y, 5.0 * w * L / 8.0, rtol=1e-9)
    assert np.isclose(abs(M3), w * L**2 / 8.0, rtol=1e-9)
    # element end moment at the released end is exactly zero too
    assert abs(m.ele_response(10, "M1")) < 1e-9


def test_release_both_udl_matches_simple_beam():
    L, w = 200.0, 0.5
    # rz fixed at both nodes only to keep the (disconnected) rotation DOFs
    # out of the free set — the double release makes them element-inert.
    m = _frame_model(
        nodes=[(1, 0.0, 0.0, 1, 1, 1), (2, L, 0.0, 0, 1, 1)],
        elements=[(10, 1, 2, {"release_i": True, "release_j": True})])
    m.ele_load(10, wy=-w)
    m.analyze(1)
    _, R1y, M1 = m.node_reaction(1)
    _, R2y, M2 = m.node_reaction(2)
    assert np.isclose(R1y, w * L / 2.0, rtol=1e-9)
    assert np.isclose(R2y, w * L / 2.0, rtol=1e-9)
    assert abs(M1) < 1e-9 and abs(M2) < 1e-9
    assert abs(m.ele_response(10, "M1")) < 1e-9
    assert abs(m.ele_response(10, "M2")) < 1e-9


def test_gerber_internal_hinge_reactions_by_statics():
    """Fixed(N1) -- 4 m span -- hinge(N2, unsupported) -- 4 m span -- roller(N3),
    UDL w on both spans. Determinate: R3 = wL2/2 = 20, hinge shear 20,
    R1 = w*L1 + 20 = 60, M1 = w*L1^2/2 + 20*L1 = 160 (independent of EI)."""
    w, L1, L2 = 10.0, 4.0, 4.0
    m = _frame_model(
        nodes=[(1, 0.0, 0.0, 1, 1, 1),
               (2, L1, 0.0, 0, 0, 0),
               (3, L1 + L2, 0.0, 0, 1, 0)],
        elements=[(10, 1, 2, {"release_j": True}),   # hinge at N2
                  (11, 2, 3, {})])
    m.ele_load(10, wy=-w)
    m.ele_load(11, wy=-w)
    m.analyze(1)
    _, R1y, M1 = m.node_reaction(1)
    _, R3y, _ = m.node_reaction(3)
    assert np.isclose(R1y, 60.0, rtol=1e-9)
    assert np.isclose(abs(M1), 160.0, rtol=1e-9)
    assert np.isclose(R3y, 20.0, rtol=1e-9)
    # hinge end of element 10 carries zero moment; element 11 end moment at
    # the hinge node is also zero (simple-span end)
    assert abs(m.ele_response(10, "M2")) < 1e-8
    assert abs(m.ele_response(11, "M1")) < 1e-8


def test_release_point_load_redistributes_consistently():
    """Fixed N1 -- free midspan N2 (nodal load P) -- fixed N3 with release_j
    on the second element = propped cantilever with midspan point load:
    R_fix = 11P/16, R_pin = 5P/16, M_fix = 3PL/16."""
    L, P = 144.0, 12.0
    m = _frame_model(
        nodes=[(1, 0.0, 0.0, 1, 1, 1), (2, L / 2.0, 0.0, 0, 0, 0),
               (3, L, 0.0, 1, 1, 1)],
        elements=[(10, 1, 2, {}), (11, 2, 3, {"release_j": True})])
    m.load(2, 0.0, -P, 0.0)
    m.analyze(1)
    _, R1y, M1 = m.node_reaction(1)
    _, R3y, M3 = m.node_reaction(3)
    assert np.isclose(R1y, 11.0 * P / 16.0, rtol=1e-9)
    assert np.isclose(R3y, 5.0 * P / 16.0, rtol=1e-9)
    assert np.isclose(abs(M1), 3.0 * P * L / 16.0, rtol=1e-9)
    assert abs(M3) < 1e-9


# ---------------------------------------------------------------------------
# Rigid end offsets
# ---------------------------------------------------------------------------

def test_rigid_zone_at_support_stiffens_cantilever():
    L, a, P = 100.0, 20.0, 5.0
    Lf = L - a
    m = _frame_model(
        nodes=[(1, 0.0, 0.0, 1, 1, 1), (2, L, 0.0, 0, 0, 0)],
        elements=[(10, 1, 2, {"offset_i": (a, 0.0)})])
    m.load(2, 0.0, -P, 0.0)
    m.analyze(1)
    v2 = m.node_disp(2, 1)
    th2 = m.node_disp(2, 2)
    assert np.isclose(v2, -P * Lf**3 / (3.0 * E * I), rtol=1e-9)
    assert np.isclose(th2, -P * Lf**2 / (2.0 * E * I), rtol=1e-9)
    # reaction moment still takes the full lever arm through the rigid link
    _, R1y, M1 = m.node_reaction(1)
    assert np.isclose(R1y, P, rtol=1e-9)
    assert np.isclose(abs(M1), P * L, rtol=1e-9)


def test_rigid_zone_at_loaded_tip():
    L, b, P = 100.0, 15.0, 5.0
    Lf = L - b
    m = _frame_model(
        nodes=[(1, 0.0, 0.0, 1, 1, 1), (2, L, 0.0, 0, 0, 0)],
        elements=[(10, 1, 2, {"offset_j": (-b, 0.0)})])
    m.load(2, 0.0, -P, 0.0)
    m.analyze(1)
    v2 = m.node_disp(2, 1)
    v_expected = -P * (Lf**3 / 3.0 + b * Lf**2 + b**2 * Lf) / (E * I)
    assert np.isclose(v2, v_expected, rtol=1e-9)
    _, R1y, M1 = m.node_reaction(1)
    assert np.isclose(R1y, P, rtol=1e-9)
    assert np.isclose(abs(M1), P * L, rtol=1e-9)


def test_zero_offsets_and_no_releases_unchanged():
    """Plain element built through the new kwargs defaults must match the
    closed-form cantilever exactly (regression guard on the W/condense path)."""
    L, P = 100.0, 5.0
    m = _frame_model(
        nodes=[(1, 0.0, 0.0, 1, 1, 1), (2, L, 0.0, 0, 0, 0)],
        elements=[(10, 1, 2, {})])
    m.load(2, 0.0, -P, 0.0)
    m.analyze(1)
    assert np.isclose(m.node_disp(2, 1), -P * L**3 / (3.0 * E * I), rtol=1e-12)


# ---------------------------------------------------------------------------
# Consistent tangent / symmetry on released + offset elements
# ---------------------------------------------------------------------------

def _loose_frame(**kw):
    d = Domain()
    d.add_node(Node(1, (0.0, 0.0), ndf=3))
    d.add_node(Node(2, (30.0, 40.0), ndf=3))
    sec = ElasticSection(1, E=E, A=A, Iz=I)
    elem = Frame(10, (1, 2), section=sec, **kw)
    d.add_element(elem)
    return d, elem


def test_tangent_fd_release_and_offset_with_loads():
    d, elem = _loose_frame(release_i=True, offset_i=(2.0, 1.0),
                           offset_j=(-1.5, 0.5))
    elem.zero_loads()
    elem.add_load(FrameUniformLoad(elem.tag, wy=-0.4), factor=1.0)
    elem.add_load(FramePointLoad(elem.tag, Px=3.0, Py=-8.0,
                                 distance_from_i=elem.L / 3.0), factor=1.0)
    d.get_node(1).set_trial_disp([0.01, -0.02, 0.001])
    d.get_node(2).set_trial_disp([-0.005, 0.03, -0.002])
    check_element_tangent(elem)


def test_released_offset_stiffness_symmetric_and_rank():
    _, elem = _loose_frame(release_j=True, offset_i=(1.0, -2.0))
    K = elem.get_tangent_stiff()
    assert np.allclose(K, K.T)
    # one moment release removes exactly one deformation mode: rank 2
    # (axial + one bending), so 4 zero-energy modes out of 6
    eigvals = np.linalg.eigvalsh(K)
    scale = np.max(np.abs(eigvals))
    assert int(np.sum(np.abs(eigvals) < 1e-8 * scale)) == 4
