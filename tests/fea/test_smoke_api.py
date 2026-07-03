"""Phase 0 deliverable: a 2-node model built through the Tier-1 API runs the
full component stack (handler -> numberer -> SOE -> integrator -> algorithm)
and returns correct displacements, reactions, and element forces."""
import numpy as np
import pytest

from src.fea.api import Model
from src.fea.analysis.static_analysis import StaticAnalysis
from src.fea.recorders.recorder import NodeRecorder

E = 29000.0   # ksi
A = 2.0       # in^2
L = 60.0      # in
P = 10.0      # kip


def _axial_bar(system="Dense"):
    m = Model(ndm=2, ndf=2)
    m.node(1, 0.0, 0.0)
    m.node(2, L, 0.0)
    m.fix(1, 1, 1)
    m.fix(2, 0, 1)   # keep only the axial DOF free
    m.uniaxial_material("Elastic", 1, E=E)
    m.element("Truss", 1, 1, 2, A=A, mat=1)
    m.pattern("Plain", 1, "Linear")
    m.load(2, P, 0.0)
    m.system(system)
    return m


def test_axial_bar_exact_solution():
    m = _axial_bar()
    m.analyze(1)
    assert abs(m.node_disp(2, 0) - P * L / (A * E)) < 1e-14
    assert np.allclose(m.node_reaction(1), [-P, 0.0], atol=1e-12)
    assert abs(m.ele_response(1, "axial") - P) < 1e-12   # tension +


def test_axial_bar_sparse_solver():
    pytest.importorskip("scipy")
    m = _axial_bar(system="Sparse")
    m.analyze(1)
    assert abs(m.node_disp(2, 0) - P * L / (A * E)) < 1e-14


def test_two_load_steps_scale_linearly():
    m = _axial_bar()
    m.analyze(2)   # LoadControl d_lambda=1 -> lambda = 2 after two steps
    assert abs(m.node_disp(2, 0) - 2.0 * P * L / (A * E)) < 1e-13


def test_support_settlement():
    """Two collinear bars; the far support is pushed 0.1 in toward the
    middle node: compatibility gives u_mid = delta/2, both bars carry the
    same axial force, and reactions sum to zero."""
    delta = -0.1
    m = Model(ndm=2, ndf=2)
    m.node(1, 0.0, 0.0)
    m.node(2, 120.0, 0.0)
    m.node(3, 240.0, 0.0)
    m.fix(1, 1, 1)
    m.fix(2, 0, 1)
    m.fix(3, 0, 1)
    m.sp(3, 0, delta)
    m.uniaxial_material("Elastic", 1, E=E)
    m.element("Truss", 1, 1, 2, A=A, mat=1)
    m.element("Truss", 2, 2, 3, A=A, mat=1)
    m.analyze(1)

    assert abs(m.node_disp(2, 0) - delta / 2.0) < 1e-13
    N_expected = A * E / 120.0 * (delta / 2.0)   # compression (negative)
    assert abs(m.ele_response(1, "axial") - N_expected) < 1e-10
    assert abs(m.ele_response(2, "axial") - N_expected) < 1e-10
    Rx_sum = m.node_reaction(1, 0) + m.node_reaction(3, 0)
    assert abs(Rx_sum) < 1e-10


def test_mechanism_raises_clear_error():
    m = Model(ndm=2, ndf=2)
    m.node(1, 0.0, 0.0)
    m.node(2, 60.0, 0.0)
    m.uniaxial_material("Elastic", 1, E=E)
    m.element("Truss", 1, 1, 2, A=A, mat=1)   # no supports at all
    m.pattern("Plain", 1, "Linear")
    m.load(2, P, 0.0)
    with pytest.raises(ValueError, match="[Ss]ingular"):
        m.analyze(1)


def test_node_recorder_captures_history():
    m = _axial_bar()
    rec = NodeRecorder([2], response="disp")
    analysis = StaticAnalysis(m.domain, recorders=[rec])
    analysis.analyze(2)
    assert len(rec.times) == 2
    u1 = P * L / (A * E)
    assert abs(rec.data[0][0] - u1) < 1e-13
    assert abs(rec.data[1][0] - 2.0 * u1) < 1e-13
