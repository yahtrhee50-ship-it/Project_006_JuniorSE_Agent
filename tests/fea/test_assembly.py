"""Phase 0 Tier-1 tests: assembly scatter/gather round-trips, plus the first
Tier-4 regression — identical truss through the new engine and the existing
PlaneTruss solver."""
import numpy as np

from src.calcs.truss_stiffness import PlaneTruss
from src.fea.api import Model
from src.fea.analysis.static_analysis import StaticAnalysis
from src.fea.domain import Domain, Node
from src.fea.elements.truss import Truss
from src.fea.materials.uniaxial import ElasticMaterial
from src.fea.system.soe import LinearSOE

E = 29000.0   # ksi
A = 2.0       # in^2


def test_two_bar_chain_matches_hand_assembly():
    """Collinear 2-bar chain: K_ff at the middle node = k1 + k2, and the
    solved displacement round-trips through scatter/gather exactly."""
    L1, L2, P = 60.0, 40.0, 12.0
    m = Model(ndm=2, ndf=2)
    m.node(1, 0.0, 0.0)
    m.node(2, L1, 0.0)
    m.node(3, L1 + L2, 0.0)
    m.fix(1, 1, 1)
    m.fix(2, 0, 1)
    m.fix(3, 1, 1)
    m.uniaxial_material("Elastic", 1, E=E)
    m.element("Truss", 1, 1, 2, A=A, mat=1)
    m.element("Truss", 2, 2, 3, A=A, mat=1)
    m.pattern("Plain", 1, "Linear")
    m.load(2, P, 0.0)
    m.analyze(1)

    k1, k2 = A * E / L1, A * E / L2
    assert abs(m.node_disp(2, 0) - P / (k1 + k2)) < 1e-12
    # equilibrium: reactions balance the applied load
    Rx = m.node_reaction(1, 0) + m.node_reaction(3, 0)
    assert abs(Rx + P) < 1e-9


def test_assembled_matrix_size_excludes_constrained_dofs():
    d = Domain()
    d.add_node(Node(1, (0.0, 0.0), ndf=2))
    d.add_node(Node(2, (100.0, 0.0), ndf=2))
    elem = Truss(1, (1, 2), A=A, material=ElasticMaterial(1, E=E))
    d.add_element(elem)
    from src.fea.constraints.sp import PlainHandler, SP_Constraint
    for dof in (0, 1):
        d.add_sp_constraint(SP_Constraint(1, dof, 0.0))
    d.add_sp_constraint(SP_Constraint(2, 1, 0.0))

    analysis = StaticAnalysis(d, soe=LinearSOE())
    from src.fea.numberer import PlainNumberer
    analysis.constraint_handler.handle(d, analysis.model, PlainNumberer())
    assert analysis.model.num_eq == 1
    analysis.soe.set_size(analysis.model.num_eq)
    analysis.soe.add_matrix(elem.get_tangent_stiff(),
                            analysis.model.id_for(elem))
    assert analysis.soe.A.shape == (1, 1)
    assert abs(analysis.soe.A[0, 0] - A * E / 100.0) < 1e-12


def test_regression_triangle_truss_vs_planetruss():
    """Same triangular truss through src/fea and the existing PlaneTruss:
    displacements, reactions, and member (tension +) forces must agree to
    machine precision."""
    coords = {1: (0.0, 0.0), 2: (240.0, 0.0), 3: (120.0, 120.0)}
    members = [(1, 1, 2), (2, 1, 3), (3, 2, 3)]
    Fx, Fy = 5.0, -10.0

    # --- existing solver (oracle) ---
    pt = PlaneTruss()
    pt.add_node("1", *coords[1], ux_fixed=True, uy_fixed=True)
    pt.add_node("2", *coords[2], uy_fixed=True)
    pt.add_node("3", *coords[3])
    for tag, i, j in members:
        pt.add_member(str(tag), str(i), str(j), A=A, E=E)
    pt.add_load("3", Fx=Fx, Fy=Fy)
    ref = pt.solve()

    # --- new engine ---
    m = Model(ndm=2, ndf=2)
    for tag, xy in coords.items():
        m.node(tag, *xy)
    m.fix(1, 1, 1)
    m.fix(2, 0, 1)
    m.uniaxial_material("Elastic", 1, E=E)
    for tag, i, j in members:
        m.element("Truss", tag, i, j, A=A, mat=1)
    m.pattern("Plain", 1, "Linear")
    m.load(3, Fx, Fy)
    m.analyze(1)

    for tag in coords:
        assert np.allclose(m.node_disp(tag), ref.displacements[str(tag)],
                           rtol=1e-12, atol=1e-14), f"disp mismatch node {tag}"
    for tag in (1, 2):
        assert np.allclose(m.node_reaction(tag), ref.reactions[str(tag)],
                           rtol=1e-12, atol=1e-12), f"reaction mismatch node {tag}"
    for tag, _, _ in members:
        assert abs(m.ele_response(tag, "axial")
                   - ref.member_forces[str(tag)]) < 1e-10, \
            f"member force mismatch member {tag}"
