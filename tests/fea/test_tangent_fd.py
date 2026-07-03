"""Phase 0 Tier-1 tests: consistent-tangent finite-difference check and
element stiffness sanity (symmetry, rigid-body mode count)."""
import numpy as np

from src.fea.domain import Domain, Node
from src.fea.elements.truss import Truss
from src.fea.materials.uniaxial import ElasticMaterial
from src.fea.testing import check_element_tangent


def _bar_2d(angle_nodes=((0.0, 0.0), (30.0, 40.0))):
    d = Domain()
    d.add_node(Node(1, angle_nodes[0], ndf=2))
    d.add_node(Node(2, angle_nodes[1], ndf=2))
    elem = Truss(10, (1, 2), A=2.5, material=ElasticMaterial(1, E=29000.0))
    d.add_element(elem)
    return d, elem


def test_truss_tangent_matches_fd_at_zero_and_nonzero_state():
    d, elem = _bar_2d()
    check_element_tangent(elem)
    # again at a displaced trial state
    d.get_node(1).set_trial_disp([0.01, -0.02])
    d.get_node(2).set_trial_disp([-0.005, 0.03])
    check_element_tangent(elem)


def test_truss_tangent_matches_fd_3d():
    d = Domain()
    d.add_node(Node(1, (0.0, 0.0, 0.0), ndf=3))
    d.add_node(Node(2, (12.0, 3.0, 4.0), ndf=3))
    elem = Truss(1, (1, 2), A=1.0, material=ElasticMaterial(1, E=200.0e3))
    d.add_element(elem)
    d.get_node(2).set_trial_disp([0.002, -0.001, 0.004])
    check_element_tangent(elem)


def test_truss_stiffness_symmetric():
    _, elem = _bar_2d()
    K = elem.get_tangent_stiff()
    assert np.allclose(K, K.T)


def test_truss_has_exactly_three_rigid_body_modes_2d():
    """An unconstrained 2D axial element has rank 1: exactly 3 zero-energy
    modes out of 4 DOFs (2 translations + 1 rotation... plus the transverse
    slide — for a truss all but the axial stretch mode are zero-energy)."""
    _, elem = _bar_2d()
    K = elem.get_tangent_stiff()
    eigvals = np.linalg.eigvalsh(K)
    scale = np.max(np.abs(eigvals))
    n_zero = int(np.sum(np.abs(eigvals) < 1e-10 * scale))
    assert n_zero == 3
