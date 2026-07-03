"""Phase 0 Tier-1 tests: Domain storage + deterministic DOF numbering."""
import numpy as np
import pytest

from src.fea.analysis_model import AnalysisModel
from src.fea.constraints.sp import PlainHandler, SP_Constraint
from src.fea.domain import Domain, Node
from src.fea.numberer import PlainNumberer


def test_add_get_node_by_tag():
    d = Domain()
    d.add_node(Node(7, (1.0, 2.0), ndf=2))
    n = d.get_node(7)
    assert n.tag == 7
    assert np.allclose(n.coords, [1.0, 2.0])
    with pytest.raises(KeyError):
        d.get_node(99)


def test_duplicate_node_tag_rejected():
    d = Domain()
    d.add_node(Node(1, (0.0, 0.0), ndf=2))
    with pytest.raises(ValueError, match="Duplicate node tag"):
        d.add_node(Node(1, (5.0, 0.0), ndf=2))


def test_numbering_is_deterministic_and_tag_ordered():
    """Equation numbers must follow sorted node tags, not insertion order."""
    d = Domain()
    for tag in (5, 1, 3):   # deliberately out of order
        d.add_node(Node(tag, (float(tag), 0.0), ndf=2))
    model = AnalysisModel()
    PlainHandler().handle(d, model, PlainNumberer())
    assert model.num_eq == 6
    assert list(model.eq_numbers(1)) == [0, 1]
    assert list(model.eq_numbers(3)) == [2, 3]
    assert list(model.eq_numbers(5)) == [4, 5]


def test_constrained_dofs_get_negative_numbers():
    d = Domain()
    d.add_node(Node(1, (0.0, 0.0), ndf=2))
    d.add_node(Node(2, (10.0, 0.0), ndf=2))
    d.add_sp_constraint(SP_Constraint(1, 0, 0.0))
    d.add_sp_constraint(SP_Constraint(1, 1, 0.0))
    model = AnalysisModel()
    PlainHandler().handle(d, model, PlainNumberer())
    assert model.num_eq == 2
    assert all(e < 0 for e in model.eq_numbers(1))
    assert list(model.eq_numbers(2)) == [0, 1]


def test_conflicting_sp_constraints_rejected():
    d = Domain()
    d.add_node(Node(1, (0.0, 0.0), ndf=2))
    d.add_sp_constraint(SP_Constraint(1, 0, 0.0))
    d.add_sp_constraint(SP_Constraint(1, 0, 0.5))
    model = AnalysisModel()
    with pytest.raises(ValueError, match="Conflicting"):
        PlainHandler().handle(d, model, PlainNumberer())


def test_sp_constraint_dof_out_of_range_rejected():
    d = Domain()
    d.add_node(Node(1, (0.0, 0.0), ndf=2))
    with pytest.raises(ValueError, match="out of range"):
        d.add_sp_constraint(SP_Constraint(1, 2, 0.0))


def test_node_commit_revert():
    n = Node(1, (0.0, 0.0), ndf=2)
    n.set_trial_disp([0.1, 0.2])
    n.commit_state()
    n.set_trial_disp([9.0, 9.0])
    n.revert_to_last_commit()
    assert np.allclose(n.get_trial_disp(), [0.1, 0.2])
    assert np.allclose(n.get_committed_disp(), [0.1, 0.2])
