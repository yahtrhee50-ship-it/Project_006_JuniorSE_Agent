"""Phase 0 Tier-1 tests: UniaxialMaterial contract (Elastic)."""
from src.fea.materials.uniaxial import ElasticMaterial
from src.fea.testing import check_material_tangent


def test_elastic_stress_and_tangent():
    m = ElasticMaterial(1, E=29000.0)
    m.set_trial_strain(0.001)
    assert abs(m.get_stress() - 29.0) < 1e-12
    assert m.get_tangent() == 29000.0


def test_elastic_commit_revert():
    m = ElasticMaterial(1, E=100.0)
    m.set_trial_strain(0.5)
    m.commit_state()
    m.set_trial_strain(2.0)
    m.revert_to_last_commit()
    assert abs(m.get_stress() - 50.0) < 1e-12


def test_elastic_tangent_is_derivative_of_stress():
    m = ElasticMaterial(1, E=29000.0)
    for strain in (-0.002, 0.0, 0.001, 0.03):
        check_material_tangent(m, strain)
