"""
Phase 2 Step 7 — solver upgrade (docs/fea_phase2_plan.md).

- storage auto-selection: dense below the threshold, scipy sparse above
- dense vs sparse vs (if installed) CHOLMOD identical results on a shell model
- factorize-once / multi-RHS == repeated dense solves
- analyze_cases (single factorization, one back-substitution per case)
  matches per-case full re-analysis, including settlements and MPs
- perf smoke: a ~10k-DOF structured slab assembles + solves within a
  generous wall-clock bound (only feasible on the sparse path)
"""
import time

import numpy as np
import pytest

from src.fea import combos
from src.fea.api import Model
from src.fea.system.soe import (LinearSOE, SPARSE_AUTO_THRESHOLD,
                                has_cholmod)

scipy = pytest.importorskip("scipy")

E_KSI = 29000.0
NU = 0.3


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _ss_plate(n, *, a=60.0, t=0.6, q=0.02, system="Auto"):
    """Hard simply supported n x n ShellMITC4 plate under uniform pressure.
    Returns (model, center node tag)."""
    m = Model(ndm=3, ndf=6)
    m.system(system)
    m.nd_material("ElasticIsotropic", 1, E=E_KSI, nu=NU,
                  formulation="plane_stress")
    tag = {}
    k = 1
    for j in range(n + 1):
        for i in range(n + 1):
            m.node(k, a * i / n, a * j / n, 0.0)
            tag[(i, j)] = k
            k += 1
    e = 1
    for j in range(n):
        for i in range(n):
            m.element("ShellMITC4", e, tag[(i, j)], tag[(i + 1, j)],
                      tag[(i + 1, j + 1)], tag[(i, j + 1)], mat=1, t=t)
            e += 1
    for (i, j), nt in tag.items():
        fx = i in (0, n)
        fy = j in (0, n)
        if fx or fy:
            m.fix(nt, 1, 1, 1, 1 if fx else 0, 1 if fy else 0, 0)
    m.pattern("Plain", 1, "Linear")
    for et in range(1, e):
        m.ele_load(et, pressure=-q)
    return m, tag[(n // 2, n // 2)]


def _all_disps(m):
    return np.concatenate([m.node_disp(t)
                           for t in sorted(m.domain.node_tags)])


def _all_reactions(m):
    return np.concatenate([m.node_reaction(t)
                           for t in sorted(m.domain.node_tags)])


# ---------------------------------------------------------------------------
# storage selection + factorize-once mechanics (LinearSOE unit level)
# ---------------------------------------------------------------------------

def test_auto_storage_selection():
    soe = LinearSOE()
    soe.set_size(10)
    assert not soe.is_sparse
    soe.set_size(SPARSE_AUTO_THRESHOLD)
    assert soe.is_sparse
    # override: forced dense stays dense at any size; forced sparse at any
    soe_d = LinearSOE(storage="dense")
    soe_d.set_size(SPARSE_AUTO_THRESHOLD + 100)
    assert not soe_d.is_sparse
    soe_s = LinearSOE(storage="sparse")
    soe_s.set_size(4)
    assert soe_s.is_sparse
    # threshold override
    soe_t = LinearSOE(sparse_threshold=5)
    soe_t.set_size(6)
    assert soe_t.is_sparse


def test_factorize_multi_rhs_equals_repeated_solves():
    rng = np.random.default_rng(7)
    n = 40
    M = rng.standard_normal((n, n))
    K = M @ M.T + n * np.eye(n)      # SPD
    ids = np.arange(n)
    for storage in ("dense", "sparse"):
        soe = LinearSOE(storage=storage)
        soe.set_size(n)
        soe.add_matrix(K, ids)
        soe.factorize()
        for seed in range(3):
            b = np.random.default_rng(seed).standard_normal(n)
            x = soe.solve_rhs(b)
            assert np.allclose(x, np.linalg.solve(K, b), rtol=1e-10,
                               atol=1e-12), storage


def test_add_matrix_invalidates_factorization():
    K = np.array([[2.0]])
    soe = LinearSOE(storage="sparse")
    soe.set_size(1)
    soe.add_matrix(K, np.array([0]))
    assert soe.solve_rhs(np.array([4.0]))[0] == pytest.approx(2.0)
    soe.add_matrix(K, np.array([0]))     # K -> 4
    assert soe.solve_rhs(np.array([4.0]))[0] == pytest.approx(1.0)


def test_sparse_singular_raises_mechanism_message():
    soe = LinearSOE(storage="sparse")
    soe.set_size(2)
    soe.add_matrix(np.array([[1.0, 1.0], [1.0, 1.0]]), np.array([0, 1]))
    soe.zero_b()
    with pytest.raises(ValueError, match="mechanism"):
        soe.solve()


# ---------------------------------------------------------------------------
# dense vs sparse vs CHOLMOD identical on a shell model
# ---------------------------------------------------------------------------

def test_dense_sparse_identical_shell():
    # 12x12 grid -> 13^2 * 6 = 1014 DOF, above the auto threshold
    m_d, c = _ss_plate(12, system="Dense")
    m_d.analyze(1)
    m_s, _ = _ss_plate(12, system="Sparse")
    m_s.analyze(1)
    m_a, _ = _ss_plate(12, system="Auto")
    m_a.analyze(1)
    ref = _all_disps(m_d)
    scale = np.max(np.abs(ref))
    assert np.allclose(_all_disps(m_s), ref, atol=1e-9 * scale)
    assert np.allclose(_all_disps(m_a), ref, atol=1e-9 * scale)
    r_ref = _all_reactions(m_d)
    r_scale = np.max(np.abs(r_ref))
    assert np.allclose(_all_reactions(m_s), r_ref, atol=1e-9 * r_scale)
    assert np.allclose(_all_reactions(m_a), r_ref, atol=1e-9 * r_scale)
    # RCM ordering on the sparse path gives the same physics
    m_r, _ = _ss_plate(12, system="Sparse")
    m_r.numberer("RCM")
    m_r.analyze(1)
    assert np.allclose(_all_disps(m_r), ref, atol=1e-9 * scale)


@pytest.mark.skipif(not has_cholmod(), reason="scikit-sparse not installed")
def test_cholmod_identical_shell():
    m_d, _ = _ss_plate(8, system="Dense")
    m_d.analyze(1)
    m_c, _ = _ss_plate(8, system="Cholmod")
    m_c.analyze(1)
    ref = _all_disps(m_d)
    scale = np.max(np.abs(ref))
    assert np.allclose(_all_disps(m_c), ref, atol=1e-9 * scale)


# ---------------------------------------------------------------------------
# analyze_cases: factorize-once path == full re-analysis per case
# ---------------------------------------------------------------------------

def _portal(system="Auto"):
    """Portal frame with two patterns (gravity UDL, wind point) plus a
    support settlement and an equal-DOF MP tie — exercises the prescribed-
    displacement and transformation paths through analyze_cases."""
    m = Model(ndm=2, ndf=3)
    m.system(system)
    H, W = 144.0, 240.0
    m.node(1, 0.0, 0.0)
    m.node(2, 0.0, H)
    m.node(3, W, H)
    m.node(4, W, 0.0)
    m.node(5, W / 2, H)          # midspan node
    m.node(6, W / 2, H)          # duplicate tied by equal_dof
    m.fix(1, 1, 1, 1)
    m.fix(4, 1, 0, 1)
    m.sp(4, 1, -0.25)            # right base settles 1/4 in
    m.section("Elastic", 1, E=E_KSI, A=20.0, Iz=800.0)
    m.element("Frame", 1, 1, 2, section=1)
    m.element("Frame", 2, 2, 5, section=1)
    m.element("Frame", 3, 6, 3, section=1)
    m.element("Frame", 4, 3, 4, section=1)
    m.equal_dof(5, 6, 0, 1, 2)
    m.pattern("Plain", 1, "Linear")
    m.ele_load(2, wy=-0.1)
    m.ele_load(3, wy=-0.1)
    m.pattern("Plain", 2, "Linear")
    m.load(2, 5.0, 0.0, 0.0)
    return m


def test_analyze_cases_matches_individual_analyses():
    # reference: full re-analysis per case (fresh model each time)
    ref = {}
    for name, tag in (("D", 1), ("W", 2)):
        m = _portal()
        m.domain.set_active_patterns([tag])
        m.analyze(1)
        ref[name] = combos.snapshot(m.domain, name)

    m2 = _portal()
    res = m2.analyze_cases({"D": 1, "W": 2})
    assert set(res) == {"D", "W"}
    for name in ("D", "W"):
        for nt, d in ref[name].node_disp.items():
            assert np.allclose(res[name].node_disp[nt], d,
                               rtol=1e-10, atol=1e-12), (name, nt)
        for nt, r in ref[name].reactions.items():
            assert np.allclose(res[name].reactions[nt], r,
                               rtol=1e-10, atol=1e-9), (name, nt)
        for et, f in ref[name].ele_force.items():
            assert np.allclose(res[name].ele_force[et], f,
                               rtol=1e-10, atol=1e-9), (name, et)


def test_analyze_cases_dense_sparse_agree():
    res_d = _portal(system="Dense").analyze_cases({"D": 1, "W": 2})
    res_s = _portal(system="Sparse").analyze_cases({"D": 1, "W": 2})
    for name in ("D", "W"):
        for nt in res_d[name].node_disp:
            assert np.allclose(res_s[name].node_disp[nt],
                               res_d[name].node_disp[nt],
                               rtol=1e-9, atol=1e-12)


def test_analyze_cases_reusable_after_run():
    """The model is left reset; a subsequent plain analyze() still works
    and matches the case result."""
    m = _portal()
    res = m.analyze_cases({"D": 1})
    m.domain.set_active_patterns([1])
    m.analyze(1)
    m.domain.set_active_patterns(None)
    for nt in m.domain.node_tags:
        assert np.allclose(m.node_disp(nt), res["D"].node_disp[nt],
                           rtol=1e-10, atol=1e-12)


# ---------------------------------------------------------------------------
# perf smoke — ~10k DOF slab within a generous bound (sparse path only)
# ---------------------------------------------------------------------------

def test_perf_smoke_10k_dof_slab():
    n = 40                                   # 41^2 * 6 = 10,086 DOF
    a, t, q = 240.0, 2.4, 0.02               # a/t = 100 -> thin plate
    t0 = time.perf_counter()
    m, center = _ss_plate(n, a=a, t=t, q=q)
    m.analyze(1)
    elapsed = time.perf_counter() - t0
    assert elapsed < 60.0, f"10k-DOF slab took {elapsed:.1f}s"
    # sanity: matches the Kirchhoff hard-SS closed form within 2%
    D = E_KSI * t ** 3 / (12.0 * (1 - NU ** 2))
    w_ref = 0.00406 * q * a ** 4 / D
    w = -m.node_disp(center, 2)
    assert w == pytest.approx(w_ref, rel=0.02)
