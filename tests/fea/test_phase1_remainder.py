"""
Phase 1 remainder: RCM numberer, load cases/combinations, Tier-2 archetypes,
and the elastic section property tool.

All numeric benchmarks are closed-form; inputs are IMPERIAL (kips, inches,
ksi) per the engineer's units directive. The engine itself is unit-agnostic.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from src.fea import Model
from src.fea import combos
from src.fea.archetypes import (cantilever, continuous_beam, portal_frame,
                                simply_supported_beam)
from src.fea.numberer import PlainNumberer, RCMNumberer
from src.fea.sections import properties as sp

E = 29000.0     # ksi
I = 800.0       # in^4
A = 20.0        # in^2
L = 360.0       # in (30 ft)
W_KLF = 2.0
W = W_KLF / 12.0            # kip/in, downward magnitude
P = 25.0                    # kips


# ---------------------------------------------------------------------------
# RCM numberer
# ---------------------------------------------------------------------------

def _chain_model(tag_order):
    """Truss chain whose node tags are deliberately scrambled."""
    m = Model(ndm=2, ndf=2)
    xs = {tag: 10.0 * i for i, tag in enumerate(tag_order)}
    for tag in tag_order:
        m.node(tag, xs[tag], 0.0)
        m.fix(tag, 0, 1)
    first, last = tag_order[0], tag_order[-1]
    m.fix(first, 1, 1)
    m.uniaxial_material("Elastic", 1, E=E)
    for k, (a, b) in enumerate(zip(tag_order, tag_order[1:]), start=1):
        m.element("Truss", k, a, b, A=2.0, mat=1)
    m.pattern("Plain", 1, "Linear")
    m.load(last, 10.0, 0.0)
    return m


def _bandwidth(domain, order):
    """Max |eq_i - eq_j| within any element under a given node ordering."""
    from src.fea.analysis_model import AnalysisModel
    from src.fea.constraints.sp import PlainHandler
    model = AnalysisModel()

    class _FixedOrder:
        def node_order(self, d):
            return order
    PlainHandler().handle(domain, model, _FixedOrder())
    bw = 0
    for elem in domain.elements():
        ids = [i for i in model.id_for(elem) if i >= 0]
        if len(ids) > 1:
            bw = max(bw, max(ids) - min(ids))
    return bw


def test_rcm_reduces_bandwidth_and_is_deterministic():
    # chain 1-9-2-8-3-7-4-6-5: tag order is terrible for bandwidth
    tags = [1, 9, 2, 8, 3, 7, 4, 6, 5]
    m = _chain_model(tags)
    plain = PlainNumberer().node_order(m.domain)
    rcm = RCMNumberer().node_order(m.domain)
    assert rcm == RCMNumberer().node_order(m.domain)  # deterministic
    assert sorted(rcm) == sorted(plain)               # a permutation
    assert _bandwidth(m.domain, rcm) < _bandwidth(m.domain, plain)


def test_rcm_isolated_nodes_go_last():
    m = Model(ndm=2, ndf=2)
    for t in (1, 2, 3):
        m.node(t, float(t), 0.0)
    m.uniaxial_material("Elastic", 1, E=E)
    m.element("Truss", 1, 1, 2, A=1.0, mat=1)   # node 3 unattached
    order = RCMNumberer().node_order(m.domain)
    assert order[-1] == 3 and set(order) == {1, 2, 3}


def test_rcm_solution_matches_plain():
    a1 = simply_supported_beam(L, E, A, I, n_ele=8, udl_wy=-W)
    a1.model.analyze(1)
    a2 = simply_supported_beam(L, E, A, I, n_ele=8, udl_wy=-W)
    a2.model.numberer("RCM")
    a2.model.analyze(1)
    mid1 = a1.model.node_disp(a1.key_nodes["mid"], 1)
    mid2 = a2.model.node_disp(a2.key_nodes["mid"], 1)
    assert mid1 == pytest.approx(mid2, rel=1e-12)
    for tag in a1.support_nodes:
        np.testing.assert_allclose(a1.model.node_reaction(tag),
                                   a2.model.node_reaction(tag),
                                   rtol=1e-10, atol=1e-9)


# ---------------------------------------------------------------------------
# Load cases + combinations
# ---------------------------------------------------------------------------

W_D = 1.0 / 12.0    # kip/in dead
W_L = 1.5 / 12.0    # kip/in live


def _two_case_beam():
    """SS beam with pattern 1 = D (UDL), pattern 2 = L (UDL)."""
    a = simply_supported_beam(L, E, A, I, n_ele=8, udl_wy=-W_D)
    m = a.model
    m.pattern("Plain", 2, "Linear")
    for tag in a.element_tags:
        m.ele_load(tag, wy=-W_L, pattern=2)
    return a


def _delta_mid(w):
    return 5.0 * w * L ** 4 / (384.0 * E * I)


def test_analyze_cases_solves_each_pattern_alone():
    a = _two_case_beam()
    res = a.model.analyze_cases({"D": 1, "L": 2})
    mid = a.key_nodes["mid"]
    left = a.key_nodes["left"]
    assert res["D"].node_disp[mid][1] == pytest.approx(-_delta_mid(W_D), rel=1e-9)
    assert res["L"].node_disp[mid][1] == pytest.approx(-_delta_mid(W_L), rel=1e-9)
    assert res["D"].reactions[left][1] == pytest.approx(W_D * L / 2, rel=1e-9)
    assert res["L"].reactions[left][1] == pytest.approx(W_L * L / 2, rel=1e-9)


def test_analyze_cases_no_state_leakage_and_resets():
    a = _two_case_beam()
    first = a.model.analyze_cases({"D": 1, "L": 2})
    second = a.model.analyze_cases({"D": 1})   # re-run D alone afterwards
    mid = a.key_nodes["mid"]
    assert second["D"].node_disp[mid][1] == pytest.approx(
        first["D"].node_disp[mid][1], rel=1e-12)
    # model left wiped, all patterns active again
    assert a.model.node_disp(mid, 1) == 0.0
    assert len(list(a.model.domain.load_patterns())) == 2


def test_combine_matches_factored_closed_form():
    a = _two_case_beam()
    res = a.model.analyze_cases({"D": 1, "L": 2})
    lc2 = combos.combine("LC2", res, {"D": 1.2, "L": 1.6})
    wu = 1.2 * W_D + 1.6 * W_L
    mid, left = a.key_nodes["mid"], a.key_nodes["left"]
    assert lc2.node_disp[mid][1] == pytest.approx(-_delta_mid(wu), rel=1e-9)
    assert lc2.reactions[left][1] == pytest.approx(wu * L / 2, rel=1e-9)
    # element end forces combine linearly too
    e1 = a.element_tags[0]
    np.testing.assert_allclose(
        lc2.ele_force[e1],
        1.2 * res["D"].ele_force[e1] + 1.6 * res["L"].ele_force[e1],
        rtol=1e-12, atol=1e-12)


def test_combine_unknown_case_raises():
    a = _two_case_beam()
    res = a.model.analyze_cases({"D": 1})
    with pytest.raises(KeyError, match="unknown case"):
        combos.combine("LC", res, {"D": 1.2, "S": 1.0})


def test_envelope_tracks_governing_combo():
    a = _two_case_beam()
    res = a.model.analyze_cases({"D": 1, "L": 2})
    lc1 = combos.combine("LC1", res, {"D": 1.4})
    lc2 = combos.combine("LC2", res, {"D": 1.2, "L": 1.6})
    env = combos.envelope([lc1, lc2])
    left, mid = a.key_nodes["left"], a.key_nodes["mid"]
    wu1, wu2 = 1.4 * W_D, 1.2 * W_D + 1.6 * W_L
    assert env.reactions_max[left][1] == pytest.approx(max(wu1, wu2) * L / 2,
                                                       rel=1e-9)
    assert env.reactions_max_src[left][1] == "LC2"
    # largest downward deflection (most negative) also from LC2
    assert env.node_disp_min[mid][1] == pytest.approx(-_delta_mid(wu2), rel=1e-9)
    assert env.node_disp_min_src[mid][1] == "LC2"


# ---------------------------------------------------------------------------
# Archetypes vs closed form
# ---------------------------------------------------------------------------

def test_ss_beam_udl_closed_form():
    a = simply_supported_beam(L, E, A, I, n_ele=10, udl_wy=-W)
    a.model.analyze(1)
    assert a.model.node_disp(a.key_nodes["mid"], 1) == pytest.approx(
        -_delta_mid(W), rel=1e-9)
    for s in a.support_nodes:
        assert a.model.node_reaction(s, 1) == pytest.approx(W * L / 2, rel=1e-9)


def test_ss_beam_midspan_point_load():
    a = simply_supported_beam(L, E, A, I, n_ele=8,
                              point_loads=[(L / 2, 0.0, -P)])
    a.model.analyze(1)
    assert a.model.node_disp(a.key_nodes["mid"], 1) == pytest.approx(
        -P * L ** 3 / (48 * E * I), rel=1e-9)


def test_ss_beam_offcenter_point_load_gets_mesh_node():
    x = 0.3 * L                     # not on the even mesh
    a = simply_supported_beam(L, E, A, I, n_ele=7, point_loads=[(x, 0.0, -P)])
    a.model.analyze(1)
    assert a.model.node_reaction(a.key_nodes["left"], 1) == pytest.approx(
        P * 0.7, rel=1e-9)
    assert a.model.node_reaction(a.key_nodes["right"], 1) == pytest.approx(
        P * 0.3, rel=1e-9)
    a.node_near(x)                  # the load position is a real node


def test_cantilever_tip_load_and_udl():
    c = cantilever(L, E, A, I, n_ele=6, tip_Py=-P)
    c.model.analyze(1)
    assert c.model.node_disp(c.key_nodes["tip"], 1) == pytest.approx(
        -P * L ** 3 / (3 * E * I), rel=1e-9)
    assert c.model.node_reaction(c.key_nodes["base"], 2) == pytest.approx(
        P * L, rel=1e-9)            # base moment

    c2 = cantilever(L, E, A, I, n_ele=6, udl_wy=-W)
    c2.model.analyze(1)
    assert c2.model.node_disp(c2.key_nodes["tip"], 1) == pytest.approx(
        -W * L ** 4 / (8 * E * I), rel=1e-9)


def test_continuous_two_equal_spans_udl():
    Ls = 240.0                      # 20 ft spans
    cb = continuous_beam([Ls, Ls], E, A, I, n_ele_per_span=8, udl_wy=-W)
    cb.model.analyze(1)
    R_end = 3.0 * W * Ls / 8.0
    R_mid = 10.0 * W * Ls / 8.0
    assert cb.model.node_reaction(cb.key_nodes["support_0"], 1) == \
        pytest.approx(R_end, rel=1e-9)
    assert cb.model.node_reaction(cb.key_nodes["support_1"], 1) == \
        pytest.approx(R_mid, rel=1e-9)
    assert cb.model.node_reaction(cb.key_nodes["support_2"], 1) == \
        pytest.approx(R_end, rel=1e-9)


def test_portal_frame_statics_and_mesh_invariance():
    kw = dict(width=240.0, height=144.0, E=E, A_col=A, I_col=I,
              A_beam=A, I_beam=1.5 * I, lateral_Px=10.0, beam_udl_wy=-W)
    p1 = portal_frame(n_ele_per_member=1, **kw)
    p4 = portal_frame(n_ele_per_member=4, **kw)
    p1.model.analyze(1)
    p4.model.analyze(1)

    # Bernoulli elements are nodally exact -> mesh density cannot matter
    for name in ("top_left", "top_right"):
        np.testing.assert_allclose(
            p1.model.node_disp(p1.key_nodes[name]),
            p4.model.node_disp(p4.key_nodes[name]), rtol=1e-9, atol=1e-12)

    # global statics
    for p in (p1, p4):
        Rx = sum(p.model.node_reaction(t, 0) for t in p.support_nodes)
        Ry = sum(p.model.node_reaction(t, 1) for t in p.support_nodes)
        assert Rx == pytest.approx(-10.0, abs=1e-9)          # balances P
        assert Ry == pytest.approx(W * 240.0, rel=1e-9)      # balances UDL


def test_portal_frame_pinned_base_has_no_base_moment():
    p = portal_frame(width=240.0, height=144.0, E=E, A_col=A, I_col=I,
                     A_beam=A, I_beam=I, n_ele_per_member=2,
                     fixed_base=False, lateral_Px=10.0)
    p.model.analyze(1)
    for t in p.support_nodes:
        assert p.model.node_reaction(t, 2) == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Section property tool
# ---------------------------------------------------------------------------

def test_rectangle_closed_form():
    b, h = 4.0, 12.0
    r = sp.compute([sp.rectangle(b, h)])
    assert r.A == pytest.approx(48.0)
    assert r.Ix == pytest.approx(b * h ** 3 / 12)
    assert r.Iy == pytest.approx(h * b ** 3 / 12)
    assert r.rx == pytest.approx(math.sqrt(r.Ix / r.A))
    assert not r.j_is_approximate
    # square torsion: J = 0.141 a^4
    sq = sp.compute([sp.rectangle(5.0, 5.0)])
    assert sq.J == pytest.approx(0.141 * 5.0 ** 4, rel=1e-6)


def test_circle_closed_form():
    d = 10.0
    c = sp.compute([sp.circle(d)])
    assert c.A == pytest.approx(math.pi * 25.0)
    assert c.Ix == pytest.approx(math.pi * d ** 4 / 64)
    assert c.J == pytest.approx(math.pi * d ** 4 / 32)
    assert c.Avy == pytest.approx(0.9 * c.A)


def test_polygon_matches_rectangle():
    b, h = 6.0, 10.0
    poly = sp.polygon([(0, 0), (b, 0), (b, h), (0, h)])
    r = sp.compute([poly])
    assert r.A == pytest.approx(60.0)
    assert (r.cx, r.cy) == (pytest.approx(3.0), pytest.approx(5.0))
    assert r.Ix == pytest.approx(b * h ** 3 / 12)
    assert r.Iy == pytest.approx(h * b ** 3 / 12)
    # clockwise vertex order gives identical positives
    r2 = sp.compute([sp.polygon([(0, 0), (0, h), (b, h), (b, 0)])])
    assert r2.Ix == pytest.approx(r.Ix) and r2.A == pytest.approx(r.A)


def test_box_with_hole():
    outer, wall = 10.0, 1.0
    inner = outer - 2 * wall
    r = sp.compute([sp.rectangle(outer, outer),
                    sp.rectangle(inner, inner, hole=True)])
    assert r.A == pytest.approx(outer ** 2 - inner ** 2)
    assert r.Ix == pytest.approx((outer ** 4 - inner ** 4) / 12)
    assert r.J == 0.0 and r.j_is_approximate  # closed cell: no open-sum J


def test_w12x26_from_three_plates_near_catalogue():
    # AISC v16: W12X26  A=7.65 in^2, Ix=204 in^4, Iy=17.3 in^4 (incl. fillets)
    r = sp.compute(sp.i_shape(d=12.2, bf=6.49, tf=0.38, tw=0.23))
    assert r.A == pytest.approx(7.65, rel=0.025)
    assert r.Ix == pytest.approx(204.0, rel=0.025)
    assert r.Iy == pytest.approx(17.3, rel=0.025)
    assert r.Ixy == pytest.approx(0.0, abs=1e-9)
    assert r.theta_p_deg == pytest.approx(0.0, abs=1e-6)


def test_rotated_rectangle_principal_recovery():
    b, h, ang = 3.0, 9.0, 30.0
    r = sp.compute([sp.rectangle(b, h, angle_deg=ang)])
    assert r.I1 == pytest.approx(b * h ** 3 / 12, rel=1e-9)
    assert r.I2 == pytest.approx(h * b ** 3 / 12, rel=1e-9)
    # tall rect tilted CCW: mass in quadrants II/IV -> Ixy < 0, and the
    # strong (I1) principal axis sits at exactly +30 deg CCW from x
    assert r.Ixy < 0
    assert r.theta_p_deg == pytest.approx(ang, abs=1e-9)


def test_right_triangle_polygon_ixy():
    b, h = 6.0, 9.0
    r = sp.compute([sp.polygon([(0, 0), (b, 0), (0, h)])])
    assert r.A == pytest.approx(b * h / 2)
    assert (r.cx, r.cy) == (pytest.approx(b / 3), pytest.approx(h / 3))
    assert r.Ix == pytest.approx(b * h ** 3 / 36)
    assert r.Ixy == pytest.approx(-b ** 2 * h ** 2 / 72, rel=1e-9)


def test_equal_leg_angle_principal_axis_45deg():
    leg, t = 6.0, 1.0
    shapes = [sp.rectangle(t, leg, cx=t / 2, cy=leg / 2),
              sp.rectangle(leg - t, t, cx=t + (leg - t) / 2, cy=t / 2)]
    r = sp.compute(shapes)
    assert abs(abs(r.theta_p_deg) - 45.0) < 1e-6


def test_to_elastic_section_mapping():
    r = sp.compute([sp.rectangle(4.0, 12.0)])
    sec = sp.to_elastic_section(1, E=29000.0, props=r)
    assert sec.A == pytest.approx(r.A)
    assert sec.Iz == pytest.approx(r.Ix)
    assert sec.E == 29000.0
