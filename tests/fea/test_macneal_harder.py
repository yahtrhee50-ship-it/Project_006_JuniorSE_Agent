"""
FEA-engine validation: MacNeal-Harder standard element accuracy tests
(beam-relevant subset: 2D Bernoulli Frame + 3D Frame3D since Phase 2 Step 4).

Problem specs and theoretical targets are in Reference/macneal_harder_models.json
(MacNeal & Harder 1985, cross-checked against Altair OS-V:0710/0720).

2D cases (Frame):
  1. Straight cantilever, extension       — target tip u = 3.0e-5 (exact for any mesh)
  2. Straight cantilever, in-plane shear  — target tip v = 0.1081
     (target includes shear deformation; Bernoulli converges to PL^3/3EI = 0.10800,
      ratio 0.9991 — inside the 0.5% band asserted below)
  3. Curved cantilever (90 deg arc), in-plane tip load — target 0.08734
     (arc meshed with straight Frame chords on the mean radius; Bernoulli
      bending+axial limit is 0.088551, ratio 1.0139 vs the target, which
      reflects shear/curved-beam theory — asserted inside a 2% band, plus a
      mesh-convergence check toward the Bernoulli analytic limit)

3D cases (Frame3D, previously deferred):
  4. Straight cantilever, out-of-plane shear — target 0.4321; Bernoulli is
     nodally exact at PL^3/3EIy = 0.43200 (ratio 0.99977).
  5. Straight cantilever, twist — target 0.03208. The element gives exact
     St-Venant TL/GJ; with the exact rectangular J = 4.573634e-5 that is
     0.0341085 (ratio 1.0632). The paper's implied J (4.8628e-5) matches no
     standard rectangle formula — see the JSON's J_provenance note. Both
     behaviors are asserted: exact TL/GJ with the exact J, and exact target
     reproduction with the target-implied J.
  6. Curved cantilever, out-of-plane tip load — target 0.5022; Bernoulli
     bending+torsion limit R^3[(pi/4)/EI + (3pi/4-2)/GJ] = 0.500463
     (ratio 0.99654), with a mesh-convergence check toward that limit.
"""
import json
import math
from pathlib import Path

import pytest

from src.fea.api import Model

_JSON = Path(__file__).resolve().parents[2] / "Reference" / "macneal_harder_models.json"
_SPEC = json.loads(_JSON.read_text())
_SC = next(m for m in _SPEC["models"] if m["id"] == "MH-SC")
_CB = next(m for m in _SPEC["models"] if m["id"] == "MH-CB")

E = _SC["material"]["E"]
L = _SC["geometry"]["length"]
A = _SC["geometry"]["A"]
I_IN = _SC["geometry"]["I_in_plane"]


def _straight_cantilever(n_ele=6):
    m = Model(ndm=2, ndf=3)
    for k in range(n_ele + 1):
        m.node(k + 1, k * L / n_ele, 0.0)
    m.fix(1, 1, 1, 1)
    m.section("Elastic", 1, E=E, A=A, Iz=I_IN)
    for k in range(n_ele):
        m.element("Frame", k + 1, k + 1, k + 2, section=1)
    m.pattern("Plain", 1, "Linear")
    return m, n_ele + 1


def test_sc_extension_matches_target():
    target = next(c for c in _SC["cases"] if c["case"] == "extension")[
        "target_tip_displacement"]
    m, tip = _straight_cantilever()
    m.load(tip, 1.0, 0.0, 0.0)
    m.analyze(1)
    ux = m.node_disp(tip, 0)
    # PL/AE — exact for the axial stiffness, so the band is tight.
    assert ux == pytest.approx(target, rel=1e-6)


def test_sc_in_plane_shear_matches_target():
    target = next(c for c in _SC["cases"] if c["case"] == "in_plane_shear")[
        "target_tip_displacement"]
    m, tip = _straight_cantilever()
    m.load(tip, 0.0, 1.0, 0.0)
    m.analyze(1)
    uy = m.node_disp(tip, 1)
    # Bernoulli is nodally exact for a tip point load: PL^3/3EI.
    assert uy == pytest.approx(L**3 / (3 * E * I_IN), rel=1e-9)
    # Target includes shear deformation the element omits (ratio 0.9991).
    assert uy == pytest.approx(target, rel=5e-3)


def _curved_cantilever(n_ele):
    """90-degree arc on the mean radius, fixed at (R,0), tip at (0,R),
    unit +y load at the tip (radial there, per the paper's in-plane case)."""
    g = _CB["geometry"]
    R = g["mean_radius"]
    m = Model(ndm=2, ndf=3)
    for k in range(n_ele + 1):
        th = (math.pi / 2) * k / n_ele
        m.node(k + 1, R * math.cos(th), R * math.sin(th))
    m.fix(1, 1, 1, 1)
    m.section("Elastic", 1, E=_CB["material"]["E"], A=g["A"], Iz=g["I_in_plane"])
    for k in range(n_ele):
        m.element("Frame", k + 1, k + 1, k + 2, section=1)
    m.pattern("Plain", 1, "Linear")
    m.load(n_ele + 1, 0.0, 1.0, 0.0)
    m.analyze(1)
    return m.node_disp(n_ele + 1, 1)


def test_cb_in_plane_matches_target():
    case = next(c for c in _CB["cases"] if c["case"] == "in_plane_load")
    target = case["target_tip_displacement_in_load_dir"]
    uy = _curved_cantilever(64)
    assert uy == pytest.approx(target, rel=2e-2)


def test_cb_mesh_convergence_to_bernoulli_limit():
    g = _CB["geometry"]
    R = g["mean_radius"]
    Ec = _CB["material"]["E"]
    # Analytic Bernoulli (no shear def) curved-cantilever tip deflection in
    # the load direction: bending pi*R^3/(4EI) + axial pi*R/(4EA).
    limit = (math.pi * R**3 / (4 * Ec * g["I_in_plane"])
             + math.pi * R / (4 * Ec * g["A"]))
    err = [abs(_curved_cantilever(n) - limit) for n in (8, 16, 64)]
    assert err[0] > err[1] > err[2], f"non-monotonic convergence: {err}"
    assert _curved_cantilever(64) == pytest.approx(limit, rel=1e-3)


# -- 3D cases (Frame3D, FEA Phase 2 Step 4) -----------------------------------

I_OUT = _SC["geometry"]["I_out_of_plane"]
J_EXACT = _SC["geometry"]["J_st_venant_exact"]
G_SC = E / (2 * (1 + _SC["material"]["nu"]))


def _straight_cantilever_3d(n_ele=6, J=None):
    m = Model(ndm=3, ndf=6)
    for k in range(n_ele + 1):
        m.node(k + 1, k * L / n_ele, 0.0, 0.0)
    m.fix(1, 1, 1, 1, 1, 1, 1)
    m.section("Elastic", 1, E=E, A=A, Iz=I_IN, Iy=I_OUT,
              J=J if J is not None else J_EXACT, G=G_SC)
    for k in range(n_ele):
        m.element("Frame3D", k + 1, k + 1, k + 2, section=1,
                  vecxz=(0.0, 0.0, 1.0))
    m.pattern("Plain", 1, "Linear")
    return m, n_ele + 1


def test_sc_out_of_plane_shear_matches_target():
    target = next(c for c in _SC["cases"] if c["case"] == "out_of_plane_shear")[
        "target_tip_displacement"]
    m, tip = _straight_cantilever_3d()
    m.load(tip, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    m.analyze(1)
    uz = m.node_disp(tip, 2)
    # Bernoulli is nodally exact for a tip point load: PL^3/3EIy.
    assert uz == pytest.approx(L**3 / (3 * E * I_OUT), rel=1e-9)
    # Target includes shear deformation the element omits (ratio 0.99977).
    assert uz == pytest.approx(target, rel=5e-3)


def test_sc_twist_exact_TL_GJ():
    """The element is exact St-Venant torsion: theta = TL/GJ to machine
    precision with the exact rectangular J (ratio vs the paper target is
    1.0632 — the target's J convention is unresolved, see the JSON)."""
    m, tip = _straight_cantilever_3d()
    m.load(tip, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    m.analyze(1)
    assert m.node_disp(tip, 3) == pytest.approx(L / (G_SC * J_EXACT), rel=1e-9)


def test_sc_twist_reproduces_target_with_implied_J():
    case = next(c for c in _SC["cases"] if c["case"] == "twist")
    m, tip = _straight_cantilever_3d(J=case["J_implied_by_target"])
    m.load(tip, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    m.analyze(1)
    assert m.node_disp(tip, 3) == pytest.approx(
        case["target_tip_rotation"], rel=1e-4)


def _curved_cantilever_3d(n_ele):
    """Same arc as the 2D case (x-y plane, fixed at (R,0)), unit +z load
    at the tip — out-of-plane bending (EIy) + torsion (GJ)."""
    g = _CB["geometry"]
    R = g["mean_radius"]
    Ec = _CB["material"]["E"]
    Gc = Ec / (2 * (1 + _CB["material"]["nu"]))
    m = Model(ndm=3, ndf=6)
    for k in range(n_ele + 1):
        th = (math.pi / 2) * k / n_ele
        m.node(k + 1, R * math.cos(th), R * math.sin(th), 0.0)
    m.fix(1, 1, 1, 1, 1, 1, 1)
    m.section("Elastic", 1, E=Ec, A=g["A"], Iz=g["I_in_plane"],
              Iy=g["I_out_of_plane"], J=g["J_st_venant_exact"], G=Gc)
    for k in range(n_ele):
        m.element("Frame3D", k + 1, k + 1, k + 2, section=1,
                  vecxz=(0.0, 0.0, 1.0))
    m.pattern("Plain", 1, "Linear")
    m.load(n_ele + 1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    m.analyze(1)
    return m.node_disp(n_ele + 1, 2)


def test_cb_out_of_plane_matches_target():
    case = next(c for c in _CB["cases"] if c["case"] == "out_of_plane_load")
    target = case["target_tip_displacement_in_load_dir"]
    uz = _curved_cantilever_3d(64)
    assert uz == pytest.approx(target, rel=1e-2)


def test_cb_out_of_plane_convergence_to_bernoulli_limit():
    g = _CB["geometry"]
    R = g["mean_radius"]
    Ec = _CB["material"]["E"]
    Gc = Ec / (2 * (1 + _CB["material"]["nu"]))
    # Unit-load method on the arc: bending sin^2 + torsion (1-cos)^2 terms.
    limit = R**3 * (
        (math.pi / 4) / (Ec * g["I_out_of_plane"])
        + (3 * math.pi / 4 - 2) / (Gc * g["J_st_venant_exact"]))
    err = [abs(_curved_cantilever_3d(n) - limit) for n in (8, 16, 64)]
    assert err[0] > err[1] > err[2], f"non-monotonic convergence: {err}"
    assert _curved_cantilever_3d(64) == pytest.approx(limit, rel=1e-3)
