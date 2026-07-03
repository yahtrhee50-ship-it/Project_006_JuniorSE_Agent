"""
FEA-engine validation: MacNeal-Harder standard element accuracy tests
(beam-relevant subset runnable with the 2D Bernoulli Frame element).

Problem specs and theoretical targets are in Reference/macneal_harder_models.json
(MacNeal & Harder 1985, cross-checked against Altair OS-V:0710/0720).

Three cases run here:
  1. Straight cantilever, extension       — target tip u = 3.0e-5 (exact for any mesh)
  2. Straight cantilever, in-plane shear  — target tip v = 0.1081
     (target includes shear deformation; Bernoulli converges to PL^3/3EI = 0.10800,
      ratio 0.9991 — inside the 0.5% band asserted below)
  3. Curved cantilever (90 deg arc), in-plane tip load — target 0.08734
     (arc meshed with straight Frame chords on the mean radius; Bernoulli
      bending+axial limit is 0.088551, ratio 1.0139 vs the target, which
      reflects shear/curved-beam theory — asserted inside a 2% band, plus a
      mesh-convergence check toward the Bernoulli analytic limit)

The out-of-plane and twist cases need a 3D frame element (torsion + biaxial
bending) and are deferred; their targets are recorded in the JSON.
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
