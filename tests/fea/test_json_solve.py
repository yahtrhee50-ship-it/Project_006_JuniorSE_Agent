"""
JSON front door (solve_model_json) — the plan's §15 verification gate:
solve a small JSON portal frame and confirm the returned reactions satisfy
statics. Plus closed-form spot checks. Imperial units (kips, in, ksi).
"""
from __future__ import annotations

import re

import pytest

from src.fea.json_solve import solve_model_json

E, A, I = 29000.0, 20.0, 800.0


def _reaction(text: str, node: int) -> list[float]:
    in_section = False
    for line in text.splitlines():
        if line.startswith("## Support Reactions"):
            in_section = True
        elif in_section and line.startswith("##"):
            break
        elif in_section:
            parts = line.split()
            if parts and parts[0] == str(node):
                return [float(v) for v in parts[1:]]
    raise AssertionError(f"no reaction row for node {node}")


def test_ss_beam_udl_matches_closed_form():
    L, w = 360.0, 2.0 / 12.0
    out = solve_model_json(
        nodes=[{"id": 1, "x": 0, "y": 0, "fix": [1, 1, 0]},
               {"id": 2, "x": L / 2, "y": 0},
               {"id": 3, "x": L, "y": 0, "fix": [0, 1, 0]}],
        sections=[{"id": 1, "E": E, "A": A, "I": I}],
        elements=[{"id": 1, "type": "Frame", "i": 1, "j": 2, "section": 1},
                  {"id": 2, "type": "Frame", "i": 2, "j": 3, "section": 1}],
        ele_loads=[{"element": 1, "wy": -w}, {"element": 2, "wy": -w}],
    )
    assert "equilibrium: OK" in out
    assert _reaction(out, 1)[1] == pytest.approx(w * L / 2, rel=1e-9)
    assert _reaction(out, 3)[1] == pytest.approx(w * L / 2, rel=1e-9)
    # midspan deflection printed for node 2: 5wL^4/384EI down
    mid_row = [ln for ln in out.splitlines() if re.match(r"\s+2\s", ln)][0]
    uy = float(mid_row.split()[2])
    # report prints 6 significant figures — compare at print precision
    assert uy == pytest.approx(-5 * w * L ** 4 / (384 * E * I), rel=1e-6)


def test_portal_frame_json_statics_gate():
    W, H, P = 240.0, 144.0, 10.0
    w = 2.0 / 12.0
    out = solve_model_json(
        nodes=[{"id": 1, "x": 0, "y": 0, "fix": [1, 1, 1]},
               {"id": 2, "x": 0, "y": H},
               {"id": 3, "x": W, "y": H},
               {"id": 4, "x": W, "y": 0, "fix": [1, 1, 1]}],
        sections=[{"id": 1, "E": E, "A": A, "I": I}],
        elements=[{"id": 1, "type": "Frame", "i": 1, "j": 2, "section": 1},
                  {"id": 2, "type": "Frame", "i": 2, "j": 3, "section": 1},
                  {"id": 3, "type": "Frame", "i": 3, "j": 4, "section": 1}],
        nodal_loads=[{"node": 2, "Fx": P}],
        ele_loads=[{"element": 2, "wy": -w}],
    )
    assert "equilibrium: OK" in out
    r1, r4 = _reaction(out, 1), _reaction(out, 4)
    assert r1[0] + r4[0] == pytest.approx(-P, abs=1e-9)
    assert r1[1] + r4[1] == pytest.approx(w * W, rel=1e-9)


def test_truss_json_ndf2():
    out = solve_model_json(
        nodes=[{"id": 1, "x": 0, "y": 0, "fix": [1, 1]},
               {"id": 2, "x": 120.0, "y": 0, "fix": [0, 1]},
               {"id": 3, "x": 60.0, "y": 80.0}],
        materials=[{"id": 1, "E": E}],
        elements=[{"id": 1, "type": "Truss", "i": 1, "j": 2, "A": 2.0, "mat": 1},
                  {"id": 2, "type": "Truss", "i": 1, "j": 3, "A": 2.0, "mat": 1},
                  {"id": 3, "type": "Truss", "i": 2, "j": 3, "A": 2.0, "mat": 1}],
        nodal_loads=[{"node": 3, "Fy": -20.0}],
        ndf=2,
    )
    assert "equilibrium: OK" in out
    assert _reaction(out, 1)[1] == pytest.approx(10.0, rel=1e-9)
    assert _reaction(out, 2)[1] == pytest.approx(10.0, rel=1e-9)


def test_propped_cantilever_and_rcm_matches_plain():
    """Propped cantilever (fixed + roller) vs closed form (5wL/8, 3wL/8,
    wL^2/8); identical results under RCM numbering."""
    L, w = 240.0, 0.5
    kwargs = dict(
        nodes=[{"id": 1, "x": 0, "y": 0, "fix": [1, 1, 1]},
               {"id": 2, "x": L / 2, "y": 0},
               {"id": 3, "x": L, "y": 0, "fix": [0, 1, 0]}],
        sections=[{"id": 1, "E": E, "A": A, "I": I}],
        elements=[{"id": 1, "type": "Frame", "i": 1, "j": 2, "section": 1},
                  {"id": 2, "type": "Frame", "i": 2, "j": 3, "section": 1}],
        ele_loads=[{"element": 1, "wy": -w}, {"element": 2, "wy": -w}],
    )
    out_plain = solve_model_json(**kwargs)
    out_rcm = solve_model_json(numberer="RCM", **kwargs)
    for out in (out_plain, out_rcm):
        assert "equilibrium: OK" in out
        assert _reaction(out, 1)[1] == pytest.approx(5 * w * L / 8, rel=1e-9)
        assert _reaction(out, 3)[1] == pytest.approx(3 * w * L / 8, rel=1e-9)
        assert _reaction(out, 1)[2] == pytest.approx(w * L ** 2 / 8, rel=1e-9)


def test_input_errors():
    with pytest.raises(ValueError, match="nodes is empty"):
        solve_model_json([], [{"id": 1}])
    with pytest.raises(ValueError, match="unknown type"):
        solve_model_json(
            nodes=[{"id": 1, "x": 0, "y": 0, "fix": [1, 1, 1]},
                   {"id": 2, "x": 10, "y": 0}],
            elements=[{"id": 1, "type": "Shell", "i": 1, "j": 2}])


def test_mcp_tool_registered_and_wraps_errors():
    import asyncio
    from src import mcp_server
    names = {t.name for t in asyncio.run(mcp_server.mcp.list_tools())}
    assert "solve_model" in names
    out = mcp_server.solve_model(nodes=[], elements=[])
    assert out.startswith("INPUT ERROR:")
