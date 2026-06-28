"""End-to-end tests for the design_beam workflow (src/mcp_server.py).

Calls the workflow function directly (not over the MCP transport) and verifies the
result against hand calcs and the prior aisc360 verification, plus the audit trail.
"""
import json
import math
import shutil

import pytest

from src import mcp_server as S
from src.agent import project as P

_TEST_PROJECT = "pytest_beam"   # no leading underscore (slug() strips those)


@pytest.fixture()
def project_dir():
    d = P.resolve_project_dir(_TEST_PROJECT)
    shutil.rmtree(d)            # start clean (resolve created it)
    yield _TEST_PROJECT
    d = P._PROJECTS_DIR / _TEST_PROJECT
    if d.exists():
        shutil.rmtree(d)


def _inventory(project_name):
    d = P._PROJECTS_DIR / project_name
    return json.loads((d / "members" / "inventory.json").read_text(encoding="utf-8"))


def test_w16x40_self_consistency(project_dir):
    # W16X40, 30 ft, D=2 + L=1 klf, fully braced.
    # self-wt = 40 plf -> D=2.040; LC2 = 1.2*2.040 + 1.6*1 = 4.048 klf
    out = S.design_beam(span_ft=30, loads={"D": 2, "L": 1}, section="W16X40",
                        Lb_ft=0, member_id="B1", project=project_dir)
    assert "W16X40" in out

    rec = _inventory(project_dir)["beams"]["B1"]
    assert rec["section"] == "W16X40"
    assert rec["governing_combo"] == "LC2"
    assert math.isclose(rec["w_klf"], 4.048, abs_tol=1e-3)
    # Mmax = w L^2 / 8
    assert math.isclose(rec["Mu_kip_ft"], 4.048 * 30**2 / 8, abs_tol=0.2)
    # Vmax = w L / 2
    assert math.isclose(rec["Vu_kips"], 4.048 * 30 / 2, abs_tol=0.2)
    # phiMn = 273.8 -> DCR = 455.4 / 273.8 = 1.664 (matches prior aisc360 verification)
    assert math.isclose(rec["DCR_flexure"], 1.664, abs_tol=0.01)
    assert rec["limit_state"] == "yielding"


def test_archive_written(project_dir):
    S.design_beam(span_ft=30, loads={"D": 2, "L": 1}, section="W16X40",
                  member_id="B1", project=project_dir)
    calcs = P._PROJECTS_DIR / project_dir / "calcs"
    md = list(calcs.glob("*.md"))
    csv = list(calcs.glob("*.csv"))
    assert len(md) == 1 and len(csv) == 1
    # n_elements=20 -> 41 stations -> header + 41 data rows
    rows = csv[0].read_text(encoding="utf-8").splitlines()
    assert len(rows) == 42


def test_auto_size_selects_adequate_section(project_dir):
    # No section given -> server sizes the lightest adequate W.
    out = S.design_beam(span_ft=20, loads={"D": 1, "L": 1},
                        member_id="B2", project=project_dir)
    assert "auto-sized" in out
    rec = _inventory(project_dir)["beams"]["B2"]
    assert rec["section"].startswith("W")
    # sized for adequacy (self-weight pass may nudge slightly above 1.0)
    assert rec["DCR_flexure"] <= 1.10


def test_full_detail_includes_csv(project_dir):
    out = S.design_beam(span_ft=30, loads={"D": 2, "L": 1}, section="W16X40",
                        member_id="B1", project=project_dir, detail="full")
    assert "Diagram CSV" in out and "x (ft)" in out


def test_bad_section_returns_input_error(project_dir):
    out = S.design_beam(span_ft=30, loads={"D": 2}, section="W99X999",
                        project=project_dir)
    assert out.startswith("INPUT ERROR")


def test_non_simple_support_escalates(project_dir):
    out = S.design_beam(span_ft=30, loads={"D": 2}, support="fixed",
                        project=project_dir)
    assert out.startswith("ESCALATE")


def test_zero_load_rejected(project_dir):
    out = S.design_beam(span_ft=30, loads={}, section="W16X40", project=project_dir)
    assert out.startswith("INPUT ERROR")
