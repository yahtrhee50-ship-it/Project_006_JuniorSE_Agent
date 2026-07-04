"""
Live SAP2000 thick-shell validation, cross-checked against stiffness-method
references from this repo's verified solvers (src.fea).

Phase A — known scenario: simply supported square concrete plate (thick-shell
elements) under uniform load vs the classical Kirchhoff/Navier solution
w_max = 0.00406*q*a^4/D, plus exact statics on total reactions.

Phase B — system check: concrete deck (thick shell) on steel beams + girders
+ columns (added via the new P005 `add_columns` operation). Checks:
  1. Total base reaction per load case == hand statics (exact).
  2. Column base reactions equal by double symmetry, sum == total.
  3. Interior beam / girder midspan LL moments vs tributary-width
     stiffness-method references solved with src.fea (ratios reported;
     shells legitimately carry part of the load, so bands are generous).

Requires the Project_005 API server on 127.0.0.1:8000 and SAP2000.
Run:  C:\\Python314\\python.exe scripts/check_thick_shell_deck.py
"""
from __future__ import annotations

import json
import math
import sys
import urllib.request
from pathlib import Path
from urllib.parse import urlencode

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.fea import Model  # noqa: E402  (stiffness-method reference engine)

BASE = "http://127.0.0.1:8000"
OUT_DIR = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3\outputs"

CHECKS: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str) -> None:
    CHECKS.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def api(path: str, payload=None, **query) -> dict:
    url = BASE + path + ("?" + urlencode(query) if query else "")
    data = json.dumps(payload if payload is not None else {}).encode()
    req = urllib.request.Request(url, data=data, method="POST",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=900) as resp:
        return json.loads(resp.read())


def op(name: str, **params) -> dict:
    return api(f"/api/sap2000/op/{name}", params)


def fz_totals(cases: list[str]) -> dict[str, float]:
    rows = op("base_reactions", cases=cases)["base_reactions"]
    return {r["case"]: r["FZ"] for r in rows}


def frame_map(report: dict) -> dict[str, str]:
    """requested -> actual SAP2000 frame names from a build report."""
    out = {}
    for entry in report["frames"]:
        if " -> " in entry:
            req, act = entry.split(" -> ")
        else:
            req = act = entry
        out[req] = act
    return out


# ---------------------------------------------------------------------------
# Stiffness-method references (src.fea — engine itself is benchmarked against
# Hibbeler Ch14/15/16 + MacNeal-Harder; see tests/fea/)
# ---------------------------------------------------------------------------

def fea_ss_beam_udl(L: float, w: float,
                    EI: float = 1.0) -> tuple[float, float, float]:
    """Simply supported beam, UDL w (down). Returns (end reaction,
    midspan M, midspan deflection for the given EI)."""
    m = Model(ndm=2, ndf=3)
    m.node(1, 0.0, 0.0)
    m.node(2, L / 2, 0.0)
    m.node(3, L, 0.0)
    m.fix(1, 1, 1, 0)
    m.fix(3, 0, 1, 0)
    m.section("Elastic", 1, E=EI, A=1.0, Iz=1.0)
    m.element("Frame", 1, 1, 2, section=1)
    m.element("Frame", 2, 2, 3, section=1)
    m.pattern("Plain", 1, "Linear")
    m.ele_load(1, wy=-w)
    m.ele_load(2, wy=-w)
    m.analyze(1)
    R = m.node_reaction(1, 1)
    M_mid = R * (L / 2) - w * (L / 2) ** 2 / 2   # statics on the free body
    return R, M_mid, -m.node_disp(2, 1)


def fea_ss_beam_2P(L: float, a: float, P: float) -> tuple[float, float]:
    """SS beam, point loads P (down) at a and L-a. Returns (end R, midspan M)."""
    m = Model(ndm=2, ndf=3)
    m.node(1, 0.0, 0.0)
    m.node(2, a, 0.0)
    m.node(3, L - a, 0.0)
    m.node(4, L, 0.0)
    m.fix(1, 1, 1, 0)
    m.fix(4, 0, 1, 0)
    m.section("Elastic", 1, E=1.0, A=1.0, Iz=1.0)
    for tag, (i, j) in enumerate([(1, 2), (2, 3), (3, 4)], start=1):
        m.element("Frame", tag, i, j, section=1)
    m.pattern("Plain", 1, "Linear")
    m.load(2, 0.0, -P, 0.0)
    m.load(3, 0.0, -P, 0.0)
    m.analyze(1)
    R = m.node_reaction(1, 1)
    M_mid = R * (L / 2) - P * (L / 2 - a)
    return R, M_mid


# ---------------------------------------------------------------------------
# Phase A — SS square plate benchmark
# ---------------------------------------------------------------------------

def phase_a() -> None:
    print("\n=== PHASE A: thick-shell SS square plate vs classical solution ===")
    a, t, fc, q, uw = 6.0, 0.15, 28.0, 10.0, 24.0
    n = 12  # 0.5 m grid/mesh

    step = a / n
    piles = []
    for i in range(n + 1):
        x = i * step
        for y in (0.0, a):
            piles.append({"x": x, "y": y,
                          "restraint": [True, True, True, False, False, False]})
    for j in range(1, n):
        y = j * step
        for x in (0.0, a):
            piles.append({"x": x, "y": y,
                          "restraint": [True, True, True, False, False, False]})

    model = {
        "project": {"name": "SS plate benchmark", "unit_system": "kN_m",
                    "structure_type": "building_floor"},
        "grid": {"x_spacings": [step] * n, "y_spacings": [step] * n},
        "piles": piles,
        "slab": {"thickness": t, "concrete_fc": fc, "unit_weight": uw,
                 "mesh_size": step},
        "loads": {"dead_load": 0.0, "live_load": q},
    }
    r = api("/api/sap2000/build-from-json", model)
    rep = r["report"]
    check("A0 build clean", r["status"] == "success" and not rep["errors"],
          f"{len(rep['areas'])} shells, errors={rep['errors']}")

    op("run_analysis", save_path=OUT_DIR + r"\plate_ss_check.sdb")

    # exact statics: total vertical reaction per case
    tot = fz_totals(["LL", "DEAD"])
    exp_ll, exp_dead = q * a * a, t * a * a * uw
    check("A1 sum reactions LL == q*a^2",
          abs(tot["LL"] - exp_ll) <= 0.005 * exp_ll,
          f"SAP {tot['LL']:.2f} vs {exp_ll:.2f} kN")
    check("A2 sum reactions DEAD == slab weight",
          abs(tot["DEAD"] - exp_dead) <= 0.005 * exp_dead,
          f"SAP {tot['DEAD']:.2f} vs {exp_dead:.2f} kN")

    # center deflection vs Kirchhoff/Navier series
    E = 4700.0 * math.sqrt(fc) * 1000.0          # kPa (same formula as builder)
    D = E * t ** 3 / (12.0 * (1 - 0.2 ** 2))     # kN*m
    w_ref = 0.00406 * q * a ** 4 / D             # m (alpha for nu-independent form)

    j = op("find_joints", coords=[[a / 2, a / 2, 0.0]])["joints"][0]
    check("A3 center joint found", j["matched"],
          f"joint {j['joint']} at {j['coords']}")
    disp = op("joint_displacements", cases=["LL"], joints=[j["joint"]])
    w_sap = -disp["displacements"][0]["U3"]
    ratio = w_sap / w_ref
    # Mindlin thick shell adds a little shear deflection over Kirchhoff theory
    check("A4 center deflection vs 0.00406*q*a^4/D",
          0.95 <= ratio <= 1.08,
          f"SAP {w_sap*1000:.3f} mm vs theory {w_ref*1000:.3f} mm "
          f"(ratio {ratio:.4f})")


# ---------------------------------------------------------------------------
# Phase B — deck (thick shell) + beams + girders + columns
# ---------------------------------------------------------------------------

def phase_b() -> None:
    print("\n=== PHASE B: thick-shell deck + beams + girders + columns ===")
    Lx, By = 8.0, 3.0                 # girder bay 8 m; beams every 3 m (9 m total)
    t, fc, uw = 0.2, 28.0, 24.0
    sdl, ll = 2.0, 4.8
    H = 3.5                            # column height
    area = Lx * (3 * By)

    model = {
        "project": {"name": "Deck+beams+girders+columns", "unit_system": "kN_m",
                    "structure_type": "building_floor"},
        "grid": {"x_spacings": [Lx], "y_spacings": [By, By, By]},
        "girders": {"direction": "Y", "row_indices": [0, 1],
                    "section": {"name": "GIRD", "section_type": "W24x94"}},
        "beams": {"section": {"name": "BEAM", "section_type": "W18x35"},
                  "col_indices": [0, 1, 2, 3]},
        "piles": [],                   # supports come from the columns
        "slab": {"thickness": t, "concrete_fc": fc, "unit_weight": uw,
                 "mesh_size": 0.5},
        "loads": {"dead_load": sdl, "live_load": ll},
    }
    r = api("/api/sap2000/build-from-json", model)
    rep = r["report"]
    check("B0 build clean", r["status"] == "success" and not rep["errors"],
          f"{len(rep['frames'])} frames, {len(rep['areas'])} shells, "
          f"errors={rep['errors']}")
    fmap = frame_map(rep)

    cols = op("add_columns", positions=[[0, 0], [Lx, 0], [0, 3 * By], [Lx, 3 * By]],
              height=H, section_shape="W14x90", fix_base=True)
    check("B1 add_columns ok", cols["status"] == "ok" and len(cols["columns"]) == 4,
          f"columns {[c['name'] for c in cols['columns']]}, "
          f"bases at z={cols['base_z']}")

    op("run_analysis", save_path=OUT_DIR + r"\deck_bgc_check.sdb")

    # --- exact statics on totals (steel SW from AISC areas x 76.97 kN/m3)
    kNm3 = 76.9729
    wpm = {"W24x94": 27.7, "W18x35": 10.3, "W14x90": 26.5}
    wpm = {k: v * 0.00064516 * kNm3 for k, v in wpm.items()}   # kN/m
    exp = {
        "LL": ll * area,
        "SDL": sdl * area,
        "DEAD": (t * area * uw
                 + 2 * (3 * By) * wpm["W24x94"]      # 2 girders x 9 m
                 + 4 * Lx * wpm["W18x35"]            # 4 beams x 8 m
                 + 4 * H * wpm["W14x90"]),           # 4 columns x 3.5 m
    }
    tot = fz_totals(["LL", "SDL", "DEAD"])
    for case, tol in (("LL", 0.005), ("SDL", 0.005), ("DEAD", 0.02)):
        check(f"B2 sum reactions {case}",
              abs(tot[case] - exp[case]) <= tol * exp[case],
              f"SAP {tot[case]:.2f} vs statics {exp[case]:.2f} kN")

    # --- column base reactions: equal by double symmetry, sum == total
    bases = [c["base_joint"] for c in cols["columns"]]
    rows = op("joint_reactions", cases=["LL"], joints=bases)["reactions"]
    f3 = [row["F3"] for row in rows]
    per_col = exp["LL"] / 4
    check("B3 column reactions equal (symmetry)",
          all(abs(v - per_col) <= 0.02 * per_col for v in f3),
          f"F3 = {[f'{v:.2f}' for v in f3]} vs {per_col:.2f} kN each")
    check("B3b column reactions sum == LL total",
          abs(sum(f3) - exp["LL"]) <= 0.005 * exp["LL"],
          f"sum {sum(f3):.2f} vs {exp['LL']:.2f} kN")

    # --- stiffness-method references (src.fea) for member moments
    w_trib = ll * By                                   # 14.4 kN/m on interior beam
    EI_beam = 2.0e8 * 510 * 0.0254 ** 4                # W18x35: Ix=510 in^4, kN*m^2
    R_beam, M_beam_ref, w_mid_alone = fea_ss_beam_udl(Lx, w_trib, EI=EI_beam)
    check("B4 fea beam ref == closed form",
          abs(M_beam_ref - w_trib * Lx ** 2 / 8) < 1e-9
          and abs(R_beam - w_trib * Lx / 2) < 1e-9
          and abs(w_mid_alone - 5 * w_trib * Lx ** 4 / (384 * EI_beam)) < 1e-9,
          f"R={R_beam:.2f} kN, M_mid={M_beam_ref:.2f} kN*m (= wL^2/8), "
          f"w_mid={w_mid_alone*1000:.2f} mm (= 5wL^4/384EI)")

    R_gird, M_gird_ref = fea_ss_beam_2P(3 * By, By, R_beam)
    check("B5 fea girder ref == closed form",
          abs(M_gird_ref - R_beam * By) < 1e-9,
          f"P={R_beam:.2f} kN at 3rd points, M_mid={M_gird_ref:.2f} kN*m (= P*a)")

    # --- interior beam (y = 3 m) LL midspan moment vs tributary reference.
    # The 0.2 m slab strip is stiffer in bending than the W18x35 it sits on
    # (EI_slab(3 m) ~ 49,700 vs EI_beam ~ 42,500 kN*m^2, both at the same
    # node plane), so the shell legitimately carries most of the tributary
    # moment — the frame's share must be a plausible fraction, peaking at
    # midspan; the FULL tributary moment is the upper bound.
    E_c = 4700.0 * math.sqrt(fc) * 1000.0
    EI_slab = E_c * By * t ** 3 / 12.0
    share = EI_beam / (EI_beam + EI_slab)
    ff = op("frame_forces", cases=["LL"], frames=[fmap["B_1_0"]])["frame_forces"]
    m_max = max(ff, key=lambda row: row["M3"])
    ratio_b = m_max["M3"] / M_beam_ref
    check("B6 beam midspan LL moment vs tributary stiffness ref",
          0.10 <= ratio_b <= 1.15 and abs(m_max["station"] - Lx / 2) <= 0.75,
          f"SAP max M3 {m_max['M3']:.2f} kN*m at x={m_max['station']:.2f} "
          f"vs tributary ref {M_beam_ref:.2f} kN*m (ratio {ratio_b:.3f}; "
          f"beam EI share of beam+slab strip = {share:.2f})")

    # --- beam-shell connectivity: with the shell attached, the shared joint
    # at beam midspan must deflect far less than the beam-alone tributary
    # model (fea reference) but still a physically nonzero amount.
    jmid = op("find_joints", coords=[[Lx / 2, By, 0.0]])["joints"][0]
    check("B6b beam midspan joint found", jmid["matched"],
          f"joint {jmid['joint']} at {jmid['coords']}")
    d = op("joint_displacements", cases=["LL"], joints=[jmid["joint"]])
    w_sap_mid = -d["displacements"][0]["U3"]
    check("B6c beam-shell connectivity (deflection bounded by beam-alone ref)",
          0.02 * w_mid_alone <= w_sap_mid <= 0.9 * w_mid_alone,
          f"SAP {w_sap_mid*1000:.2f} mm vs beam-alone tributary "
          f"{w_mid_alone*1000:.2f} mm")

    # --- girder line x=0, middle segment (y=3..6): moment at y=4.5
    ff = op("frame_forces", cases=["LL"], frames=[fmap["G_Y_0_1"]])["frame_forces"]
    mid = min(ff, key=lambda row: abs(row["station"] - By / 2))
    ratio_g = mid["M3"] / M_gird_ref
    check("B7 girder midspan LL moment vs point-load stiffness ref",
          0.35 <= ratio_g <= 1.15,
          f"SAP M3 {mid['M3']:.2f} kN*m at segment station {mid['station']:.2f} "
          f"vs ref {M_gird_ref:.2f} kN*m (ratio {ratio_g:.3f})")


def main() -> int:
    api("/api/sap2000/connect", {"visible": True})
    phase_a()
    phase_b()
    n_fail = sum(1 for _, ok, _ in CHECKS if not ok)
    print(f"\n{len(CHECKS) - n_fail}/{len(CHECKS)} checks passed")
    print("RESULT: PASS" if n_fail == 0 else f"RESULT: FAIL ({n_fail} checks)")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
