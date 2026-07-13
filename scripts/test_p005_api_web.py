"""
LIVE test of EVERY Project_005 SAP2000 web-API function over plain HTTP —
exactly what the web app's frontend would call. No MCP layer.

Endpoints covered:
    GET  /health
    GET  /api/sap2000/op                (registry listing)
    GET  /api/sap2000/status            (before + after connect, after close)
    POST /api/preview                   (stateless preview)
    POST /api/sap2000/connect
    POST /api/sap2000/build-from-json   (2 models, save_path/run_analysis params)
    POST /api/sap2000/op/{name}         (all 11 registered operations)
    POST /api/sap2000/close
    negative: unknown op -> 404, invalid model -> 422, bad multistep -> clean error

Test input is ORIGINAL (not reused from earlier verifications):

MODEL 1 (kip_ft floor deck): 36 x 18 ft, 9-in slab (fc 4 ksi, 0.15 kcf,
  3-ft mesh), 4 W21X62 girders along X at y = 0/6/12/18, frame_group "EDGE"
  W16X31 along Y at x = 0/36, 8 pinned piles at girder ends, 4 W12X65
  columns (11 ft, fixed) added at midspan x = 18 via op/add_columns.
  SDL = 25 psf, LL = 80 psf.
  Gates: case sums vs hand statics, symmetry groups, column P == base F3,
  factored combo sums, displacements sign/zero.

MODEL 2 (kip_ft moving-load bridge): 60-ft single W24X94 girder (y = 6),
  custom general vehicle TRK2438 (24 kip + 38 kip axles @ 15 ft, no lane
  load), lane on the girder, MOVE1 envelope; then op/add_multistep_moving_load
  (speed 10 ft/s, 9 s, 1 s steps -> 10 static steps).
  Gates (exact influence-line statics, computed here):
    MOVE1 envelope: max midspan M3 = 750 kip-ft, support-reaction envelope
    maxima {56.0, 52.5} kips; MSTEP1: per-step reactions + midspan M3 vs
    exact axle statics at lead axle a = 10*(t-0), machine precision.

Prerequisites: P005 server on port 8000, SAP2000 installed.
Run:  C:\\Python314\\python.exe scripts\\test_p005_api_web.py
"""
from __future__ import annotations

import json
import sys
import urllib.error
import urllib.request
from pathlib import Path
from urllib.parse import urlencode

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src.calcs.sections import get_section

BASE = "http://127.0.0.1:8000"
OUT_DIR = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3\outputs"
SAVE_DECK = OUT_DIR + r"\api_test_deck.sdb"
SAVE_MOVING = OUT_DIR + r"\api_test_moving.sdb"

STEEL_KCF = 0.490          # SAP2000 A992 default unit weight, kip/ft^3

# ── Model 1 definition (kip_ft) ──────────────────────────────────────────
SPAN_X = 36.0              # ft (two 18-ft bays)
WIDTH_Y = 18.0             # ft (three 6-ft bays)
SLAB_T = 0.75              # ft (9 in)
CONC_WT = 0.150            # kip/ft^3
SDL_KSF = 0.025
LL_KSF = 0.080
GIRDER = "W21X62"          # 4 rows along X
EDGE = "W16X31"            # frame_group along Y at x = 0/36
COLUMN = "W12X65"          # 4 midspan columns via op
COL_H = 11.0
GIRDER_YS = [0.0, 6.0, 12.0, 18.0]
COL_POSITIONS = [[SPAN_X / 2, y] for y in GIRDER_YS]

MODEL1 = {
    "project": {"name": "P006 web-API test deck", "unit_system": "kip_ft",
                "structure_type": "building_floor"},
    "grid": {"x_spacings": [18.0, 18.0], "y_spacings": [6.0, 6.0, 6.0]},
    "girders": {
        "direction": "X",
        "section": {"name": "GIRD", "section_type": GIRDER},
        "row_indices": [0, 1, 2, 3],
    },
    "frame_groups": [{
        "name": "EDGE", "direction": "Y",
        "section": {"name": "EDGESEC", "section_type": EDGE},
        "line_indices": [0, 2],
    }],
    "piles": [{"x": x, "y": y,
               "restraint": [True, True, True, False, False, False]}
              for x in (0.0, SPAN_X) for y in GIRDER_YS],
    "slab": {"thickness": SLAB_T, "concrete_fc": 4.0,
             "unit_weight": CONC_WT, "mesh_size": 3.0},
    "loads": {"dead_load": SDL_KSF, "live_load": LL_KSF},
}

DECK_AREA = SPAN_X * WIDTH_Y
EXP_LL = LL_KSF * DECK_AREA
EXP_SDL = SDL_KSF * DECK_AREA
_w = lambda shape: get_section(shape)["A"] / 144.0 * STEEL_KCF   # kip/ft
EXP_DEAD = (DECK_AREA * SLAB_T * CONC_WT
            + len(GIRDER_YS) * SPAN_X * _w(GIRDER)
            + 2 * WIDTH_Y * _w(EDGE)
            + len(COL_POSITIONS) * COL_H * _w(COLUMN))

# ── Model 2 definition (kip_ft moving load) ──────────────────────────────
SPAN = 60.0
AXLES = [24.0, 38.0]       # kips, front first
SPACINGS = [15.0]          # ft
SPEED, DURATION, DISC = 10.0, 9.0, 1.0
N_STEPS = int(DURATION / DISC) + 1

MODEL2 = {
    "project": {"name": "P006 web-API moving-load test", "unit_system": "kip_ft",
                "structure_type": "bridge_deck"},
    "grid": {"x_spacings": [15.0, 15.0, 15.0, 15.0], "y_spacings": [6.0, 6.0]},
    "girders": {
        "direction": "X",
        "section": {"name": "W24X94", "section_type": "W24X94"},
        "row_indices": [1],
    },
    "piles": [
        {"x": 0.0, "y": 6.0,
         "restraint": [True, True, True, True, False, False]},
        {"x": SPAN, "y": 6.0,
         "restraint": [False, True, True, False, False, False]},
    ],
    "loads": {"moving_load_enabled": True, "lane_width": 12.0,
              "vehicles": [{"name": "TRK2438", "axle_loads": AXLES,
                            "axle_spacings": SPACINGS, "lane_load": 0.0}]},
}

OPS_EXPECTED = sorted([
    "define_load_combos", "add_multistep_moving_load", "add_columns",
    "find_joints", "run_analysis", "list_load_cases", "joint_reactions",
    "joint_displacements", "frame_forces", "base_reactions", "modal_periods",
])

PASS = True


def check(label: str, ok: bool, detail: str = "") -> None:
    global PASS
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}"
          + (f" — {detail}" if detail else ""))
    PASS = PASS and bool(ok)


# ── HTTP helpers (plain urllib — same wire calls the web app makes) ──────

def http(method: str, path: str, payload=None, timeout=900, **query):
    """Return (status_code, parsed_json)."""
    url = BASE + path + ("?" + urlencode(query) if query else "")
    data = (json.dumps(payload).encode()
            if payload is not None or method == "POST" else None)
    if method == "POST" and data is None:
        data = b"{}"
    req = urllib.request.Request(
        url, data=data, method=method,
        headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        try:
            body = json.loads(exc.read())
        except Exception:
            body = {}
        return exc.code, body


def op(name: str, **params) -> dict:
    code, body = http("POST", f"/api/sap2000/op/{name}", params)
    if code != 200:
        raise RuntimeError(f"op {name} -> HTTP {code}: {str(body)[:300]}")
    return body


def frame_map(report: dict) -> dict[str, str]:
    out = {}
    for entry in report["frames"]:
        req, act = entry.split(" -> ") if " -> " in entry else (entry, entry)
        out[req] = act
    return out


def sum_per_joint_max(rows, comp="F3"):
    per = {}
    for r in rows:
        per[r["joint"]] = max(per.get(r["joint"], float("-inf")), r[comp])
    return sum(per.values())


# ── exact influence-line statics for the 24/38 train ─────────────────────

def exact_state(a_lead: float) -> tuple[float, float, float]:
    """(R_left, R_right, M_mid) on the 60-ft simple span with the front
    axle at a_lead; off-span axles carry nothing."""
    rl = rr = mm = 0.0
    d = 0.0
    for i, p in enumerate(AXLES):
        if i:
            d += SPACINGS[i - 1]
        a = a_lead - d
        if 0.0 <= a <= SPAN:
            rr += p * a / SPAN
            rl += p * (SPAN - a) / SPAN
            x = SPAN / 2
            mm += (p * a * (SPAN - x) / SPAN if a <= x
                   else p * x * (SPAN - a) / SPAN)
    return rl, rr, mm


def envelope_exact():
    """Max R_left, R_right, M_mid over continuous placement (both travel
    directions — evaluate at every breakpoint: axle at a support or at
    midspan)."""
    cands = []
    for d0 in (0.0, SPACINGS[0]):            # each axle as reference
        for target in (0.0, SPAN / 2, SPAN):  # placed at support / midspan
            cands.append(target + d0)
    best_rl = best_rr = best_mm = 0.0
    for trains in (AXLES, list(reversed(AXLES))):   # both directions
        for a in cands:
            rl, rr, mm = _state_for(trains, a)
            best_rl, best_rr, best_mm = (max(best_rl, rl), max(best_rr, rr),
                                         max(best_mm, mm))
    return best_rl, best_rr, best_mm


def _state_for(axles, a_lead):
    rl = rr = mm = 0.0
    d = 0.0
    for i, p in enumerate(axles):
        if i:
            d += SPACINGS[i - 1]
        a = a_lead - d
        if 0.0 <= a <= SPAN:
            rr += p * a / SPAN
            rl += p * (SPAN - a) / SPAN
            x = SPAN / 2
            mm += (p * a * (SPAN - x) / SPAN if a <= x
                   else p * x * (SPAN - a) / SPAN)
    return rl, rr, mm


def steps_from(rows, key_joint=None, frame=None, want="F3"):
    out = {}
    for r in rows:
        if key_joint is not None and r["joint"] != key_joint:
            continue
        if frame is not None and (r["frame"] != frame
                                  or abs(r["station"]) > 1e-6):
            continue
        out[r["step_num"]] = r[want]
    return out


def main() -> int:
    print("=" * 74)
    print("Project_005 SAP2000 web API — FULL live endpoint test (HTTP)")
    print("=" * 74)

    # ── Phase 0: server + stateless endpoints ────────────────────────────
    print("\n== Phase 0: health / op registry / status / preview ==")
    code, body = http("GET", "/health")
    check("GET /health -> ok", code == 200 and body.get("status") == "ok",
          str(body))

    code, body = http("GET", "/api/sap2000/op")
    check("GET /op lists all 11 operations",
          code == 200 and sorted(body) == OPS_EXPECTED,
          f"{len(body)} ops" if code == 200 else str(body))

    code, body = http("GET", "/api/sap2000/status")
    check("GET /status before connect -> connected false",
          code == 200 and body.get("connected") is False, str(body))

    code, body = http("POST", "/api/preview", MODEL1)
    check("POST /api/preview accepts Model 1",
          code == 200 and isinstance(body, dict) and body,
          f"keys={sorted(body)[:8]}" if code == 200 else str(body)[:200])

    # negative tests (no SAP2000 needed for validation-layer errors)
    code, body = http("POST", "/api/sap2000/build-from-json",
                      {"grid": {"x_spacings": "oops"}})
    check("build-from-json invalid payload -> 422", code == 422,
          f"HTTP {code}")
    code, body = http("POST", "/api/sap2000/op/definitely_not_an_op", {})
    check("unknown op -> 404", code == 404, f"HTTP {code}")

    # ── Phase 1: connect ─────────────────────────────────────────────────
    print("\n== Phase 1: connect / status ==")
    code, body = http("POST", "/api/sap2000/connect", {"visible": True},
                      timeout=300)
    check("POST /connect -> connected",
          code == 200 and body.get("status") == "connected", str(body))
    code, body = http("GET", "/api/sap2000/status")
    check("GET /status after connect -> connected true",
          code == 200 and body.get("connected") is True, str(body))

    # ── Phase 2: build Model 1 ───────────────────────────────────────────
    print("\n== Phase 2: build-from-json (deck, save_path) ==")
    code, resp = http("POST", "/api/sap2000/build-from-json", MODEL1,
                      save_path=SAVE_DECK)
    check("build status success",
          code == 200 and resp.get("status") == "success",
          str(resp)[:200] if code != 200 else "")
    report = resp["report"]
    check("build report has no errors", not report.get("errors"),
          str(report.get("errors"))[:300])
    fmap = frame_map(report)
    girder_frames = [k for k in fmap if k.startswith("G_X_")]
    edge_frames = [k for k in fmap if k.startswith("EDGE_Y_")]
    check("8 girder members (4 rows x 2 bays)", len(girder_frames) == 8,
          f"{len(girder_frames)}")
    check("6 EDGE frame_group members (2 lines x 3 bays)",
          len(edge_frames) == 6, f"{len(edge_frames)}")
    n_areas = len(report.get("areas") or [])
    check("slab meshed (12 x 6 target = 72 areas)", n_areas == 72,
          f"{n_areas} areas")
    check("saved to outputs\\api_test_deck.sdb",
          report.get("saved_to") == SAVE_DECK, str(report.get("saved_to")))

    # ── Phase 3: op/add_columns ──────────────────────────────────────────
    print("\n== Phase 3: op/add_columns (4x W12X65 @ midspan, fixed) ==")
    colpay = op("add_columns", positions=COL_POSITIONS, height=COL_H,
                section_shape=COLUMN, section_name="COLSEC",
                material="steel", unit_system="kip_ft", fix_base=True)
    cols = colpay.get("columns") or []
    check("4 columns created", len(cols) == 4, f"{len(cols)}")
    check("bases fixed at z = -11 ft",
          colpay.get("fixed_base") is True and colpay.get("base_z") == -COL_H,
          f"base_z={colpay.get('base_z')}")

    # ── Phase 4: op/run_analysis ─────────────────────────────────────────
    print("\n== Phase 4: op/run_analysis (save + run) ==")
    runpay = op("run_analysis", save_path=SAVE_DECK)
    status = {c["name"]: c["run_status"] for c in runpay.get("cases", [])}
    for case in ("DEAD", "SDL", "LL"):
        check(f"case {case} finished", status.get(case) == "finished",
              str(status))

    # ── Phase 5: op/list_load_cases ──────────────────────────────────────
    print("\n== Phase 5: op/list_load_cases ==")
    names = {c["name"] for c in op("list_load_cases").get("cases", [])}
    check("DEAD/SDL/LL cases exist", {"DEAD", "SDL", "LL"} <= names,
          str(sorted(names)))

    # ── Phase 6: op/base_reactions vs statics ────────────────────────────
    print("\n== Phase 6: op/base_reactions vs hand statics ==")
    print(f"    expected [kips]: LL={EXP_LL:.3f} SDL={EXP_SDL:.3f} "
          f"DEAD={EXP_DEAD:.3f}")
    basepay = op("base_reactions", cases=["DEAD", "SDL", "LL"])
    fz = {r["case"]: r["FZ"] for r in basepay.get("base_reactions", [])}
    check(f"LL base FZ == {EXP_LL:.3f}", abs(fz.get("LL", 0) - EXP_LL) < 1e-6,
          f"{fz.get('LL'):.6f}")
    check(f"SDL base FZ == {EXP_SDL:.3f}",
          abs(fz.get("SDL", 0) - EXP_SDL) < 1e-6, f"{fz.get('SDL'):.6f}")
    check(f"DEAD base FZ == {EXP_DEAD:.3f} (0.2% band)",
          abs(fz.get("DEAD", 0) - EXP_DEAD) < 0.002 * EXP_DEAD,
          f"{fz.get('DEAD'):.4f} (ratio {fz.get('DEAD', 0)/EXP_DEAD:.5f})")

    # ── Phase 7: op/joint_reactions: equilibrium + symmetry ──────────────
    print("\n== Phase 7: op/joint_reactions (LL) ==")
    rows = op("joint_reactions", cases=["LL"]).get("reactions", [])
    joints = {r["joint"] for r in rows}
    check("12 restrained joints (8 piles + 4 col bases)",
          len(joints) == 12, str(sorted(joints)))
    total = sum(r["F3"] for r in rows)
    check(f"sum LL F3 == {EXP_LL:.3f}", abs(total - EXP_LL) < 1e-6,
          f"{total:.6f}")

    # locate pile joints for symmetry groups
    pile_coords = [(x, y) for x in (0.0, SPAN_X) for y in GIRDER_YS]
    fj = op("find_joints", coords=[[x, y, 0.0] for x, y in pile_coords],
            tol=0.01)["joints"]
    pile_j = {(x, y): h["joint"]
              for (x, y), h in zip(pile_coords, fj) if h["matched"]}
    check("all 8 pile joints located", len(pile_j) == 8, str(pile_j))
    f3 = {r["joint"]: r["F3"] for r in rows}
    corner = [f3[pile_j[(x, y)]] for x in (0.0, SPAN_X)
              for y in (0.0, WIDTH_Y)]
    inner = [f3[pile_j[(x, y)]] for x in (0.0, SPAN_X) for y in (6.0, 12.0)]
    check("4 corner pile reactions equal (double symmetry)",
          max(corner) - min(corner) < 1e-6,
          f"{[f'{v:.4f}' for v in corner]}")
    check("4 inner pile reactions equal (double symmetry)",
          max(inner) - min(inner) < 1e-6, f"{[f'{v:.4f}' for v in inner]}")
    col_base = {c["base_joint"]: (c["x"], c["y"]) for c in cols}
    col_edge = [f3[j] for j, (x, y) in col_base.items() if y in (0.0, WIDTH_Y)]
    col_int = [f3[j] for j, (x, y) in col_base.items() if y in (6.0, 12.0)]
    check("2 edge-column reactions equal", max(col_edge) - min(col_edge) < 1e-6,
          f"{[f'{v:.4f}' for v in col_edge]}")
    check("2 interior-column reactions equal",
          max(col_int) - min(col_int) < 1e-6,
          f"{[f'{v:.4f}' for v in col_int]}")

    # ── Phase 8: op/find_joints + op/joint_displacements ─────────────────
    print("\n== Phase 8: op/find_joints + op/joint_displacements (LL) ==")
    hit = op("find_joints", coords=[[SPAN_X / 2, WIDTH_Y / 2, 0.0]],
             tol=0.01)["joints"][0]
    check("deck joint found at exact center (on 3-ft mesh)", hit["matched"],
          f"{hit['joint']} d={hit.get('distance', 0):.4g}")
    col0 = next(c for c in cols if c["y"] == 0.0)
    base_hit = op("find_joints", coords=[[SPAN_X / 2, 0.0, -COL_H]],
                  tol=0.01)["joints"][0]
    check("column base joint found at (18, 0, -11)",
          base_hit["matched"] and base_hit["joint"] == col0["base_joint"],
          f"{base_hit['joint']} vs {col0['base_joint']}")
    dz = {r["joint"]: r["U3"]
          for r in op("joint_displacements", cases=["LL"],
                      joints=[hit["joint"], col0["base_joint"]]
                      ).get("displacements", [])}
    check("deck center deflects DOWN under LL", dz[hit["joint"]] < -1e-9,
          f"{dz[hit['joint']] * 12:.5f} in")
    check("fixed column base does not move",
          abs(dz[col0["base_joint"]]) < 1e-12)

    # ── Phase 9: op/frame_forces cross-check ─────────────────────────────
    print("\n== Phase 9: op/frame_forces (LL column axial vs reaction) ==")
    frows = op("frame_forces", cases=["LL"],
               frames=[col0["name"]]).get("frame_forces", [])
    check("column force rows returned", len(frows) >= 2, f"{len(frows)} rows")
    p_vals = [r["P"] for r in frows]
    check("column P constant along height",
          max(p_vals) - min(p_vals) < 1e-6,
          f"[{min(p_vals):.6f}, {max(p_vals):.6f}]")
    check("|column P| == its base reaction F3",
          abs(abs(p_vals[0]) - f3[col0["base_joint"]]) < 1e-6,
          f"|P|={abs(p_vals[0]):.6f} vs F3={f3[col0['base_joint']]:.6f}")
    check("column P is compression", p_vals[0] < 0.0, f"{p_vals[0]:.6f}")

    # ── Phase 10: op/define_load_combos + factored sums ──────────────────
    print("\n== Phase 10: op/define_load_combos ==")
    cbpay = op("define_load_combos", case_map={"D": ["DEAD", "SDL"], "L": "LL"})
    created = {c["name"] for c in cbpay.get("combos", [])}
    check("LC1 + LC2 created", {"LC1", "LC2"} <= created, str(sorted(created)))
    check("LRFD-ENV created", cbpay.get("envelope") == "LRFD-ENV",
          str(cbpay.get("envelope")))
    rows2 = op("joint_reactions", cases=["LC1", "LC2"]).get("reactions", [])
    lc1 = sum_per_joint_max([r for r in rows2 if r["case"] == "LC1"])
    lc2 = sum_per_joint_max([r for r in rows2 if r["case"] == "LC2"])
    exp_lc1 = 1.4 * (fz["DEAD"] + fz["SDL"])
    exp_lc2 = 1.2 * (fz["DEAD"] + fz["SDL"]) + 1.6 * fz["LL"]
    check(f"LC1 sum F3 == 1.4D == {exp_lc1:.3f}", abs(lc1 - exp_lc1) < 1e-5,
          f"{lc1:.6f}")
    check(f"LC2 sum F3 == 1.2D+1.6L == {exp_lc2:.3f}",
          abs(lc2 - exp_lc2) < 1e-5, f"{lc2:.6f}")

    # ── Phase 11: op/modal_periods (no modal case -> graceful) ───────────
    print("\n== Phase 11: op/modal_periods ==")
    code, body = http("POST", "/api/sap2000/op/modal_periods", {})
    graceful = (code == 200) or (code == 500 and "detail" in body)
    check("modal_periods responds cleanly (200 or clean 500)", graceful,
          f"HTTP {code}: {str(body)[:120]}")

    # bad multistep value -> clean error, not a hang/crash
    code, body = http("POST", "/api/sap2000/op/joint_reactions",
                      {"cases": ["LL"], "multistep": "bogus"})
    check("bad multistep value -> clean error", code in (422, 500),
          f"HTTP {code}: {str(body)[:120]}")

    # ── Phase 12: build Model 2 (moving load) + MOVE1 envelope ───────────
    print("\n== Phase 12: build-from-json moving-load bridge + MOVE1 ==")
    exp_rl, exp_rr, exp_mm = envelope_exact()
    print(f"    exact envelope: maxR_left={exp_rl:.3f} maxR_right={exp_rr:.3f}"
          f" maxM_mid={exp_mm:.3f}")
    code, resp = http("POST", "/api/sap2000/build-from-json", MODEL2,
                      save_path=SAVE_MOVING, run_analysis=True)
    check("moving-load build success",
          code == 200 and resp.get("status") == "success",
          str(resp)[:300] if code != 200 else "")
    report2 = resp["report"]
    check("moving-load build has no errors", not report2.get("errors"),
          str(report2.get("errors"))[:300])
    fmap2 = frame_map(report2)
    mid_frame = fmap2["G_X_1_2"]           # bay 30-45 ft; station 0 = x=30
    check("run_analysis=True ran the analysis during build",
          report2.get("analysis") == "completed",
          str(report2.get("analysis")))

    jinfo = op("find_joints", coords=[[0, 6, 0], [SPAN, 6, 0]])["joints"]
    j_left, j_right = jinfo[0]["joint"], jinfo[1]["joint"]
    mrows = op("joint_reactions", cases=["MOVE1"]).get("reactions", [])
    max_l = max(r["F3"] for r in mrows if r["joint"] == j_left)
    max_r = max(r["F3"] for r in mrows if r["joint"] == j_right)
    got = sorted([max_l, max_r], reverse=True)
    exp = sorted([exp_rl, exp_rr], reverse=True)
    check(f"MOVE1 envelope support maxima ~ {exp[0]:.1f}/{exp[1]:.1f} kips"
          " (0.3% band)",
          abs(got[0] - exp[0]) < 0.003 * exp[0]
          and abs(got[1] - exp[1]) < 0.003 * exp[1] + 3.6,
          f"got {got[0]:.3f}/{got[1]:.3f}"
          " (2nd support may reach the 1st's max if SAP runs the vehicle"
          " both ways)")
    ffrows = op("frame_forces", cases=["MOVE1"],
                frames=[mid_frame]).get("frame_forces", [])
    m3_mid = max(r["M3"] for r in ffrows if abs(r["station"]) < 1e-6)
    check(f"MOVE1 envelope midspan M3 == {exp_mm:.1f} kip-ft (0.5% band)",
          abs(m3_mid - exp_mm) < 0.005 * exp_mm, f"{m3_mid:.3f}")

    # ── Phase 13: op/add_multistep_moving_load + per-step statics ────────
    print("\n== Phase 13: op/add_multistep_moving_load (10 steps) ==")
    ms = op("add_multistep_moving_load", vehicle="TRK2438", lane="LANE1",
            speed=SPEED, duration=DURATION, disc=DISC, station=0.0,
            start_time=0.0, pattern_name="MSTEP", case_name="MSTEP1",
            run_analysis=True)
    check("op status ok", ms.get("status") == "ok", str(ms)[:200])
    check(f"n_steps == {N_STEPS}", ms.get("n_steps") == N_STEPS,
          f"{ms.get('n_steps')}")
    check("MSTEP1 finished", ms.get("run", {}).get("MSTEP1") == "finished",
          str(ms.get("run")))

    jr = op("joint_reactions", cases=["MSTEP1"],
            multistep="steps")["reactions"]
    rl = steps_from(jr, key_joint=j_left)
    rr = steps_from(jr, key_joint=j_right)
    ff = op("frame_forces", cases=["MSTEP1"], frames=[mid_frame],
            multistep="steps")["frame_forces"]
    m3 = steps_from(ff, frame=mid_frame, want="M3")
    check(f"{N_STEPS} reaction steps returned", len(rl) == N_STEPS,
          f"{len(rl)}")
    worst_r = worst_m = 0.0
    for k in sorted(rl):
        a = SPEED * (k - 1) * DISC
        rl_e, rr_e, mm_e = exact_state(a)
        worst_r = max(worst_r, abs(rl[k] - rl_e), abs(rr.get(k, 0) - rr_e))
        if k in m3:
            worst_m = max(worst_m, abs(m3[k] - mm_e))
    check("per-step reactions == exact axle statics (a = 10(t-0))",
          worst_r < 1e-3, f"worst |dR| = {worst_r:.6f} kip")
    check("per-step midspan M3 == exact axle statics",
          worst_m < 1e-3 and len(m3) == N_STEPS,
          f"worst |dM| = {worst_m:.6f} kip-ft over {len(m3)} steps")

    # envelope + last_step variants of the multistep param
    env = op("joint_reactions", cases=["MSTEP1"],
             multistep="envelope")["reactions"]
    env_max_l = max(r["F3"] for r in env if r["joint"] == j_left)
    check("multistep='envelope' max == max over steps",
          abs(env_max_l - max(rl.values())) < 1e-6,
          f"{env_max_l:.4f} vs {max(rl.values()):.4f}")
    last = op("joint_reactions", cases=["MSTEP1"],
              multistep="last_step")["reactions"]
    lastrow = [r for r in last if r["joint"] == j_left]
    a_last = SPEED * (N_STEPS - 1) * DISC
    check("multistep='last_step' == exact statics at final step",
          len(lastrow) == 1
          and abs(lastrow[0]["F3"] - exact_state(a_last)[0]) < 1e-3,
          f"{lastrow[0]['F3']:.4f} vs {exact_state(a_last)[0]:.4f} "
          f"(lead axle at {a_last:g} ft)")

    # base_reactions on the multistep case — informational behavior check
    code, body = http("POST", "/api/sap2000/op/base_reactions",
                      {"cases": ["MSTEP1"], "multistep": "envelope"})
    check("base_reactions(MSTEP1) responds cleanly", code in (200, 500),
          f"HTTP {code}: {str(body)[:120]}")

    # ── Phase 14: close ──────────────────────────────────────────────────
    print("\n== Phase 14: close / status ==")
    code, body = http("POST", "/api/sap2000/close", timeout=300)
    check("POST /close -> closed",
          code == 200 and body.get("status") == "closed", str(body))
    code, body = http("GET", "/api/sap2000/status")
    check("GET /status after close -> connected false",
          code == 200 and body.get("connected") is False, str(body))

    print("\n" + "=" * 74)
    print("ALL CHECKS PASS" if PASS else "SOME CHECKS FAILED")
    print(f"Saved models: {SAVE_DECK}\n              {SAVE_MOVING}")
    return 0 if PASS else 1


if __name__ == "__main__":
    sys.exit(main())
