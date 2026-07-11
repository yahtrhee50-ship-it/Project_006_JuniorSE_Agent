"""
End-to-end live verification of the P005 `add_multistep_moving_load`
operation (multi-step STATIC vehicle stepping — distinct from the MOVE1
influence-line envelope).

Phase A — SETUP: build the standard 60 ft single-girder kip_ft model with an
  HS20 moving-load setup (gives LANE1 + general vehicle HS20-44), saved to
  .sdb. Add the multistep case via the new REST op (speed 5 ft/s, 24 s at
  1 s steps = 25 static steps, lead axle entering at station 0, t=0) with
  run_analysis=True.

Phase B — PHYSICS (the core check): per-step support reactions AND midspan
  M3 from the MSTEP1 case vs exact hand statics for the HS20 axle train
  (8/32/32 kip @ 14/14 ft) with the lead axle at a = 5*(k-1) ft on step k.
  Expected agreement: machine precision (the s2k route applies exact axle
  loads; verified in the calibration experiments).

Phase C — SECOND PATTERN + REOPEN INTEGRITY: add MSTEP2 (start_time=4 s).
  The op round-trips the saved .sdb through its text form, so a successful
  second call re-opens the file the first call wrote — a corruption gate on
  its own. Then: all cases coexist (MOVE1 envelope + MSTEP1 + MSTEP2),
  MSTEP1 physics unchanged, MSTEP2 delayed exactly 4 s, MLCLASS/vehicle
  tables intact (round-1/2 corruption lessons).

Phase D — IDEMPOTENCY: re-issue MSTEP1 with the same names; no duplicate
  rows/cases; physics unchanged.

Requires the P005 API server on 127.0.0.1:8000 (fresh code) and SAP2000.
Run:  C:\\Python314\\python.exe scripts/verify_multistep_moving_live.py
"""
from __future__ import annotations

import json
import sys
import urllib.request
from urllib.parse import urlencode

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
OUT_DIR = P005_ROOT + r"\outputs"
BASE = "http://127.0.0.1:8000"
sys.path.insert(0, P005_ROOT)

SPAN = 60.0
HS20_AXLES = [8.0, 32.0, 32.0]
HS20_SPACINGS = [14.0, 14.0]
SPEED, DURATION, DISC = 5.0, 24.0, 1.0
N_STEPS = int(DURATION / DISC) + 1

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


def frame_map(report: dict) -> dict[str, str]:
    out = {}
    for entry in report["frames"]:
        req, act = entry.split(" -> ") if " -> " in entry else (entry, entry)
        out[req] = act
    return out


# ── exact statics for the HS20 train ─────────────────────────────────────

def exact_state(a_lead: float) -> tuple[float, float, float]:
    """(R_left, R_right, M_mid) with the lead axle at a_lead; trailing
    axles behind it (a_lead - spacing sums); off-span axles carry nothing."""
    rl = rr = mm = 0.0
    d = 0.0
    for i, p in enumerate(HS20_AXLES):
        if i:
            d += HS20_SPACINGS[i - 1]
        a = a_lead - d
        if 0.0 <= a <= SPAN:
            rr += p * a / SPAN
            rl += p * (SPAN - a) / SPAN
            x = SPAN / 2
            mm += (p * a * (SPAN - x) / SPAN if a <= x
                   else p * x * (SPAN - a) / SPAN)
    return rl, rr, mm


def lead_axle_at(step: float, start_time: float = 0.0) -> float | None:
    """Lead-axle station at a step, or None while the vehicle is not yet
    applied (SAP applies it for t >= start_time only — probed live: with
    start_time=4 s, steps at t=0..3 carry ZERO load and the t=4 step has
    the lead axle exactly at the start station)."""
    t = (step - 1) * DISC
    if t < start_time:
        return None
    return SPEED * (t - start_time)


# ── physics assertions on one multistep case ─────────────────────────────

def steps_from(rows: list[dict], key_joint: str | None = None,
               frame: str | None = None, want: str = "F3") -> dict[float, float]:
    out = {}
    for r in rows:
        if key_joint is not None and r["joint"] != key_joint:
            continue
        if frame is not None and (r["frame"] != frame
                                  or abs(r["station"]) > 1e-6):
            continue
        out[r["step_num"]] = r[want]
    return out


def physics_check(tag: str, case: str, mid_frame: str,
                  start_time: float = 0.0) -> None:
    jr = op("joint_reactions", cases=[case], multistep="steps")["reactions"]
    joints = sorted({r["joint"] for r in jr})
    jinfo = op("find_joints", coords=[[0, 6, 0], [SPAN, 6, 0]])["joints"]
    j_left, j_right = jinfo[0]["joint"], jinfo[1]["joint"]
    check(f"{tag} support joints found",
          jinfo[0]["matched"] and jinfo[1]["matched"] and
          set(joints) == {j_left, j_right},
          f"left={j_left} right={j_right} (reaction joints {joints})")

    rl = steps_from(jr, key_joint=j_left)
    rr = steps_from(jr, key_joint=j_right)
    ff = op("frame_forces", cases=[case], frames=[mid_frame],
            multistep="steps")["frame_forces"]
    m3 = steps_from(ff, frame=mid_frame, want="M3")

    check(f"{tag} {case} step count", len(rl) == N_STEPS,
          f"{len(rl)} reaction steps (expected {N_STEPS})")

    worst_r = worst_m = 0.0
    for k in sorted(rl):
        a = lead_axle_at(k, start_time)
        rl_e, rr_e, mm_e = (0.0, 0.0, 0.0) if a is None else exact_state(a)
        worst_r = max(worst_r, abs(rl[k] - rl_e), abs(rr.get(k, 0.0) - rr_e))
        if k in m3:
            worst_m = max(worst_m, abs(m3[k] - mm_e))
    check(f"{tag} {case} reactions vs exact statics (all steps)",
          worst_r < 1e-3, f"worst |dR| = {worst_r:.6f} kip over "
          f"{len(rl)} steps (lead axle a = 5*(t-{start_time:g}))")
    check(f"{tag} {case} midspan M3 vs exact statics (all steps)",
          worst_m < 1e-3 and len(m3) == N_STEPS,
          f"worst |dM| = {worst_m:.6f} kip-ft over {len(m3)} steps")


# ── COM table read-backs (corruption gates) ──────────────────────────────

_conn = None


def com_model():
    global _conn
    if _conn is None:
        from src.backend.services.sap2000.connector import SAP2000Connection
        _conn = SAP2000Connection()
        _conn.connect(visible=True)
    return _conn.model


def table_rows(m, key: str) -> list[list[str]]:
    r = m.DatabaseTables.GetTableForDisplayArray(key, [], "", 0, [], 0, [])
    if r[-1] != 0:
        return []
    fields = [str(f) for f in (r[2] or ())]
    n, ncol = int(r[3] or 0), len(fields)
    data = list(r[4] or ())
    return [["" if c is None else str(c) for c in data[i*ncol:(i+1)*ncol]]
            for i in range(n)]


def main() -> int:
    print("=== PHASE A: build HS20 girder model + add multistep case ===")
    model = {
        "project": {"name": "MultistepMovingCheck", "unit_system": "kip_ft",
                    "structure_type": "bridge_deck"},
        "grid": {"x_spacings": [15, 15, 15, 15], "y_spacings": [6, 6]},
        "girders": {
            "direction": "X",
            "section": {"name": "W24X94", "section_type": "W24X94",
                        "material": "A992"},
            "row_indices": [1],
        },
        "piles": [
            {"x": 0, "y": 6, "restraint": [True, True, True, True, False, False]},
            {"x": 60, "y": 6, "restraint": [False, True, True, False, False, False]},
        ],
        "loads": {"moving_load_enabled": True, "lane_width": 12.0,
                  "truck_type": "HS20"},
    }
    save = OUT_DIR + r"\multistep_moving_check.sdb"
    resp = api("/api/sap2000/build-from-json", model, save_path=save)
    report = resp["report"]
    check("A build errors", not report["errors"], f"errors={report['errors']}")
    fmap = frame_map(report)
    mid_frame = fmap["G_X_1_2"]          # bay 2 spans 30-45 ft; station 0 = 30
    print(f"    midspan frame: G_X_1_2 -> {mid_frame}")

    ms = op("add_multistep_moving_load", vehicle="HS20-44", lane="LANE1",
            speed=SPEED, duration=DURATION, disc=DISC, station=0.0,
            start_time=0.0, pattern_name="MSTEP", case_name="MSTEP1",
            run_analysis=True)
    check("A op status ok", ms.get("status") == "ok",
          f"status={ms.get('status')}, n_steps={ms.get('n_steps')}")
    check("A op n_steps", ms.get("n_steps") == N_STEPS,
          f"n_steps={ms.get('n_steps')} (expected {N_STEPS})")
    check("A case finished", ms.get("run", {}).get("MSTEP1") == "finished",
          f"run={ms.get('run')}")

    print("\n=== PHASE B: per-step physics vs exact statics ===")
    physics_check("B", "MSTEP1", mid_frame)

    print("\n=== PHASE C: second pattern (start_time=4) + reopen integrity ===")
    ms2 = op("add_multistep_moving_load", vehicle="HS20-44", lane="LANE1",
             speed=SPEED, duration=DURATION, disc=DISC, station=0.0,
             start_time=4.0, pattern_name="MSTEP2", case_name="MSTEP2",
             run_analysis=True)
    check("C second op ok (reopens file written by first op)",
          ms2.get("status") == "ok" and ms2.get("run", {}).get("MSTEP2") == "finished",
          f"status={ms2.get('status')}, run={ms2.get('run')}")
    cases = {c["name"] for c in op("list_load_cases")["cases"]}
    check("C cases coexist", {"MOVE1", "MSTEP1", "MSTEP2"} <= cases,
          f"cases={sorted(cases)}")
    physics_check("C", "MSTEP1", mid_frame)             # unchanged
    physics_check("C", "MSTEP2", mid_frame, start_time=4.0)

    m = com_model()
    classes = sorted({(r[0], r[1]) for r in
                      table_rows(m, "Vehicles 4 - Vehicle Classes")})
    check("C MLCLASS intact", ("MLCLASS", "HS20-44") in classes,
          f"classes={classes}")
    r = m.DatabaseTables.GetAvailableTables(0, [], [], [])
    keys = [str(k) for k in (r[1] or ())]
    lib = [k for k in keys if k.startswith("Vehicles 1")]
    check("C no library vehicles", not lib, f"lib tables={lib}")

    print("\n=== PHASE D: idempotency (same pattern/case re-issued) ===")
    ms3 = op("add_multistep_moving_load", vehicle="HS20-44", lane="LANE1",
             speed=SPEED, duration=DURATION, disc=DISC, station=0.0,
             start_time=0.0, pattern_name="MSTEP", case_name="MSTEP1",
             run_analysis=True)
    check("D re-issue ok", ms3.get("status") == "ok"
          and ms3.get("run", {}).get("MSTEP1") == "finished",
          f"status={ms3.get('status')}")
    cases2 = [c["name"] for c in op("list_load_cases")["cases"]]
    check("D no duplicate cases", len(cases2) == len(set(cases2))
          and cases2.count("MSTEP1") == 1, f"cases={sorted(cases2)}")
    physics_check("D", "MSTEP1", mid_frame)

    n_fail = sum(1 for _, ok, _ in CHECKS if not ok)
    print(f"\n{'='*60}\nRESULT: {len(CHECKS) - n_fail}/{len(CHECKS)} checks passed"
          + ("" if n_fail == 0 else f" — {n_fail} FAILED"))
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
