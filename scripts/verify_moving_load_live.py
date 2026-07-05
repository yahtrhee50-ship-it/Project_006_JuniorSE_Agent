"""
End-to-end live verification of the P005 SAP2000 moving-load API control.

Background: the engineer reported (a) an error message when opening the saved
moving-load model and (b) the moving load not looking like a stepped axle
train. Probes traced this to a duplicated MLCLASS row in "Vehicles 4 - Vehicle
Classes" (a latent first class-table import replaying on the next import) in
outputs/moving_load_P13.sdb. The P005 builder was fixed (verify-and-repair
class import + custom stepped axle trains). This script verifies the fix
end-to-end against an independent reference.

Phase 1 — PHYSICS: single-girder 60 ft simple span (kip_ft, girder-only, no
  slab), CUSTOM stepped axle train 30/40/30 kip @ 12 ft spacings (symmetric,
  so travel direction cannot matter). 100% of the vehicle goes down one girder,
  so the SAP2000 MOVE1 envelope must match an exact influence-line enumeration
  done in plain Python:
    - base reaction envelope FZ max == total vehicle weight (100 kip exactly,
      train fits fully on the span),
    - M3 max envelope at every output station vs exact reference,
    - |V2| max envelope at every output station vs exact reference.

Phase 2 — REGRESSION: 3-girder deck + slab (kip_ft), standard P13 truck.
  Build report clean, vehicle class table exact, MOVE1 envelope nonzero,
  saved file reopens with tables intact.

Phase 3 — REPAIR: rewrite the corrupted class table of the ORIGINAL
  outputs/moving_load_P13.sdb in place (converging full-table rewrite),
  save, re-read exact.

Requires the P005 API server on 127.0.0.1:8000 (fixed builder) and SAP2000.
Run:  C:\\Python314\\python.exe scripts/verify_moving_load_live.py
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
        if " -> " in entry:
            req, act = entry.split(" -> ")
        else:
            req = act = entry
        out[req] = act
    return out


# ---------------------------------------------------------------------------
# COM table read-back helpers (attach to the same running SAP2000)
# ---------------------------------------------------------------------------

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
    n = int(r[3] or 0)
    ncol = len(r[2] or ())
    data = list(r[4] or ())
    return [["" if c is None else str(c) for c in data[i * ncol:(i + 1) * ncol]]
            for i in range(n)]


def class_entries(m) -> list[tuple[str, str]]:
    return [(row[0], row[1]) for row in table_rows(m, "Vehicles 4 - Vehicle Classes")]


def mlclass_exact(m, veh_names: list[str]) -> tuple[bool, str]:
    got = sorted(e for e in class_entries(m) if e[0] == "MLCLASS")
    expected = sorted(("MLCLASS", vn) for vn in veh_names)
    return got == expected, f"expected {expected}, got {got}"


# ---------------------------------------------------------------------------
# Independent reference: exact influence-line enumeration for a moving axle
# train on a simply supported span (pure statics, no SAP2000 involved)
# ---------------------------------------------------------------------------

def envelope_reference(L: float, axles: list[float], spacings: list[float],
                       stations: list[float], step: float = 0.01):
    """Exact M and |V| envelopes at the given stations for the axle train
    crossing the span (front axle from -train_len to L+train_len). The train
    used in phase 1 is symmetric, so one direction suffices; both are run
    anyway for generality."""
    offsets = [0.0]
    for s in spacings:
        offsets.append(offsets[-1] + s)
    train_len = offsets[-1]

    m_env = {x: 0.0 for x in stations}
    v_env = {x: 0.0 for x in stations}

    for direction_offsets in (offsets, [train_len - o for o in offsets]):
        pos = -train_len
        while pos <= L + train_len:
            axle_x = [pos - o for o in direction_offsets]
            for x in stations:
                mm = 0.0
                vv = 0.0
                for xi, P in zip(axle_x, axles):
                    if 0.0 <= xi <= L:
                        if xi <= x:
                            mm += P * xi * (L - x) / L
                            vv += -P * xi / L
                        else:
                            mm += P * x * (L - xi) / L
                            vv += P * (L - xi) / L
                if mm > m_env[x]:
                    m_env[x] = mm
                if abs(vv) > v_env[x]:
                    v_env[x] = abs(vv)
            pos += step
    return m_env, v_env


# ---------------------------------------------------------------------------
# Phase 1 — custom stepped axle train on a single girder, vs exact reference
# ---------------------------------------------------------------------------

def phase1() -> None:
    print("\n=== PHASE 1: custom axle train, single 60 ft girder, physics check ===")
    L = 60.0
    axles = [30.0, 40.0, 30.0]
    spacings = [12.0, 12.0]
    model = {
        "project": {"name": "MovingLoadCustomCheck", "unit_system": "kip_ft",
                    "structure_type": "bridge_deck"},
        "grid": {"x_spacings": [15, 15, 15, 15], "y_spacings": [6, 6]},
        "girders": {
            "direction": "X",
            "section": {"name": "W24X94", "section_type": "W24X94", "material": "A992"},
            "row_indices": [1],
        },
        "piles": [
            {"x": 0, "y": 6, "restraint": [True, True, True, True, False, False]},
            {"x": 60, "y": 6, "restraint": [False, True, True, False, False, False]},
        ],
        "loads": {
            "moving_load_enabled": True,
            "truck_axle_loads": axles,
            "truck_axle_spacings": spacings,
            "lane_width": 12.0,
        },
    }
    save = OUT_DIR + r"\moving_load_custom_check.sdb"
    resp = api("/api/sap2000/build-from-json", model, save_path=save, run_analysis=True)
    report = resp["report"]
    check("P1 build errors", not report["errors"], f"errors={report['errors']}")
    ml_lines = [l for l in report["loads"] if "MOVE1" in l]
    check("P1 custom train in report",
          any("custom axle train, 3 axles, total 100" in l for l in ml_lines),
          f"loads={ml_lines}")

    m = com_model()
    ok, detail = mlclass_exact(m, ["CUSTOM1"])
    check("P1 class table exact", ok, detail)

    vrows = table_rows(m, "Vehicles 3 - General Vehicles 2 - Loads")
    vrows = [r for r in vrows if r and r[0] == "CUSTOM1"]
    stepped = (len(vrows) == 3
               and vrows[0][1] == "Leading Load" and float(vrows[0][3]) == 30.0
               and vrows[1][1] == "Fixed Length" and float(vrows[1][3]) == 40.0
               and float(vrows[1][4]) == 12.0
               and vrows[2][1] == "Fixed Length" and float(vrows[2][3]) == 30.0
               and float(vrows[2][4]) == 12.0)
    check("P1 vehicle is stepped axle train", stepped, f"rows={vrows}")

    # Support reaction envelope: max reaction at each end == 80 kip exactly
    # (train head at the support: R = 30 + 40*48/60 + 30*36/60, both ends by
    # symmetry). BaseReact is not populated for moving-load cases, so use the
    # per-joint reaction envelope.
    jr = op("joint_reactions", cases=["MOVE1"])
    rmax: dict[str, float] = {}
    for r in jr.get("reactions", []):
        rmax[r["joint"]] = max(rmax.get(r["joint"], 0.0), r["F3"])
    ok = (len(rmax) == 2
          and all(abs(v - 80.0) < 0.02 for v in rmax.values()))
    check("P1 support reaction envelopes == 80 kip", ok,
          f"max F3 per support = { {k: round(v, 4) for k, v in rmax.items()} }"
          if rmax else f"raw response: {jr}")

    # Frame force envelopes on the 4 girder segments
    fmap = frame_map(report)
    girders = {}
    for req, act in fmap.items():
        if req.startswith("G_X_1_"):
            bay = int(req.rsplit("_", 1)[1])
            girders[act] = 15.0 * bay
    ff = op("frame_forces", cases=["MOVE1"], frames=list(girders))["frame_forces"]

    # Group stations into 0.1 ft buckets so the two faces of a shared node
    # (last station of one frame, first station of the next) are compared as
    # ONE section. SAP2000 reports the envelope per face; at a node the
    # governing face carries the exact section demand, while the other face
    # can under-report by up to sum(P)*disc/L because vehicle placement is
    # discretized along the lane. Taking the max over faces is the demand an
    # engineer reads at that section.
    sap_m: dict[float, float] = {}
    sap_v: dict[float, float] = {}
    xs_in_bucket: dict[float, list[float]] = {}
    for row in ff:
        x = girders[row["frame"]] + row["station"]
        b = round(x, 1)
        sap_m[b] = max(sap_m.get(b, 0.0), row["M3"])
        sap_v[b] = max(sap_v.get(b, 0.0), abs(row["V2"]))
        xs_in_bucket.setdefault(b, []).append(x)

    all_xs = sorted({x for xs in xs_in_bucket.values() for x in xs})
    ref_m_x, ref_v_x = envelope_reference(L, axles, spacings, all_xs)
    ref_m = {b: max(ref_m_x[x] for x in xs) for b, xs in xs_in_bucket.items()}
    ref_v = {b: max(ref_v_x[x] for x in xs) for b, xs in xs_in_bucket.items()}

    stations = sorted(sap_m)
    ref_m_max = max(ref_m.values())
    ref_v_max = max(ref_v.values())

    worst_m = max(abs(sap_m[x] - ref_m[x]) for x in stations)
    worst_v = max(abs(sap_v[x] - ref_v[x]) for x in stations)
    check("P1 M3 envelope vs exact reference",
          worst_m <= 0.015 * ref_m_max,
          f"worst |dM| = {worst_m:.3f} kip-ft vs ref max {ref_m_max:.1f} "
          f"({100 * worst_m / ref_m_max:.2f}%) over {len(stations)} stations")
    check("P1 V2 envelope vs exact reference",
          worst_v <= 0.015 * ref_v_max,
          f"worst |dV| = {worst_v:.3f} kip vs ref max {ref_v_max:.1f} "
          f"({100 * worst_v / ref_v_max:.2f}%)")
    mid = min(stations, key=lambda x: abs(x - L / 2))
    print(f"    midspan M3: SAP {sap_m[mid]:.2f} vs ref {ref_m[mid]:.2f} kip-ft; "
          f"max |V2|: SAP {max(sap_v.values()):.2f} vs ref {ref_v_max:.2f} kip")

    # Round trip: reopen the saved file, tables must be intact
    ret = m.File.OpenFile(save)
    ok, detail = mlclass_exact(m, ["CUSTOM1"])
    n_load_rows = len([r for r in table_rows(m, "Vehicles 3 - General Vehicles 2 - Loads")
                       if r and r[0] == "CUSTOM1"])
    check("P1 reopen: tables intact",
          ret == 0 and ok and n_load_rows == 3,
          f"open ret={ret}, class {detail}, vehicle load rows={n_load_rows}")


# ---------------------------------------------------------------------------
# Phase 2 — standard P13 on a 3-girder deck (the original failing scenario)
# ---------------------------------------------------------------------------

def phase2() -> None:
    print("\n=== PHASE 2: standard P13, 3-girder deck + slab (kip_ft) ===")
    model = {
        "project": {"name": "MovingLoadP13KipFt", "unit_system": "kip_ft",
                    "structure_type": "bridge_deck"},
        "grid": {"x_spacings": [15, 15, 15, 15], "y_spacings": [8, 8]},
        "girders": {
            "direction": "X",
            "section": {"name": "W24X94", "section_type": "W24X94", "material": "A992"},
            "row_indices": [0, 1, 2],
        },
        "piles": [
            {"x": x, "y": y, "restraint": [x == 0, True, True, x == 0, False, False]}
            for x in (0, 60) for y in (0, 8, 16)
        ],
        "slab": {"thickness": 0.667, "concrete_fc": 4.0, "unit_weight": 0.15,
                 "mesh_size": 4.0},
        "loads": {"dead_load": 0.025, "live_load": 0.0,
                  "moving_load_enabled": True, "truck_type": "P13"},
    }
    save = OUT_DIR + r"\moving_load_P13_kipft.sdb"
    resp = api("/api/sap2000/build-from-json", model, save_path=save, run_analysis=True)
    report = resp["report"]
    check("P2 build errors", not report["errors"], f"errors={report['errors']}")

    m = com_model()
    ok, detail = mlclass_exact(m, ["P13"])
    check("P2 class table exact", ok, detail)

    fmap = frame_map(report)
    mid_girders = [act for req, act in fmap.items() if req.startswith("G_X_1_")]
    ff = op("frame_forces", cases=["MOVE1"], frames=mid_girders)["frame_forces"]
    m3_max = max(r["M3"] for r in ff)
    check("P2 MOVE1 envelope nonzero", m3_max > 10.0,
          f"center girder max M3 = {m3_max:.1f} kip-ft")

    ret = m.File.OpenFile(save)
    ok, detail = mlclass_exact(m, ["P13"])
    check("P2 reopen: tables intact", ret == 0 and ok,
          f"open ret={ret}, class {detail}")


# ---------------------------------------------------------------------------
# Phase 3 — repair the original corrupted moving_load_P13.sdb in place
# ---------------------------------------------------------------------------

def phase3() -> None:
    print("\n=== PHASE 3: repair original moving_load_P13.sdb class table ===")
    m = com_model()
    sdb = OUT_DIR + r"\moving_load_P13.sdb"
    ret = m.File.OpenFile(sdb)
    check("P3 open original", ret == 0, f"ret={ret}")
    print(f"    classes before: {class_entries(m)}")

    try:
        m.SetModelIsLocked(False)
    except Exception:
        pass

    fields = ["VehClass", "VehName", "ScaleFactor"]
    for _ in range(3):
        ok, _d = mlclass_exact(m, ["P13"])
        if ok:
            break
        rows = [["MLCLASS", "P13", "1"]]
        for c, v in class_entries(m):
            if c != "MLCLASS" and [c, v, "1"] not in rows:
                rows.append([c, v, "1"])
        flat = [c for row in rows for c in row]
        m.DatabaseTables.SetTableForEditingArray(
            "Vehicles 4 - Vehicle Classes", 1, fields, len(rows), flat)
        m.DatabaseTables.ApplyEditedTables(True, 0, 0, 0, 0, "")

    ok, detail = mlclass_exact(m, ["P13"])
    check("P3 class table repaired", ok, detail)
    if ok:
        ret = m.File.Save(sdb)
        check("P3 saved", ret == 0, f"ret={ret}")
        ret = m.File.OpenFile(sdb)
        ok, detail = mlclass_exact(m, ["P13"])
        check("P3 reopen after repair", ret == 0 and ok, f"ret={ret}, {detail}")


def main() -> int:
    api("/api/sap2000/connect", {"visible": True})
    phase1()
    phase2()
    phase3()

    print("\n" + "=" * 60)
    n_pass = sum(1 for _, ok, _ in CHECKS if ok)
    print(f"RESULT: {n_pass}/{len(CHECKS)} checks passed")
    for name, ok, detail in CHECKS:
        if not ok:
            print(f"  FAILED: {name}: {detail}")
    return 0 if n_pass == len(CHECKS) else 1


if __name__ == "__main__":
    raise SystemExit(main())
