"""
End-to-end live verification of the P005 SAP2000 moving-load API control.

ROUND 2 (2026-07-09). Round 1 verified only the custom general-vehicle path
with physics; the standard-truck path imported a SAP2000 LIBRARY vehicle
("Vehicles 1 - Standard Vehicles"), which a non-Bridge license strips to a
FLAT load with a CSiBridge conversion warning on open. The deck models were
smoke-checked only, so that shipped. P005 commit 61a6a4f rebuilt ALL trucks
as general vehicles (Vehicles 2/3, AASHTO registry); Caltrans permit trucks
(P5-P13) now require engineer-supplied axle data. This script closes the
verification gap:

Phase 1 — CUSTOM-TRAIN PHYSICS (regression): single-girder 60 ft simple span
  (kip_ft), custom stepped axle train 30/40/30 kip @ 12 ft. MOVE1 envelope
  (M3, |V2|, reactions) vs exact influence-line enumeration in plain Python.

Phase 2 — HS20 PHYSICS (the gap-closer): same girder-only span, standard
  truck_type "HS20" (asymmetric 8/32/32 kip @ 14/14 ft). Proves the
  standard->general path analyzes as DISCRETE AXLES, not a flat load:
  envelope vs exact influence lines, plus table-shape assertions
  ("Vehicles 1" EMPTY = no CSiBridge-warning features; "Vehicles 2/3" hold
  the stepped general vehicle).

Phase 3 — HL-93 SHAPE + MIDSPAN: truck+tandem general vehicles, both in
  MLCLASS, 0.64 klf lane load as InterUnif; midspan envelope compared to the
  AASHTO code-intent value (max(truck, tandem) at midspan + w*L^2/8).

Phase 4 — HS20 DECK: 3-girder deck + slab (the scenario whose smoke checks
  missed the round-1 bug). Build clean, Vehicles 1 empty, Vehicles 2
  populated, MLCLASS exact, envelope nonzero, reopen with tables intact.

Phase 5 — CALTRANS GUARD: truck_type "P13" with no axle data must return the
  actionable error (never a silent library import).

NOT COVERED (engineer decision 2026-07-09): rebuilding moving_load_P13*.sdb
is deferred until the engineer source-confirms a P13 axle train vs the
Caltrans BDA. Those two files remain broken deliverables until then.

Requires the P005 API server on 127.0.0.1:8000 and SAP2000.
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


def table_fields_rows(m, key: str) -> tuple[list[str], list[list[str]]]:
    r = m.DatabaseTables.GetTableForDisplayArray(key, [], "", 0, [], 0, [])
    if r[-1] != 0:
        return [], []          # table absent/empty
    fields = [str(f) for f in (r[2] or ())]
    n = int(r[3] or 0)
    ncol = len(fields)
    data = list(r[4] or ())
    rows = [["" if c is None else str(c) for c in data[i * ncol:(i + 1) * ncol]]
            for i in range(n)]
    return fields, rows


def table_rows(m, key: str) -> list[list[str]]:
    return table_fields_rows(m, key)[1]


def class_entries(m) -> list[tuple[str, str]]:
    return [(row[0], row[1]) for row in table_rows(m, "Vehicles 4 - Vehicle Classes")]


def mlclass_exact(m, veh_names: list[str]) -> tuple[bool, str]:
    got = sorted(e for e in class_entries(m) if e[0] == "MLCLASS")
    expected = sorted(("MLCLASS", vn) for vn in veh_names)
    return got == expected, f"expected {expected}, got {got}"


def check_no_library_vehicles(m, tag: str) -> None:
    """No 'Vehicles 1 - Standard Vehicles' table in the model is the
    detectable proxy for 'no CSiBridge conversion warning on open' — nothing
    to convert. This is the assertion whose absence let the round-1 bug ship.
    NOTE: membership in GetAvailableTables is the only reliable test —
    GetTableForDisplayArray on an absent key returns ret=0 with the Program
    Control table's row as silent fallback (probed live on SAP2000 27.1)."""
    r = m.DatabaseTables.GetAvailableTables(0, [], [], [])
    keys = [str(k) for k in (r[1] or ())]
    lib = [k for k in keys if k.startswith("Vehicles 1")]
    check(f"{tag} Vehicles 1 (library) absent", r[-1] == 0 and not lib,
          f"library vehicle table present: {lib}" if lib
          else f"no library vehicles among {len(keys)} tables "
               f"(no conversion warning possible)")


def general_vehicle_load_rows(m, veh_name: str) -> list[dict]:
    """Vehicles 3 rows for one vehicle as dicts keyed by field name."""
    fields, rows = table_fields_rows(m, "Vehicles 3 - General Vehicles 2 - Loads")
    out = []
    for r in rows:
        d = dict(zip(fields, r))
        if d.get("VehName") == veh_name:
            out.append(d)
    return out


def assert_stepped_vehicle(tag: str, m, veh_name: str, axles: list[float],
                           spacings: list[float], unif: float = 0.0) -> None:
    """The vehicle definition itself must be a stepped axle train: a Leading
    Load axle, Fixed Length axles at the given spacings, and — when a lane
    load rides with the vehicle — a Trailing Load row so the uniform load
    extends behind the last axle too."""
    rows = general_vehicle_load_rows(m, veh_name)
    n_expected = len(axles) + (1 if unif else 0)
    ok = len(rows) == n_expected
    if ok:
        ok = (rows[0]["LoadType"] == "Leading Load"
              and abs(float(rows[0]["InterAxle"]) - axles[0]) < 1e-6)
        for r, a, s in zip(rows[1:], axles[1:], spacings):
            ok = ok and (r["LoadType"] == "Fixed Length"
                         and abs(float(r["InterAxle"]) - a) < 1e-6
                         and abs(float(r["InterMinD"]) - s) < 1e-6)
        if unif:
            ok = ok and rows[-1]["LoadType"] == "Trailing Load"
        for r in rows:
            ok = ok and abs(float(r.get("InterUnif") or 0.0) - unif) < 1e-9
    check(f"{tag} {veh_name} is stepped axle train", ok,
          f"{len(rows)} load rows "
          f"{[(r['LoadType'], r['InterAxle'], r.get('InterMinD'), r.get('InterUnif')) for r in rows]}")


# ---------------------------------------------------------------------------
# Independent reference: exact influence-line enumeration for a moving axle
# train on a simply supported span (pure statics, no SAP2000 involved)
# ---------------------------------------------------------------------------

def envelope_reference(L: float, axles: list[float], spacings: list[float],
                       stations: list[float], step: float = 0.01):
    """Exact M and |V| envelopes at the given stations for the axle train
    crossing the span in BOTH directions (front axle from -train_len to
    L+train_len)."""
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


def reaction_reference(L: float, axles: list[float], spacings: list[float],
                       step: float = 0.01) -> float:
    """Exact max simple-span support reaction for the axle train (both
    directions; by span symmetry the two supports envelope identically)."""
    offsets = [0.0]
    for s in spacings:
        offsets.append(offsets[-1] + s)
    train_len = offsets[-1]
    rmax = 0.0
    for direction_offsets in (offsets, [train_len - o for o in offsets]):
        pos = -train_len
        while pos <= L + train_len:
            r = sum(P * (L - (pos - o)) / L
                    for o, P in zip(direction_offsets, axles)
                    if 0.0 <= pos - o <= L)
            rmax = max(rmax, r)
            pos += step
    return rmax


# ---------------------------------------------------------------------------
# Shared single-girder model + physics comparison
# ---------------------------------------------------------------------------

GIRDER_L = 60.0


def girder_model(name: str, loads: dict) -> dict:
    return {
        "project": {"name": name, "unit_system": "kip_ft",
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
        "loads": {"moving_load_enabled": True, "lane_width": 12.0, **loads},
    }


def girder_envelopes(report: dict) -> tuple[dict[float, float], dict[float, float],
                                            dict[float, list[float]]]:
    """SAP MOVE1 envelopes on the 4 girder segments, bucketed to 0.1 ft so
    the two faces of a shared node are compared as ONE section (SAP reports
    the envelope per face; the governing face carries the exact section
    demand — see round-1 notes)."""
    fmap = frame_map(report)
    girders = {}
    for req, act in fmap.items():
        if req.startswith("G_X_1_"):
            bay = int(req.rsplit("_", 1)[1])
            girders[act] = 15.0 * bay
    ff = op("frame_forces", cases=["MOVE1"], frames=list(girders))["frame_forces"]

    sap_m: dict[float, float] = {}
    sap_v: dict[float, float] = {}
    xs_in_bucket: dict[float, list[float]] = {}
    for row in ff:
        x = girders[row["frame"]] + row["station"]
        b = round(x, 1)
        sap_m[b] = max(sap_m.get(b, 0.0), row["M3"])
        sap_v[b] = max(sap_v.get(b, 0.0), abs(row["V2"]))
        xs_in_bucket.setdefault(b, []).append(x)
    return sap_m, sap_v, xs_in_bucket


def physics_check(tag: str, report: dict, axles: list[float],
                  spacings: list[float]) -> None:
    """MOVE1 envelope vs exact influence-line reference: reactions, M3, |V2|."""
    L = GIRDER_L
    jr = op("joint_reactions", cases=["MOVE1"])
    rmax: dict[str, float] = {}
    for r in jr.get("reactions", []):
        rmax[r["joint"]] = max(rmax.get(r["joint"], 0.0), r["F3"])
    r_ref = reaction_reference(L, axles, spacings)
    ok = (len(rmax) == 2
          and all(abs(v - r_ref) < 0.05 for v in rmax.values()))
    check(f"{tag} support reaction envelopes == {r_ref:.3f} kip", ok,
          f"max F3 per support = { {k: round(v, 4) for k, v in rmax.items()} }"
          if rmax else f"raw response: {jr}")

    sap_m, sap_v, xs_in_bucket = girder_envelopes(report)
    all_xs = sorted({x for xs in xs_in_bucket.values() for x in xs})
    ref_m_x, ref_v_x = envelope_reference(L, axles, spacings, all_xs)
    ref_m = {b: max(ref_m_x[x] for x in xs) for b, xs in xs_in_bucket.items()}
    ref_v = {b: max(ref_v_x[x] for x in xs) for b, xs in xs_in_bucket.items()}

    stations = sorted(sap_m)
    ref_m_max = max(ref_m.values())
    ref_v_max = max(ref_v.values())
    worst_m = max(abs(sap_m[x] - ref_m[x]) for x in stations)
    worst_v = max(abs(sap_v[x] - ref_v[x]) for x in stations)
    check(f"{tag} M3 envelope vs exact reference",
          worst_m <= 0.015 * ref_m_max,
          f"worst |dM| = {worst_m:.3f} kip-ft vs ref max {ref_m_max:.1f} "
          f"({100 * worst_m / ref_m_max:.2f}%) over {len(stations)} stations")
    check(f"{tag} V2 envelope vs exact reference",
          worst_v <= 0.015 * ref_v_max,
          f"worst |dV| = {worst_v:.3f} kip vs ref max {ref_v_max:.1f} "
          f"({100 * worst_v / ref_v_max:.2f}%)")
    mid = min(stations, key=lambda x: abs(x - L / 2))
    print(f"    midspan M3: SAP {sap_m[mid]:.2f} vs ref {ref_m[mid]:.2f} kip-ft; "
          f"max |V2|: SAP {max(sap_v.values()):.2f} vs ref {ref_v_max:.2f} kip")


# ---------------------------------------------------------------------------
# Phase 1 — custom stepped axle train (round-1 regression)
# ---------------------------------------------------------------------------

def phase1() -> None:
    print("\n=== PHASE 1: custom axle train, single 60 ft girder, physics ===")
    axles = [30.0, 40.0, 30.0]
    spacings = [12.0, 12.0]
    model = girder_model("MovingLoadCustomCheck",
                         {"truck_axle_loads": axles,
                          "truck_axle_spacings": spacings})
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
    check_no_library_vehicles(m, "P1")
    assert_stepped_vehicle("P1", m, "CUSTOM1", axles, spacings)

    physics_check("P1", report, axles, spacings)

    ret = m.File.OpenFile(save)
    ok, detail = mlclass_exact(m, ["CUSTOM1"])
    n_load_rows = len(general_vehicle_load_rows(m, "CUSTOM1"))
    check("P1 reopen: tables intact",
          ret == 0 and ok and n_load_rows == 3,
          f"open ret={ret}, class {detail}, vehicle load rows={n_load_rows}")
    check_no_library_vehicles(m, "P1 reopen:")


# ---------------------------------------------------------------------------
# Phase 2 — HS20 standard truck as a general vehicle (the gap-closer)
# ---------------------------------------------------------------------------

HS20_AXLES = [8.0, 32.0, 32.0]
HS20_SPACINGS = [14.0, 14.0]


def phase2() -> None:
    print("\n=== PHASE 2: HS20 standard truck, single 60 ft girder, physics ===")
    model = girder_model("MovingLoadHS20Girder", {"truck_type": "HS20"})
    save = OUT_DIR + r"\moving_load_HS20_girder.sdb"
    resp = api("/api/sap2000/build-from-json", model, save_path=save, run_analysis=True)
    report = resp["report"]
    check("P2 build errors", not report["errors"], f"errors={report['errors']}")
    ml_lines = [l for l in report["loads"] if "MOVE1" in l]
    check("P2 HS20 general vehicle in report",
          any("HS20 (AASHTO general vehicle)" in l for l in ml_lines),
          f"loads={ml_lines}")
    check("P2 VERIFY source citation in report",
          any("VERIFY axle data" in l and "AASHTO LRFD" in l
              for l in report["loads"]),
          "AASHTO source + VERIFY flag surfaced")

    m = com_model()
    ok, detail = mlclass_exact(m, ["HS20-44"])
    check("P2 class table exact", ok, detail)
    check_no_library_vehicles(m, "P2")
    assert_stepped_vehicle("P2", m, "HS20-44", HS20_AXLES, HS20_SPACINGS)

    # The proof that a standard truck now analyzes as DISCRETE AXLES: the
    # asymmetric 8/32/32 envelope must match exact influence-line statics.
    physics_check("P2", report, HS20_AXLES, HS20_SPACINGS)

    ret = m.File.OpenFile(save)
    ok, detail = mlclass_exact(m, ["HS20-44"])
    n_load_rows = len(general_vehicle_load_rows(m, "HS20-44"))
    check("P2 reopen: tables intact",
          ret == 0 and ok and n_load_rows == 3,
          f"open ret={ret}, class {detail}, vehicle load rows={n_load_rows}")
    check_no_library_vehicles(m, "P2 reopen:")


# ---------------------------------------------------------------------------
# Phase 3 — HL-93: truck + tandem in one class, 0.64 klf lane load
# ---------------------------------------------------------------------------

def phase3() -> None:
    print("\n=== PHASE 3: HL-93, single 60 ft girder, shape + midspan ===")
    L = GIRDER_L
    model = girder_model("MovingLoadHL93Girder", {"truck_type": "HL-93"})
    save = OUT_DIR + r"\moving_load_HL93_girder.sdb"
    resp = api("/api/sap2000/build-from-json", model, save_path=save, run_analysis=True)
    report = resp["report"]
    check("P3 build errors", not report["errors"], f"errors={report['errors']}")

    m = com_model()
    ok, detail = mlclass_exact(m, ["HL93TRUCK", "HL93TANDEM"])
    check("P3 class envelopes truck + tandem", ok, detail)
    check_no_library_vehicles(m, "P3")
    assert_stepped_vehicle("P3", m, "HL93TRUCK", HS20_AXLES, HS20_SPACINGS, unif=0.64)
    assert_stepped_vehicle("P3", m, "HL93TANDEM", [25.0, 25.0], [4.0], unif=0.64)

    # AASHTO code intent at midspan: max(design truck, design tandem) placed
    # for midspan moment + 0.64 klf lane load over the full span (adverse
    # everywhere on a simple span).
    ref_truck, _ = envelope_reference(L, HS20_AXLES, HS20_SPACINGS, [L / 2])
    ref_tandem, _ = envelope_reference(L, [25.0, 25.0], [4.0], [L / 2])
    lane_m = 0.64 * L * L / 8.0
    expected = max(ref_truck[L / 2], ref_tandem[L / 2]) + lane_m

    sap_m, _sap_v, _b = girder_envelopes(report)
    mid = min(sap_m, key=lambda x: abs(x - L / 2))
    got = sap_m[mid]
    check("P3 midspan M3 == truck+lane code intent",
          abs(got - expected) <= 0.025 * expected,
          f"SAP {got:.1f} vs expected {expected:.1f} kip-ft "
          f"(truck {ref_truck[L / 2]:.1f} / tandem {ref_tandem[L / 2]:.1f} "
          f"+ lane {lane_m:.1f})")


# ---------------------------------------------------------------------------
# Phase 4 — HS20 on a 3-girder deck + slab (the round-1 blind spot)
# ---------------------------------------------------------------------------

def phase4() -> None:
    print("\n=== PHASE 4: HS20, 3-girder deck + slab (kip_ft) ===")
    model = {
        "project": {"name": "MovingLoadHS20Deck", "unit_system": "kip_ft",
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
                  "moving_load_enabled": True, "truck_type": "HS20"},
    }
    save = OUT_DIR + r"\moving_load_HS20_deck.sdb"
    resp = api("/api/sap2000/build-from-json", model, save_path=save, run_analysis=True)
    report = resp["report"]
    check("P4 build errors", not report["errors"], f"errors={report['errors']}")

    m = com_model()
    ok, detail = mlclass_exact(m, ["HS20-44"])
    check("P4 class table exact", ok, detail)
    check_no_library_vehicles(m, "P4")
    assert_stepped_vehicle("P4", m, "HS20-44", HS20_AXLES, HS20_SPACINGS)

    fmap = frame_map(report)
    mid_girders = [act for req, act in fmap.items() if req.startswith("G_X_1_")]
    ff = op("frame_forces", cases=["MOVE1"], frames=mid_girders)["frame_forces"]
    m3_max = max(r["M3"] for r in ff)
    check("P4 MOVE1 envelope nonzero", m3_max > 10.0,
          f"center girder max M3 = {m3_max:.1f} kip-ft")

    ret = m.File.OpenFile(save)
    ok, detail = mlclass_exact(m, ["HS20-44"])
    check("P4 reopen: tables intact", ret == 0 and ok,
          f"open ret={ret}, class {detail}")
    check_no_library_vehicles(m, "P4 reopen:")


# ---------------------------------------------------------------------------
# Phase 5 — Caltrans permit truck guard
# ---------------------------------------------------------------------------

def phase5() -> None:
    print("\n=== PHASE 5: Caltrans P13 guard (must error, never library) ===")
    model = girder_model("MovingLoadP13Guard", {"truck_type": "P13"})
    resp = api("/api/sap2000/build-from-json", model)
    report = resp["report"]
    errs = [e for e in report["errors"] if "Caltrans" in e]
    check("P5 P13 returns actionable error",
          len(errs) == 1 and "truck_axle_loads" in errs[0],
          errs[0][:140] if errs else f"errors={report['errors']}")
    check("P5 no MOVE1 case created",
          not any("MOVE1" in l for l in report["loads"]),
          "moving-load definition correctly aborted")
    m = com_model()
    check_no_library_vehicles(m, "P5")


def main() -> int:
    api("/api/sap2000/connect", {"visible": True})
    phase1()
    phase2()
    phase3()
    phase4()
    phase5()

    print("\n" + "=" * 60)
    n_pass = sum(1 for _, ok, _ in CHECKS if ok)
    print(f"RESULT: {n_pass}/{len(CHECKS)} checks passed")
    for name, ok, detail in CHECKS:
        if not ok:
            print(f"  FAILED: {name}: {detail}")
    return 0 if n_pass == len(CHECKS) else 1


if __name__ == "__main__":
    raise SystemExit(main())
