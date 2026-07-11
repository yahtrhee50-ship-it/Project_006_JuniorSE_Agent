"""
Experiment 2 for the multi-step static moving-load task (blank model, safe):

Goal: prove the full COM path for stepping a vehicle along a lane at a given
speed/start time as a MULTI-STEP STATIC case (distinct from the MOVE1
influence-line envelope):

  E1. Load pattern of type Move (eLoadPatternType 9) — what DesignType string
      does "Load Pattern Definitions" read back? (GUI calls it "VEHICLE LIVE")
  E2. Import "Multi-Step Moving Load 1 - General" (LoadPat, LoadDur, LoadDisc,
      SpeedFrom) + "Multi-Step Moving Load 2 - Vehicle Data" (Vehicle, Lane,
      Station, StartTime, Direction, Speed, ...) for that pattern.
  E3. LoadCases.StaticLinearMultistep.SetCase + SetLoads(pattern).
  E4. Run; read per-step frame forces (Results.Setup.SetOptionMultiStepStatic
      step-by-step) and compare midspan M3 vs exact statics for a single
      30 kip axle at x = Speed * t on a 60 ft simply supported span.

Model: kip_ft. Girder = 6 frames of 10 ft (so the lane + output stations have
interior nodes), pin at x=0 (+R1 torsion), roller at x=60.

Run:  C:\\Python314\\python.exe scripts/experiment_multistep_moving.py
"""
from __future__ import annotations

import sys
import time

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402

SPAN = 60.0        # ft
P_AXLE = 30.0      # kip
SPEED = 5.0        # ft/s
LOAD_DUR = 12.0    # s -> vehicle travels 0..60 ft
LOAD_DISC = 1.0    # s -> 12 steps
SAVE_PATH = (r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
             r"\outputs\experiment_multistep.sdb")


def ret_of(r):
    return r if isinstance(r, int) else r[-1]


def table_rows(m, key):
    r = m.DatabaseTables.GetTableForDisplayArray(key, [], "", 0, [], 0, [])
    if r[-1] != 0:
        return [], []
    fields = [str(f) for f in (r[2] or ())]
    n = int(r[3] or 0)
    data = list(r[4] or ())
    ncol = len(fields)
    rows = [["" if c is None else str(c) for c in data[i * ncol:(i + 1) * ncol]]
            for i in range(n)]
    return fields, rows


def available_tables(m) -> set[str]:
    r = m.DatabaseTables.GetAvailableTables(0, [], [], [])
    if r[-1] != 0:
        return set()
    return {str(k) for k in (r[1] or ())}


def show_table(m, key):
    print(f"--- {key} ---")
    if key not in available_tables(m):
        print("  (table NOT in GetAvailableTables — no data)")
        return
    fields, rows = table_rows(m, key)
    if not fields:
        print("  (read failed)")
        return
    print("  " + " | ".join(fields))
    for row in rows:
        print("  " + " | ".join(row))


def edit(m, key, fields, rows):
    flat = [str(c) for row in rows for c in row]
    r = m.DatabaseTables.SetTableForEditingArray(key, 1, fields, len(rows), flat)
    code = ret_of(r)
    print(f"queue {key}: ret={code}")
    return code


def apply(m, label):
    r = m.DatabaseTables.ApplyEditedTables(True, 0, 0, 0, 0, "")
    print(f"apply [{label}]: fatal={r[0]} errs={r[1]} warn={r[2]} info={r[3]} ret={r[-1]}")
    if r[4]:
        log = str(r[4])
        print("  import log: " + (log[:1500] + "..." if len(log) > 1500 else log))
    return int(r[0]) == 0 and int(r[1]) == 0 and r[-1] == 0


def exact_midspan_m(x):
    """M3 at midspan (30 ft) for a single P_AXLE at position x on the span."""
    if x < 0 or x > SPAN:
        return 0.0
    if x <= SPAN / 2:
        return P_AXLE * x * (SPAN - SPAN / 2) / SPAN   # P*x*(L-b)/L with b=30
    return P_AXLE * (SPAN - x) * (SPAN / 2) / SPAN


def main() -> int:
    conn = SAP2000Connection()
    conn.connect(visible=True)
    m = conn.model
    for _ in range(30):
        try:
            m.InitializeNewModel(4)  # kip_ft
            m.File.NewBlank()
            break
        except Exception:
            time.sleep(2)
    print("blank kip_ft model")

    # ── structure: material, section, 6 frames of 10 ft ──
    assert ret_of(m.PropMaterial.SetMaterial("STL", 1)) == 0
    assert ret_of(m.PropMaterial.SetMPIsotropic("STL", 29000.0 * 144, 0.3, 6.5e-6)) == 0
    assert ret_of(m.PropFrame.SetRectangle("GIRD", "STL", 2.0, 1.0)) == 0
    frames = []
    for i in range(6):
        x1, x2 = 10.0 * i, 10.0 * (i + 1)
        r = m.FrameObj.AddByCoord(x1, 0, 0, x2, 0, 0, "", "GIRD", f"G{i+1}", "Global")
        assert r[-1] == 0
        frames.append(str(r[0]))
    print("frames:", frames)
    # supports: pin + torsion at x=0 joint of first frame, roller at x=60
    r = m.FrameObj.GetPoints(frames[0], "", "")
    j_left = str(r[0])
    r = m.FrameObj.GetPoints(frames[-1], "", "")
    j_right = str(r[1])
    assert ret_of(m.PointObj.SetRestraint(j_left, [True, True, True, True, False, False])) == 0
    assert ret_of(m.PointObj.SetRestraint(j_right, [False, True, True, False, False, False])) == 0

    # ── E0: general vehicle (single 30 kip axle) + lane ──
    ok = (edit(m, "Vehicles 2 - General Vehicles 1 - General",
               ["VehName", "NumInter", "StayInLane"], [["TESTVEH", "0", "Yes"]]) == 0
          and edit(m, "Vehicles 3 - General Vehicles 2 - Loads",
                   ["VehName", "LoadType", "InterUnif", "InterAxle",
                    "InterMinD", "InterMaxD"],
                   [["TESTVEH", "Leading Load", "0", str(P_AXLE), "", ""]]) == 0
          and apply(m, "vehicle"))
    if not ok:
        return 1
    lane_fields = ["Lane", "LaneFrom", "LaneType", "Frame", "Width", "Offset", "DiscAlong"]
    lane_rows = []
    for i, fname in enumerate(frames):
        lane_rows.append(["LANE1", "Frame" if i == 0 else "",
                          "Vehicle" if i == 0 else "", fname, "10",
                          "0", "1.1111" if i == 0 else ""])
    if edit(m, "Lane Definition Data", lane_fields, lane_rows) != 0 \
            or not apply(m, "lane"):
        return 1
    show_table(m, "Vehicles 2 - General Vehicles 1 - General")
    show_table(m, "Lane Definition Data")

    # ── E1: load pattern type Move(9) ──
    r = m.LoadPatterns.Add("MSV", 9, 0.0, False)
    print(f"LoadPatterns.Add('MSV', 9): ret={ret_of(r)}")
    show_table(m, "Load Pattern Definitions")

    # ── E2: Multi-Step Moving Load tables ──
    g_ok = edit(m, "Multi-Step Moving Load 1 - General",
                ["LoadPat", "LoadDur", "LoadDisc", "SpeedFrom"],
                [["MSV", str(LOAD_DUR), str(LOAD_DISC), "Vehicle"]]) == 0
    v_ok = edit(m, "Multi-Step Moving Load 2 - Vehicle Data",
                ["LoadPat", "Vehicle", "Lane", "Station", "StartTime",
                 "Direction", "Speed", "FLLocation", "VertSF",
                 "LongitSF", "TransvSF", "CentrifSF"],
                [["MSV", "TESTVEH", "LANE1", "0", "0",
                  "Forward", str(SPEED), "", "1", "", "", ""]]) == 0
    if not (g_ok and v_ok and apply(m, "multi-step moving load")):
        return 1
    show_table(m, "Multi-Step Moving Load 1 - General")
    show_table(m, "Multi-Step Moving Load 2 - Vehicle Data")

    # ── E3: multistep static case ──
    ms = m.LoadCases.StaticLinearMultistep
    print(f"SetCase('MSCASE'): ret={ret_of(ms.SetCase('MSCASE'))}")
    r = ms.SetLoads("MSCASE", 1, ["Load"], ["MSV"], [1.0])
    print(f"SetLoads('MSCASE', Load MSV x1.0): ret={ret_of(r)}")
    show_table(m, "Case - Multistep Static 1 - Load Assignments")

    # ── E4: run + per-step midspan M3 ──
    assert ret_of(m.File.Save(SAVE_PATH)) == 0
    assert ret_of(m.Analyze.RunAnalysis()) == 0
    setup = m.Results.Setup
    assert setup.DeselectAllCasesAndCombosForOutput() == 0
    assert setup.SetCaseSelectedForOutput("MSCASE") == 0
    r = setup.SetOptionMultiStepStatic(2)   # 1=envelope, 2=step-by-step
    print(f"SetOptionMultiStepStatic(2): ret={ret_of(r)}")

    r = m.Results.FrameForce("ALL", 2, 0, [], [], [], [], [], [], [],
                             [], [], [], [], [], [])
    assert r[-1] == 0, f"FrameForce ret={r[-1]}"
    n = int(r[0])
    obj = [str(v) for v in (r[1] or ())]
    sta = [float(v) for v in (r[2] or ())]
    case = [str(v) for v in (r[5] or ())]
    steptype = [str(v) for v in (r[6] or ())]
    stepnum = [float(v) for v in (r[7] or ())]
    m3 = [float(v) for v in (r[13] or ())]
    print(f"FrameForce rows: {n}")

    # midspan = station 0 of frame G4 (x=30) or station 10 of G3 — pick G4@0
    mid_frame = frames[3]
    rows = [(stepnum[i], m3[i]) for i in range(n)
            if obj[i] == mid_frame and abs(sta[i]) < 1e-6
            and case[i] == "MSCASE"]
    rows.sort()
    print("\nstep | t(s) | x_axle(ft) | M3_mid SAP | M3_mid exact | ratio")
    worst = 0.0
    for step, m3v in rows:
        t = step * LOAD_DISC
        x = SPEED * t
        exact = exact_midspan_m(x)
        ratio = m3v / exact if abs(exact) > 1e-9 else (0.0 if abs(m3v) < 1e-6 else 999)
        err = abs(m3v - exact)
        worst = max(worst, err)
        print(f"{step:4.0f} | {t:4.1f} | {x:6.1f} | {m3v:10.3f} | {exact:10.3f} | {ratio:.5f}")
    print(f"\nstep types seen: {sorted(set(steptype))}")
    print(f"worst |M3 - exact| = {worst:.4f} kip-ft")
    print("EXPERIMENT DONE — SUCCESS" if rows else "EXPERIMENT DONE — NO STEP ROWS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
