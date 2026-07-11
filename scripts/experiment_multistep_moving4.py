"""
Experiment 4: does the Multi-Step Moving Load table data actually attach?

Learned in exp 3: LoadPatterns.Add(name, 9) DOES create the pattern (the
"Load Pattern Definitions" DISPLAY table hides Move-type patterns). So read
everything back with GetTableForEditingArray instead, which reads the raw
definition data:

  E1. After Add(MSV, 9): what do "Multi-Step Moving Load 1 - General" /
      "2 - Vehicle Data" look like via the EDITING read (default row? exact
      enum strings for SpeedFrom / Direction / FLLocation)?
  E2. Import our rows, read back via editing read — did they stick?
  E3. Case + run: how many steps does MSCASE produce, and does midspan M3
      track the axle?

Run:  C:\\Python314\\python.exe scripts/experiment_multistep_moving4.py
"""
from __future__ import annotations

import sys
import time

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402

SPAN, P_AXLE, SPEED, LOAD_DUR, LOAD_DISC = 60.0, 30.0, 5.0, 12.0, 1.0
SAVE_PATH = (r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
             r"\outputs\experiment_multistep.sdb")


def ret_of(r):
    return r if isinstance(r, int) else r[-1]


def edit_read(m, key):
    """Read a table with GetTableForEditingArray (raw definition data)."""
    try:
        r = m.DatabaseTables.GetTableForEditingArray(key, "", 0, [], 0, [])
    except Exception as exc:
        return None, f"EXC {exc}"
    if r[-1] != 0:
        return None, f"ret={r[-1]}"
    fields = [str(f) for f in (r[1] or ())]
    n = int(r[2] or 0)
    data = list(r[3] or ())
    ncol = len(fields)
    rows = [["" if c is None else str(c) for c in data[i * ncol:(i + 1) * ncol]]
            for i in range(n)]
    return (fields, rows), None


def show_edit(m, key):
    out, err = edit_read(m, key)
    print(f"--- EDIT-READ {key} ---")
    if err:
        print(f"  ({err})")
        return
    fields, rows = out
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
    ok = int(r[0]) == 0 and int(r[1]) == 0 and r[-1] == 0
    print(f"apply [{label}]: fatal={r[0]} errs={r[1]} warn={r[2]} ret={r[-1]}")
    if r[4] and (not ok or int(r[2])):
        for line in str(r[4]).splitlines():
            ll = line.strip()
            if ll and ("error" in ll.lower() or "warn" in ll.lower()
                       or "Table:" in ll):
                print("   " + ll)
    return ok


def exact_midspan_m(x):
    if x < 0 or x > SPAN:
        return 0.0
    return P_AXLE * min(x, SPAN - x) * 0.5 if 0 <= x <= SPAN else 0.0


def main() -> int:
    conn = SAP2000Connection()
    conn.connect(visible=True)
    m = conn.model
    for _ in range(30):
        try:
            m.InitializeNewModel(4)
            m.File.NewBlank()
            break
        except Exception:
            time.sleep(2)
    print("blank kip_ft model")

    assert ret_of(m.PropMaterial.SetMaterial("STL", 1)) == 0
    assert ret_of(m.PropMaterial.SetMPIsotropic("STL", 29000.0 * 144, 0.3, 6.5e-6)) == 0
    assert ret_of(m.PropFrame.SetRectangle("GIRD", "STL", 2.0, 1.0)) == 0
    frames = []
    for i in range(6):
        r = m.FrameObj.AddByCoord(10.0 * i, 0, 0, 10.0 * (i + 1), 0, 0,
                                  "", "GIRD", f"G{i+1}", "Global")
        assert r[-1] == 0
        frames.append(str(r[0]))
    r = m.FrameObj.GetPoints(frames[0], "", "")
    j_left = str(r[0])
    r = m.FrameObj.GetPoints(frames[-1], "", "")
    j_right = str(r[1])
    assert ret_of(m.PointObj.SetRestraint(j_left, [True, True, True, True, False, False])) == 0
    assert ret_of(m.PointObj.SetRestraint(j_right, [False, True, True, False, False, False])) == 0

    # vehicle + lane
    ok = (edit(m, "Vehicles 2 - General Vehicles 1 - General",
               ["VehName", "NumInter", "StayInLane"], [["TESTVEH", "0", "Yes"]]) == 0
          and edit(m, "Vehicles 3 - General Vehicles 2 - Loads",
                   ["VehName", "LoadType", "InterUnif", "InterAxle",
                    "InterMinD", "InterMaxD"],
                   [["TESTVEH", "Leading Load", "0", str(P_AXLE), "", ""]]) == 0
          and apply(m, "vehicle"))
    assert ok
    lane_fields = ["Lane", "LaneFrom", "LaneType", "Frame", "Width", "Offset", "DiscAlong"]
    lane_rows = [["LANE1", "Frame" if i == 0 else "", "Vehicle" if i == 0 else "",
                  f, "10", "0", "1.1111" if i == 0 else ""]
                 for i, f in enumerate(frames)]
    assert edit(m, "Lane Definition Data", lane_fields, lane_rows) == 0
    assert apply(m, "lane")

    # E1: pattern + default multi-step rows
    assert ret_of(m.LoadPatterns.Add("MSV", 9, 0.0, False)) == 0
    r = m.LoadPatterns.GetLoadType("MSV", 0)
    print(f"GetLoadType(MSV): type={r[0] if not isinstance(r, int) else r} "
          f"ret={ret_of(r)}")
    print(f"patterns: {[str(n) for n in m.LoadPatterns.GetNameList(0, [])[1]]}")
    show_edit(m, "Multi-Step Moving Load 1 - General")
    show_edit(m, "Multi-Step Moving Load 2 - Vehicle Data")

    # E2: import our data, read back via editing read
    assert edit(m, "Multi-Step Moving Load 1 - General",
                ["LoadPat", "LoadDur", "LoadDisc", "SpeedFrom"],
                [["MSV", str(LOAD_DUR), str(LOAD_DISC), "Vehicle"]]) == 0
    assert edit(m, "Multi-Step Moving Load 2 - Vehicle Data",
                ["LoadPat", "Vehicle", "Lane", "Station", "StartTime",
                 "Direction", "Speed", "FLLocation", "VertSF",
                 "LongitSF", "TransvSF", "CentrifSF"],
                [["MSV", "TESTVEH", "LANE1", "0", "0",
                  "Forward", str(SPEED), "", "1", "", "", ""]]) == 0
    apply(m, "multi-step moving load data")
    show_edit(m, "Multi-Step Moving Load 1 - General")
    show_edit(m, "Multi-Step Moving Load 2 - Vehicle Data")

    # E3: case + run + step count
    ms = m.LoadCases.StaticLinearMultistep
    assert ret_of(ms.SetCase("MSCASE")) == 0
    assert ret_of(ms.SetLoads("MSCASE", 1, ["Load"], ["MSV"], [1.0])) == 0
    assert ret_of(m.File.Save(SAVE_PATH)) == 0
    assert ret_of(m.Analyze.RunAnalysis()) == 0
    setup = m.Results.Setup
    assert setup.DeselectAllCasesAndCombosForOutput() == 0
    assert setup.SetCaseSelectedForOutput("MSCASE") == 0
    assert ret_of(setup.SetOptionMultiStepStatic(2)) == 0

    r = m.Results.FrameForce("ALL", 2, 0, [], [], [], [], [], [], [],
                             [], [], [], [], [], [])
    assert r[-1] == 0
    n = int(r[0])
    obj = [str(v) for v in (r[1] or ())]
    sta = [float(v) for v in (r[2] or ())]
    steptype = [str(v) for v in (r[6] or ())]
    stepnum = [float(v) for v in (r[7] or ())]
    m3 = [float(v) for v in (r[13] or ())]
    steps = sorted(set(stepnum))
    print(f"\nFrameForce rows={n}; unique steps={steps}; "
          f"steptypes={sorted(set(steptype))}")

    mid = frames[3]
    rows = sorted((stepnum[i], m3[i]) for i in range(n)
                  if obj[i] == mid and abs(sta[i]) < 1e-6)
    print("\nstep | t | x_axle | M3 SAP | M3 exact | diff")
    worst = 0.0
    for step, m3v in rows:
        t = step * LOAD_DISC
        x = SPEED * t
        ex = exact_midspan_m(x)
        worst = max(worst, abs(m3v - ex))
        print(f"{step:4.0f} | {t:4.1f} | {x:5.1f} | {m3v:9.3f} | {ex:9.3f} | {m3v-ex:+.4f}")
    print(f"worst |M3-exact| = {worst:.4f} kip-ft over {len(rows)} steps")
    print("EXPERIMENT 4 DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
