"""
Experiment 6: interactive import of the Multi-Step Moving Load tables with
the CORRECT TableVersion (builder convention of TableVersion=1 may be the
reason rows are silently dropped) and fully-populated field values.

Verification of storage = save + read the auto text backup (.$2k), which is
the only read-back that shows vehicle-live pattern data (display AND editing
table reads hide it; s2k grep is authoritative — learned exp 2-5).

Run:  C:\\Python314\\python.exe scripts/experiment_multistep_moving6.py
"""
from __future__ import annotations

import os
import sys
import time

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402

SPAN, P_AXLE, SPEED, LOAD_DUR, LOAD_DISC = 60.0, 30.0, 5.0, 12.0, 1.0
SCRATCH = (r"C:\Users\richa\AppData\Local\Temp\claude"
           r"\D--AI-TEST-Agent-Developer-Project-006-JuniorSE-Agent"
           r"\238bd2e6-ebf3-4239-8620-221bf7f2d1b1\scratchpad")
SDB = SCRATCH + r"\exp6.sdb"


def ret_of(r):
    return r if isinstance(r, int) else r[-1]


def edit(m, key, fields, rows, version):
    flat = [str(c) for row in rows for c in row]
    r = m.DatabaseTables.SetTableForEditingArray(key, version, fields,
                                                 len(rows), flat)
    code = ret_of(r)
    print(f"queue v{version} {key}: ret={code}")
    return code


def apply(m, label):
    r = m.DatabaseTables.ApplyEditedTables(True, 0, 0, 0, 0, "")
    ok = int(r[0]) == 0 and int(r[1]) == 0 and r[-1] == 0
    print(f"apply [{label}]: fatal={r[0]} errs={r[1]} warn={r[2]} ret={r[-1]}")
    if r[4] and not ok:
        print(str(r[4])[:1200])
    return ok


def table_version(m, key):
    r = m.DatabaseTables.GetAllFieldsInTable(key, 0, 0, [], [], [], [], [])
    # r = (TableVersion, NumberFields, FieldKey, FieldName, Description,
    #      UnitsString, IsImportable, ret)
    return int(r[0]), ret_of(r)


def s2k_backup_lines(pattern):
    """grep the freshly saved .$2k for a pattern."""
    path = SDB.replace(".sdb", ".$2k")
    if not os.path.exists(path):
        return None
    txt = open(path, encoding="ascii", errors="replace").read()
    return [ln for ln in txt.splitlines() if pattern.lower() in ln.lower()]


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

    # versions of the tables of interest
    for key in ("Multi-Step Moving Load 1 - General",
                "Multi-Step Moving Load 2 - Vehicle Data",
                "Vehicles 2 - General Vehicles 1 - General",
                "Lane Definition Data"):
        v, code = table_version(m, key)
        print(f"TableVersion[{key}] = {v} (ret={code})")

    # structure
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

    # vehicle + lane (known-good v1 imports)
    assert edit(m, "Vehicles 2 - General Vehicles 1 - General",
                ["VehName", "NumInter", "StayInLane"],
                [["TESTVEH", "0", "Yes"]], 1) == 0
    assert edit(m, "Vehicles 3 - General Vehicles 2 - Loads",
                ["VehName", "LoadType", "InterUnif", "InterAxle",
                 "InterMinD", "InterMaxD"],
                [["TESTVEH", "Leading Load", "0", str(P_AXLE), "", ""]], 1) == 0
    assert apply(m, "vehicle")
    lane_fields = ["Lane", "LaneFrom", "LaneType", "Frame", "Width", "Offset", "DiscAlong"]
    lane_rows = [["LANE1", "Frame" if i == 0 else "", "Vehicle" if i == 0 else "",
                  f, "10", "0", "1.1111" if i == 0 else ""]
                 for i, f in enumerate(frames)]
    assert edit(m, "Lane Definition Data", lane_fields, lane_rows, 1) == 0
    assert apply(m, "lane")

    # pattern + case
    assert ret_of(m.LoadPatterns.Add("MSV", 9, 0.0, False)) == 0
    ms = m.LoadCases.StaticLinearMultistep
    assert ret_of(ms.SetCase("MSCASE")) == 0
    assert ret_of(ms.SetLoads("MSCASE", 1, ["Load"], ["MSV"], [1.0])) == 0

    # multi-step tables with native version + all fields populated
    v1, _ = table_version(m, "Multi-Step Moving Load 1 - General")
    v2, _ = table_version(m, "Multi-Step Moving Load 2 - Vehicle Data")
    assert edit(m, "Multi-Step Moving Load 1 - General",
                ["LoadPat", "LoadDur", "LoadDisc", "SpeedFrom"],
                [["MSV", str(LOAD_DUR), str(LOAD_DISC), "Vehicle"]], v1) == 0
    assert edit(m, "Multi-Step Moving Load 2 - Vehicle Data",
                ["LoadPat", "Vehicle", "Lane", "Station", "StartTime",
                 "Direction", "Speed", "FLLocation", "VertSF",
                 "LongitSF", "TransvSF", "CentrifSF"],
                [["MSV", "TESTVEH", "LANE1", "0", "0",
                  "Forward", str(SPEED), "0", "1", "1", "1", "1"]], v2) == 0
    apply(m, "multi-step tables (native version)")

    # authoritative check: text backup
    assert ret_of(m.File.Save(SDB)) == 0
    hits = s2k_backup_lines("MULTI-STEP MOVING")
    print(f"backup grep MULTI-STEP MOVING: {hits}")
    if hits:
        hits2 = s2k_backup_lines("Vehicle=TESTVEH")
        print(f"backup grep vehicle row: {hits2}")
        print("STORED — SUCCESS PATH CONFIRMED")
        # run + step check
        assert m.Analyze.RunAnalysis() == 0
        setup = m.Results.Setup
        assert setup.DeselectAllCasesAndCombosForOutput() == 0
        assert setup.SetCaseSelectedForOutput("MSCASE") == 0
        assert setup.SetOptionMultiStepStatic(2) == 0
        r = m.Results.FrameForce("ALL", 2, 0, [], [], [], [], [], [], [],
                                 [], [], [], [], [], [])
        assert r[-1] == 0
        n = int(r[0])
        obj = [str(v) for v in (r[1] or ())]
        sta = [float(v) for v in (r[2] or ())]
        stepnum = [float(v) for v in (r[7] or ())]
        m3 = [float(v) for v in (r[13] or ())]
        rows = sorted((stepnum[i], m3[i]) for i in range(n)
                      if obj[i] == "G4" and abs(sta[i]) < 1e-6)
        worst = 0.0
        print("step | t | x | M3 SAP | M3 exact")
        for step, m3v in rows:
            x = SPEED * step * LOAD_DISC
            ex = P_AXLE * min(x, SPAN - x) * 0.5 if 0 <= x <= SPAN else 0.0
            worst = max(worst, abs(m3v - ex))
            print(f"{step:4.0f} | {step*LOAD_DISC:4.1f} | {x:5.1f} | "
                  f"{m3v:9.3f} | {ex:9.3f}")
        print(f"steps={len(rows)} worst diff={worst:.4f} kip-ft")
    else:
        print("STILL DROPPED — interactive import cannot write these tables")
    print("EXPERIMENT 6 DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
