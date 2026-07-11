"""
Experiment 3: why didn't LoadPatterns.Add(name, 9) create a pattern?

  E1. Add patterns with several type codes; read back LoadPatterns.GetNameList
      AND the "Load Pattern Definitions" table (does the table hide some
      types, or was the pattern really not created?).
  E2. If Add(9) really fails silently, import the pattern via the
      "Load Pattern Definitions" table with candidate DesignType strings
      ("Vehicle Live" per the GUI dropdown).

Run:  C:\\Python314\\python.exe scripts/experiment_multistep_moving3.py
"""
from __future__ import annotations

import sys
import time

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402


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


def pattern_names(m):
    r = m.LoadPatterns.GetNameList(0, [])
    return [str(n) for n in (r[1] or ())] if r[-1] == 0 else []


def show_patterns(m, label):
    fields, rows = table_rows(m, "Load Pattern Definitions")
    print(f"[{label}] GetNameList: {pattern_names(m)}")
    print(f"[{label}] table rows:")
    for row in rows:
        print("   " + " | ".join(row))


def edit(m, key, fields, rows):
    flat = [str(c) for row in rows for c in row]
    r = m.DatabaseTables.SetTableForEditingArray(key, 1, fields, len(rows), flat)
    return ret_of(r)


def apply(m, label):
    r = m.DatabaseTables.ApplyEditedTables(True, 0, 0, 0, 0, "")
    ok = int(r[0]) == 0 and int(r[1]) == 0 and r[-1] == 0
    print(f"apply [{label}]: fatal={r[0]} errs={r[1]} warn={r[2]} ret={r[-1]}")
    if not ok and r[4]:
        log = str(r[4])
        # print only the error lines
        for line in log.splitlines():
            if "error" in line.lower() or "Error" in line:
                print("   " + line.strip())
    return ok


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
    show_patterns(m, "initial")

    # E1: COM Add with several codes
    for name, code in [("PAT_LIVE", 3), ("PAT_MOVE", 9), ("PAT_MOVEDEF", 57),
                       ("PAT_TRAIN", 58), ("PAT_FATIGUE", 55)]:
        r = m.LoadPatterns.Add(name, code, 0.0, False)
        print(f"Add({name!r}, {code}): ret={ret_of(r)}")
    show_patterns(m, "after COM adds")

    # E2: table import with DesignType candidates
    for i, dt in enumerate(["Vehicle Live", "VEHICLE LIVE", "VehicleLive",
                            "Vehicle Live Load", "Moving", "Move"]):
        pname = f"TPAT{i}"
        code = edit(m, "Load Pattern Definitions",
                    ["LoadPat", "DesignType", "SelfWtMult"],
                    [[pname, dt, "0"]])
        if code != 0:
            print(f"queue {pname} DesignType={dt!r}: ret={code}")
            continue
        ok = apply(m, f"{pname} DesignType={dt!r}")
        names = pattern_names(m)
        print(f"   -> created: {pname in names}")
    show_patterns(m, "after table imports")
    print("EXPERIMENT 3 DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
