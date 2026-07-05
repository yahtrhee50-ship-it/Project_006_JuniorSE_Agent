"""
Round 2 of DatabaseTables semantics experiments (see experiment_vehicle_tables.py).

  E4. REPAIR: current model has MLCLASS duplicated (from round 1). Rewrite the
      FULL "Vehicles 4 - Vehicle Classes" table with corrected rows -> does the
      duplicate collapse to one row?
  E5. CREATION RECIPE: on a fresh model, queue class rows ONCE + apply; if
      MLCLASS is missing, apply AGAIN WITHOUT re-queueing -> does the original
      queued edit flush in, exactly once?

Run:  C:\\Python314\\python.exe scripts/experiment_vehicle_tables2.py
"""
from __future__ import annotations

import sys

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402

CLASS_KEY = "Vehicles 4 - Vehicle Classes"
CLASS_FIELDS = ["VehClass", "VehName", "ScaleFactor"]


def table_rows(m, key: str) -> list[list[str]]:
    r = m.DatabaseTables.GetTableForDisplayArray(key, [], "", 0, [], 0, [])
    if r[-1] != 0:
        return []
    fields = [str(f) for f in (r[2] or ())]
    n = int(r[3] or 0)
    data = list(r[4] or ())
    ncol = len(fields)
    return [["" if c is None else str(c) for c in data[i * ncol:(i + 1) * ncol]]
            for i in range(n)]


def show_classes(m, label: str) -> None:
    rows = table_rows(m, CLASS_KEY)
    print(f"  classes [{label}]: {rows}")


def edit(m, key: str, fields: list[str], rows: list[list[str]]) -> int:
    flat = [str(c) for row in rows for c in row]
    r = m.DatabaseTables.SetTableForEditingArray(key, 1, fields, len(rows), flat)
    return r if isinstance(r, int) else r[-1]


def apply(m, label: str) -> tuple[int, int, int]:
    r = m.DatabaseTables.ApplyEditedTables(True, 0, 0, 0, 0, "")
    print(f"  apply [{label}]: fatal={r[0]} errs={r[1]} warn={r[2]} info={r[3]} ret={r[-1]}")
    return int(r[0]), int(r[1]), int(r[-1])


def main() -> int:
    conn = SAP2000Connection()
    conn.connect(visible=True)
    m = conn.model

    print("E4: repair duplicated MLCLASS by rewriting the full class table")
    show_classes(m, "before repair")
    full = [["MLCLASS", "CUSTOM1", "1"], ["CUSTOM1", "CUSTOM1", "1"]]
    edit(m, CLASS_KEY, CLASS_FIELDS, full)
    apply(m, "E4 rewrite full table")
    show_classes(m, "after repair")

    print("\nE5: fresh model — queue once, apply, flush-apply without re-queue")
    m.InitializeNewModel(4)
    m.File.NewBlank()
    gen_fields = ["VehName", "NumInter", "StayInLane"]
    load_fields = ["VehName", "LoadType", "InterUnif", "InterAxle", "InterMinD", "InterMaxD"]
    edit(m, "Vehicles 2 - General Vehicles 1 - General", gen_fields, [["CUSTOM1", "3", "No"]])
    edit(m, "Vehicles 3 - General Vehicles 2 - Loads", load_fields, [
        ["CUSTOM1", "Leading Load", "0", "10", "", ""],
        ["CUSTOM1", "Fixed Length", "0", "20", "14", ""],
        ["CUSTOM1", "Fixed Length", "0", "20", "14", ""],
    ])
    apply(m, "E5 vehicle")
    edit(m, CLASS_KEY, CLASS_FIELDS, [["MLCLASS", "CUSTOM1", "1"]])
    apply(m, "E5 class apply #1")
    show_classes(m, "after class apply #1")
    apply(m, "E5 flush apply #2 (nothing re-queued)")
    show_classes(m, "after flush apply #2")

    print("EXPERIMENT 2 DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
