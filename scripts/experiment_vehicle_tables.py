"""
Scratch experiment on a blank SAP2000 model to pin down DatabaseTables
semantics before fixing the P005 moving-load builder:

  E1. Can a GENERAL vehicle (custom stepped axle train) be created via
      "Vehicles 2 - General Vehicles 1 - General" + "Vehicles 3 - General
      Vehicles 2 - Loads" using the display field keys?
  E2. Does queueing + applying the "Vehicles 4 - Vehicle Classes" table TWICE
      produce duplicate class rows (the corruption seen in moving_load_P13.sdb)?
  E3. Does ApplyEditedTables with an empty queue no-op cleanly?

Run:  C:\\Python314\\python.exe scripts/experiment_vehicle_tables.py
"""
from __future__ import annotations

import sys

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402


def table_rows(m, key: str) -> tuple[list[str], list[list[str]]]:
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


def show_table(m, key: str) -> None:
    fields, rows = table_rows(m, key)
    print(f"--- {key} ---")
    if not fields:
        print("  (no data / read failed)")
        return
    print("  " + " | ".join(fields))
    for row in rows:
        print("  " + " | ".join(row))


def edit(m, key: str, fields: list[str], rows: list[list[str]]) -> int:
    flat = [str(c) for row in rows for c in row]
    r = m.DatabaseTables.SetTableForEditingArray(key, 1, fields, len(rows), flat)
    code = r if isinstance(r, int) else r[-1]
    print(f"queue {key}: ret={code}")
    return code


def apply(m, label: str) -> None:
    r = m.DatabaseTables.ApplyEditedTables(True, 0, 0, 0, 0, "")
    print(f"apply [{label}]: fatal={r[0]} errs={r[1]} warn={r[2]} info={r[3]} ret={r[-1]}")
    if r[4]:
        log = str(r[4])
        print("  import log: " + (log[:600] + "..." if len(log) > 600 else log))


def main() -> int:
    conn = SAP2000Connection()
    conn.connect(visible=True)
    m = conn.model
    m.InitializeNewModel(4)  # kip_ft
    m.File.NewBlank()
    print("blank kip_ft model initialized")

    # E1: custom general vehicle, stepped axle train: 10 kip, then 20 kip @ 14 ft,
    # then 20 kip @ 14 ft  (HS20-like shape but custom numbers)
    gen_fields = ["VehName", "NumInter", "StayInLane"]
    gen_rows = [["CUSTOM1", "3", "No"]]
    load_fields = ["VehName", "LoadType", "InterUnif", "InterAxle", "InterMinD", "InterMaxD"]
    load_rows = [
        ["CUSTOM1", "Leading Load", "0", "10", "", ""],
        ["CUSTOM1", "Fixed Length", "0", "20", "14", ""],
        ["CUSTOM1", "Fixed Length", "0", "20", "14", ""],
    ]
    e1 = edit(m, "Vehicles 2 - General Vehicles 1 - General", gen_fields, gen_rows)
    e2 = edit(m, "Vehicles 3 - General Vehicles 2 - Loads", load_fields, load_rows)
    if e1 == 0 and e2 == 0:
        apply(m, "E1 general vehicle")
    show_table(m, "Vehicles 2 - General Vehicles 1 - General")
    show_table(m, "Vehicles 3 - General Vehicles 2 - Loads")

    # E2: class table queued+applied twice -> duplicates?
    class_fields = ["VehClass", "VehName", "ScaleFactor"]
    class_rows = [["MLCLASS", "CUSTOM1", "1"]]
    edit(m, "Vehicles 4 - Vehicle Classes", class_fields, class_rows)
    apply(m, "E2 class apply #1")
    show_table(m, "Vehicles 4 - Vehicle Classes")
    edit(m, "Vehicles 4 - Vehicle Classes", class_fields, class_rows)
    apply(m, "E2 class apply #2 (same rows again)")
    show_table(m, "Vehicles 4 - Vehicle Classes")

    # E3: apply with empty queue
    apply(m, "E3 empty queue")

    print("EXPERIMENT DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
