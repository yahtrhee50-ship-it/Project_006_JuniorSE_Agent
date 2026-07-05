"""
Dump the axle-load definition of the P13 vehicle in the currently open model
(moving_load_P13.sdb), plus the remaining moving-load tables the first probe
did not cover. Confirms whether the vehicle is a real stepped axle train.

Run:  C:\\Python314\\python.exe scripts/probe_vehicle_loads.py
"""
from __future__ import annotations

import sys

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402


def dump_table(model, key: str) -> None:
    try:
        r = model.DatabaseTables.GetTableForDisplayArray(key, [], "", 0, [], 0, [])
    except Exception as e:  # noqa: BLE001
        print(f"\n--- {key} ---\nEXCEPTION: {e}")
        return
    fields = [str(f) for f in (r[2] or ())]
    nrec = int(r[3] or 0)
    data = list(r[4] or ())
    print(f"\n--- {key} ---  ret={r[-1]}  records={nrec}")
    if not fields:
        print("  (no fields returned)")
        return
    ncol = len(fields)
    print("  " + " | ".join(fields))
    for i in range(nrec):
        row = data[i * ncol:(i + 1) * ncol]
        print("  " + " | ".join("" if c is None else str(c) for c in row))


def main() -> int:
    conn = SAP2000Connection()
    conn.connect(visible=True)
    m = conn.model
    print(f"Active model: {m.GetModelFilename(True)}")

    for key in (
        "Vehicles 2 - General Vehicles 1 - General",
        "Vehicles 3 - General Vehicles 2 - Loads",
        "Case - Moving Load 2 - Lanes Loaded",
        "Case - Moving Load 3 - MultiLane Factors",
        "Lane Centerline Points",
    ):
        dump_table(m, key)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
