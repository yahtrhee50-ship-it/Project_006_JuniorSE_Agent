"""
Probe the saved moving-load model (P005 outputs/moving_load_P13.sdb) live.

Purpose: reproduce the engineer's report of an error message when opening the
model, and confirm what the $2k text export suggests:
  - "Vehicles 1 - Standard Vehicles" table is MISSING from the saved file
    (MOVE1 -> MLCLASS -> vehicle "P13" is a dangling reference), and
  - "Vehicles 4 - Vehicle Classes" has a DUPLICATED MLCLASS/P13 row.

Attaches to the running SAP2000 instance (same one the P005 server uses),
opens the .sdb, and dumps the moving-load related database tables.

Run:  C:\\Python314\\python.exe scripts/probe_moving_load_model.py
"""
from __future__ import annotations

import sys

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
SDB = P005_ROOT + r"\outputs\moving_load_P13.sdb"

sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402


def show(label: str, obj) -> None:
    text = repr(obj)
    if len(text) > 1200:
        text = text[:1200] + " ...<truncated>"
    print(f"\n--- {label} ---\n{text}")


def dump_table(model, key: str) -> None:
    try:
        r = model.DatabaseTables.GetTableForDisplayArray(key, [], "", 0, [], 0, [])
    except Exception as e:  # noqa: BLE001
        print(f"\n--- {key} ---\nEXCEPTION: {e}")
        return
    print(f"\n--- {key} ---")
    print(f"ret={r[-1]}")
    for i, part in enumerate(r[:-1]):
        text = repr(part)
        if len(text) > 2000:
            text = text[:2000] + " ...<truncated>"
        print(f"  r[{i}] = {text}")


def main() -> int:
    conn = SAP2000Connection()
    conn.connect(visible=True)
    m = conn.model

    print(f"Opening: {SDB}")
    ret = m.File.OpenFile(SDB)
    print(f"File.OpenFile ret = {ret}")

    try:
        fname = str(m.GetModelFilename(True) or "")
    except Exception:
        fname = "?"
    print(f"Active model file: {fname}")

    # Which tables does SAP2000 report as having data?
    try:
        r = m.DatabaseTables.GetAvailableTables(0, [], [], [])
        keys = [str(k) for k in (r[1] or ())]
        vehicle_ish = [k for k in keys if any(s in k.lower() for s in ("vehicle", "lane", "moving"))]
        show("Available tables mentioning vehicle/lane/moving", vehicle_ish)
    except Exception as e:  # noqa: BLE001
        print(f"GetAvailableTables failed: {e}")

    for key in (
        "Vehicles 1 - Standard Vehicles",
        "Vehicles 2 - General Vehicles 1 - General",
        "Vehicles 4 - Vehicle Classes",
        "Lane Definition Data",
        "Case - Moving Load 1 - Lane Assignments",
    ):
        dump_table(m, key)

    # Moving-load case definition via the classic API
    try:
        r = m.LoadCases.Moving.GetLoads("MOVE1", 0, [], [], [], [])
        show("LoadCases.Moving.GetLoads('MOVE1')", r)
    except Exception as e:  # noqa: BLE001
        print(f"Moving.GetLoads failed: {e}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
