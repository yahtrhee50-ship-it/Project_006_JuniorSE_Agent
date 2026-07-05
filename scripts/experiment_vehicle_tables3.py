"""
Round 3: on the live E5 state (vehicle CUSTOM1 exists, MLCLASS missing),
rewrite the FULL class table (MLCLASS + auto class together) -> does MLCLASS
get created? This is the missing-class half of the E4 repair recipe.

Run:  C:\\Python314\\python.exe scripts/experiment_vehicle_tables3.py
"""
from __future__ import annotations

import sys

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402

CLASS_KEY = "Vehicles 4 - Vehicle Classes"
CLASS_FIELDS = ["VehClass", "VehName", "ScaleFactor"]


def rows(m) -> list[list[str]]:
    r = m.DatabaseTables.GetTableForDisplayArray(CLASS_KEY, [], "", 0, [], 0, [])
    if r[-1] != 0:
        return []
    n = int(r[3] or 0)
    ncol = len(r[2] or ())
    data = list(r[4] or ())
    return [["" if c is None else str(c) for c in data[i * ncol:(i + 1) * ncol]]
            for i in range(n)]


def main() -> int:
    conn = SAP2000Connection()
    conn.connect(visible=True)
    m = conn.model
    print(f"before: {rows(m)}")
    full = [["MLCLASS", "CUSTOM1", "1"], ["CUSTOM1", "CUSTOM1", "1"]]
    flat = [c for row in full for c in row]
    m.DatabaseTables.SetTableForEditingArray(CLASS_KEY, 1, CLASS_FIELDS, len(full), flat)
    r = m.DatabaseTables.ApplyEditedTables(True, 0, 0, 0, 0, "")
    print(f"apply: fatal={r[0]} errs={r[1]} warn={r[2]} info={r[3]} ret={r[-1]}")
    print(f"after: {rows(m)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
