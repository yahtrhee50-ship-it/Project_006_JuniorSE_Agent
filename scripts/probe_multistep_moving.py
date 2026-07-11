"""
Probe 1 (READ-ONLY) for the multi-step static moving-load task:

  P1. What is currently open in SAP2000 (so experiments don't clobber it blind)?
  P2. Which DatabaseTables exist for vehicle-live / multi-step load patterns?
      (GetAllTables lists every table the program knows, importable or not —
      unlike GetAvailableTables which only lists tables with current data.)
  P3. Field lists (GetAllFieldsInTable) for the candidate tables.
  P4. Does the COM object expose LoadCases.StaticLinearMultistep (and what
      methods does the wrapper show)?

Run:  C:\\Python314\\python.exe scripts/probe_multistep_moving.py
"""
from __future__ import annotations

import sys

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402

KEYWORDS = ("vehicle", "moving", "multi", "lane", "load pattern")


def main() -> int:
    import time

    conn = SAP2000Connection()
    conn.connect(visible=True)
    m = conn.model

    # P1 — current file; also wait for the app to be fully up (a freshly
    # launched instance throws "window handle has not been created" if COM
    # calls arrive before the GUI finishes starting).
    fname = ""
    for attempt in range(30):
        try:
            fname = str(m.GetModelFilename(True) or "")
            break
        except Exception:
            time.sleep(2)
    print(f"P1 current model file: {fname!r}")
    if not fname:
        # No model open (fresh instance) — DatabaseTables needs one.
        for attempt in range(30):
            try:
                m.InitializeNewModel(4)  # kip_ft
                m.File.NewBlank()
                print("P1 initialized blank kip_ft model")
                break
            except Exception:
                time.sleep(2)

    # P2 — all tables matching keywords
    print("\nP2 GetAllTables (keyword matches):")
    r = m.DatabaseTables.GetAllTables(0, [], [], [], [])
    ret = r[-1]
    if ret != 0:
        print(f"  GetAllTables ret={ret} — FAILED")
        return 1
    n = int(r[0])
    keys = [str(k) for k in (r[1] or ())]
    names = [str(k) for k in (r[2] or ())]
    import_types = list(r[3] or ())
    is_empty = list(r[4] or ())
    hits = []
    for i in range(n):
        low = keys[i].lower()
        if any(kw in low for kw in KEYWORDS):
            hits.append(keys[i])
            print(f"  key={keys[i]!r} import_type={import_types[i]} empty={is_empty[i]}")

    # P3 — fields of the most promising tables
    print("\nP3 fields of candidate tables:")
    candidates = [k for k in hits
                  if "vehicle" in k.lower() or "moving" in k.lower()
                  or "multi" in k.lower() or "load pattern" in k.lower()]
    for key in candidates:
        try:
            fr = m.DatabaseTables.GetAllFieldsInTable(key, 0, 0, [], [], [], [], [])
            if fr[-1] != 0:
                print(f"  {key}: GetAllFieldsInTable ret={fr[-1]}")
                continue
            nf = int(fr[1])
            fkeys = [str(f) for f in (fr[2] or ())][:nf]
            print(f"  --- {key} ({nf} fields) ---")
            print("      " + ", ".join(fkeys))
        except Exception as exc:
            print(f"  {key}: EXC {exc}")

    # P4 — StaticLinearMultistep interface
    print("\nP4 LoadCases interfaces:")
    lc = m.LoadCases
    for attr in ("StaticLinearMultistep", "StaticLinearMultiStep", "Moving",
                 "StaticLinear", "StaticNonlinear"):
        try:
            obj = getattr(lc, attr)
            methods = [a for a in dir(obj) if not a.startswith("_")]
            print(f"  LoadCases.{attr}: OK -> methods: {methods}")
        except Exception as exc:
            print(f"  LoadCases.{attr}: MISSING ({type(exc).__name__}: {exc})")

    print("\nPROBE 1 DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
