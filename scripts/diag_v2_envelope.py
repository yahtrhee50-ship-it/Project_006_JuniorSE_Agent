"""
Diagnostic for the P1 V2 envelope mismatch in verify_moving_load_live.py.
Rebuilds the single-girder custom-train model, then prints SAP2000 vs exact
reference |V| at every station, plus the joint-reaction envelope at supports.

Run:  C:\\Python314\\python.exe scripts/diag_v2_envelope.py
"""
from __future__ import annotations

import sys

sys.path.insert(0, r"D:\AI_TEST\Agent_Developer\Project_006_JuniorSE_Agent\scripts")
from verify_moving_load_live import (  # noqa: E402
    api, op, frame_map, envelope_reference, OUT_DIR,
)

L = 60.0
AXLES = [30.0, 40.0, 30.0]
SPACINGS = [12.0, 12.0]


def main() -> int:
    api("/api/sap2000/connect", {"visible": True})
    model = {
        "project": {"name": "MovingLoadCustomCheck", "unit_system": "kip_ft",
                    "structure_type": "bridge_deck"},
        "grid": {"x_spacings": [15, 15, 15, 15], "y_spacings": [6, 6]},
        "girders": {
            "direction": "X",
            "section": {"name": "W24X94", "section_type": "W24X94", "material": "A992"},
            "row_indices": [1],
        },
        "piles": [
            {"x": 0, "y": 6, "restraint": [True, True, True, True, False, False]},
            {"x": 60, "y": 6, "restraint": [False, True, True, False, False, False]},
        ],
        "loads": {"moving_load_enabled": True, "truck_axle_loads": AXLES,
                  "truck_axle_spacings": SPACINGS, "lane_width": 12.0},
    }
    resp = api("/api/sap2000/build-from-json", model,
               save_path=OUT_DIR + r"\moving_load_custom_check.sdb",
               run_analysis=True)
    report = resp["report"]
    print(f"build errors: {report['errors']}")

    girders = {}
    for req, act in frame_map(report).items():
        if req.startswith("G_X_1_"):
            girders[act] = 15.0 * int(req.rsplit("_", 1)[1])

    ff = op("frame_forces", cases=["MOVE1"], frames=list(girders))["frame_forces"]
    vmax: dict[float, float] = {}
    vmin: dict[float, float] = {}
    mmax: dict[float, float] = {}
    for row in ff:
        x = round(girders[row["frame"]] + row["station"], 4)
        if row["step_type"] == "Max":
            vmax[x] = max(vmax.get(x, -1e9), row["V2"])
            mmax[x] = max(mmax.get(x, -1e9), row["M3"])
        else:
            vmin[x] = min(vmin.get(x, 1e9), row["V2"])

    stations = sorted(vmax)
    ref_m, ref_v = envelope_reference(L, AXLES, SPACINGS, stations)

    print(f"\n{'x':>7} {'V2max':>9} {'V2min':>9} {'sap|V|':>9} {'ref|V|':>9} "
          f"{'dV':>8}   {'M3max':>9} {'refM':>9} {'dM':>8}")
    for x in stations:
        sv = max(abs(vmax[x]), abs(vmin.get(x, 0.0)))
        dv = sv - ref_v[x]
        dm = mmax[x] - ref_m[x]
        flag = " <-- " if abs(dv) > 0.015 * 80 else ""
        print(f"{x:7.2f} {vmax[x]:9.3f} {vmin.get(x, 0.0):9.3f} {sv:9.3f} "
              f"{ref_v[x]:9.3f} {dv:8.3f}   {mmax[x]:9.2f} {ref_m[x]:9.2f} "
              f"{dm:8.2f}{flag}")

    jr = op("joint_reactions", cases=["MOVE1"])["reactions"]
    print("\njoint reaction envelope (MOVE1):")
    for r in jr:
        print(f"  joint {r['joint']} {r['step_type']}: F3={r['F3']:.3f} kip")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
