"""
Experiment 8: calibrate the Multi-Step Moving Load position convention.

Exp 7 proved the pipeline (s2k splice with PROGRAM CONTROL version fix) and
showed the axle at x = 5*(k-1) - 1 ft on step k — a -1 ft offset from the
naive reading. Calibrate exactly with 3 patterns in one model, using per-step
REACTIONS to recover the load position: a = L * R_right / P.

  MSV1: Station=0,  StartTime=0, Speed=5   (baseline)
  MSV2: Station=10, StartTime=0, Speed=5   (does Station shift 1:1?)
  MSV3: Station=0,  StartTime=4, Speed=5   (does StartTime delay the run?)

Each gets its own StaticLinearMultistep case (C1, C2, C3).

Run:  C:\\Python314\\python.exe scripts/experiment_multistep_moving8.py
"""
from __future__ import annotations

import ctypes
import os
import sys
import threading
import time

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402

SPAN, P_AXLE, SPEED, LOAD_DUR, LOAD_DISC = 60.0, 30.0, 5.0, 12.0, 1.0
SCRATCH = (r"C:\Users\richa\AppData\Local\Temp\claude"
           r"\D--AI-TEST-Agent-Developer-Project-006-JuniorSE-Agent"
           r"\238bd2e6-ebf3-4239-8620-221bf7f2d1b1\scratchpad")
SRC = SCRATCH + r"\experiment_multistep.$2k"
S2K = SCRATCH + r"\ms_test8.s2k"
SDB = SCRATCH + r"\ms_test8.sdb"

VEHICLE_ROWS = {
    "MSV1": dict(Station=0, StartTime=0),
    "MSV2": dict(Station=10, StartTime=0),
    "MSV3": dict(Station=0, StartTime=4),
}
CASE_OF = {"MSV1": "C1", "MSV2": "C2", "MSV3": "C3"}

# ── watchdog (same as exp 7) ─────────────────────────────────────────────
user32 = ctypes.windll.user32
_ENUM = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)
_stop = threading.Event()


def _dismiss():
    hits = []

    def top_cb(hwnd, _):
        cls = ctypes.create_unicode_buffer(256)
        user32.GetClassNameW(hwnd, cls, 256)
        if cls.value == "#32770" and user32.IsWindowVisible(hwnd):
            t = ctypes.create_unicode_buffer(256)
            user32.GetWindowTextW(hwnd, t, 256)
            if t.value == "SAP2000":
                hits.append(hwnd)
        return True

    user32.EnumWindows(_ENUM(top_cb), None)
    for hwnd in hits:
        texts, ok = [], []

        def child_cb(c, _):
            cls = ctypes.create_unicode_buffer(256)
            user32.GetClassNameW(c, cls, 256)
            txt = ctypes.create_unicode_buffer(2048)
            user32.GetWindowTextW(c, txt, 2048)
            if cls.value == "Static" and txt.value.strip():
                texts.append(txt.value.strip())
            if cls.value == "Button" and txt.value in ("OK", "&OK"):
                ok.append(c)
            return True

        user32.EnumChildWindows(hwnd, _ENUM(child_cb), None)
        print(f"  [watchdog] DIALOG: {(' | '.join(texts))[:300]}")
        if ok:
            user32.SendMessageW(ok[0], 0x00F5, 0, 0)


def watchdog():
    while not _stop.is_set():
        try:
            _dismiss()
        except Exception:
            pass
        time.sleep(2)


def main() -> int:
    text = open(SRC, encoding="ascii", errors="replace").read()
    old_pc = 'TABLE:  "PROGRAM CONTROL"\n   SteelCode='
    new_pc = ('TABLE:  "PROGRAM CONTROL"\n   ProgramName=SAP2000   '
              'Version=27.1.0   CurrUnits="Kip, ft, F"   SteelCode=')
    text = text.replace(old_pc, new_pc)

    # drop the old single-pattern definitions from the exp-4 model:
    # remove MSV pattern row + MSCASE tables so only our 3 new sets exist
    lines = [ln for ln in text.splitlines()
             if "LoadPat=MSV " not in ln and "Case=MSCASE" not in ln]
    text = "\n".join(lines)

    gen, veh, patdef, casedef, caseassign = [], [], [], [], []
    for p, d in VEHICLE_ROWS.items():
        patdef.append(f'   LoadPat={p}   DesignType="Vehicle Live"   SelfWtMult=0')
        gen.append(f"   LoadPat={p}   LoadDur={LOAD_DUR:g}   "
                   f"LoadDisc={LOAD_DISC:g}   SpeedFrom=Vehicle")
        veh.append(f"   LoadPat={p}   Vehicle=TESTVEH   Lane=LANE1   "
                   f"Station={d['Station']}   StartTime={d['StartTime']}   "
                   f"Direction=Forward   Speed={SPEED:g}   VertSF=1")
        c = CASE_OF[p]
        casedef.append(f'   Case={c}   Type=LinMSStat   InitialCond=Zero   '
                       f'DesTypeOpt="Prog Det"   DesignType="Vehicle Live"   '
                       f'DesActOpt="Prog Det"   DesignAct="Short-Term Composite"   '
                       f'AutoType=None   RunCase=Yes')
        caseassign.append(f'   Case={c}   LoadType="Load pattern"   '
                          f'LoadName={p}   LoadSF=1')

    block = (
        'TABLE:  "MULTI-STEP MOVING LOAD 1 - GENERAL"\n'
        + "\n".join(gen) + "\n\n"
        'TABLE:  "MULTI-STEP MOVING LOAD 2 - VEHICLE DATA"\n'
        + "\n".join(veh) + "\n\n"
        "END TABLE DATA"
    )
    text = text.replace("END TABLE DATA", block)
    # add the 3 patterns into LOAD PATTERN DEFINITIONS and cases
    text = text.replace(
        'TABLE:  "LOAD PATTERN DEFINITIONS"',
        'TABLE:  "LOAD PATTERN DEFINITIONS"\n' + "\n".join(patdef))
    text = text.replace(
        'TABLE:  "LOAD CASE DEFINITIONS"',
        'TABLE:  "LOAD CASE DEFINITIONS"\n' + "\n".join(casedef))
    text = text.replace(
        'TABLE:  "CASE - MULTISTEP STATIC 1 - LOAD ASSIGNMENTS"',
        'TABLE:  "CASE - MULTISTEP STATIC 1 - LOAD ASSIGNMENTS"\n'
        + "\n".join(caseassign))
    with open(S2K, "w", encoding="ascii") as f:
        f.write(text)
    print(f"wrote {S2K}")

    threading.Thread(target=watchdog, daemon=True).start()
    conn = SAP2000Connection()
    conn.connect(visible=True)
    m = conn.model
    ret = m.File.OpenFile(S2K)
    print(f"OpenFile: ret={ret}")
    pats = [str(n) for n in m.LoadPatterns.GetNameList(0, [])[1]]
    cases = [str(n) for n in m.LoadCases.GetNameList(0, [])[1]]
    print(f"patterns: {pats}")
    print(f"cases: {cases}")
    if not all(p in pats for p in VEHICLE_ROWS):
        print("FAILED — patterns missing")
        return 1

    assert m.File.Save(SDB) == 0
    assert m.Analyze.RunAnalysis() == 0
    setup = m.Results.Setup
    assert setup.DeselectAllCasesAndCombosForOutput() == 0
    for c in CASE_OF.values():
        assert setup.SetCaseSelectedForOutput(c) == 0
    assert setup.SetOptionMultiStepStatic(2) == 0

    # reactions at the two supports per case per step
    r = m.Results.JointReact("ALL", 2, 0, [], [], [], [], [], [], [],
                             [], [], [], [])
    assert r[-1] == 0
    n = int(r[0])
    joint = [str(v) for v in (r[1] or ())]
    case = [str(v) for v in (r[3] or ())]
    stepnum = [float(v) for v in (r[5] or ())]
    f3 = [float(v) for v in (r[8] or ())]
    supports = sorted({joint[i] for i in range(n) if abs(f3[i]) > 1e-9})
    print(f"support joints seen: {supports}")

    # implied position: a = SPAN * R_right / P  (R_right = reaction at x=60)
    # joint naming: left support is the I-joint of G1, right is J-joint of G6;
    # identify by coordinates
    def joint_x(j):
        rr = m.PointObj.GetCoordCartesian(j, 0.0, 0.0, 0.0)
        return float(rr[0])

    xs = {j: joint_x(j) for j in {joint[i] for i in range(n)}}
    right = [j for j, x in xs.items() if abs(x - SPAN) < 1e-6]
    assert right, f"no joint at x={SPAN}: {xs}"
    rj = right[0]

    for pat, c in CASE_OF.items():
        rows = sorted((stepnum[i], f3[i]) for i in range(n)
                      if joint[i] == rj and case[i] == c)
        d = VEHICLE_ROWS[pat]
        print(f"\n--- {c} ({pat}: Station={d['Station']}, "
              f"StartTime={d['StartTime']}) ---")
        print("step | R_right | implied a (ft)")
        for step, rz in rows:
            a = SPAN * rz / P_AXLE
            print(f"{step:4.0f} | {rz:8.3f} | {a:7.3f}")
    print("EXPERIMENT 8 DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
