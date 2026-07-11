"""
Experiment 9: multi-axle calibration of the multi-step moving load.

HS20-shaped train (8 / 32 / 32 kip at 14 ft spacings) stepped at 5 ft/s.
Check every step's LEFT+RIGHT reactions against exact statics under the
hypothesis: lead axle at a1 = Station + Speed*t - 1 (t=(step-1)*disc),
axle i at a1 - d_i (d = 0, 14, 28 — trailing axles BEHIND the lead axle),
axles off the span carry nothing.

Also verifies the trailing zero-load steps and total-load ramp as axles
enter/leave — i.e. the whole stepping mechanics for a real truck.

Run:  C:\\Python314\\python.exe scripts/experiment_multistep_moving9.py
"""
from __future__ import annotations

import ctypes
import sys
import threading
import time

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402

SPAN, SPEED, LOAD_DUR, LOAD_DISC = 60.0, 5.0, 24.0, 1.0
AXLES = [8.0, 32.0, 32.0]        # kip, front to back
SPACINGS = [14.0, 14.0]          # ft between consecutive axles
STATION, START_TIME = 0.0, 0.0
LEAD_OFFSET = 1.0                # ft: front-of-vehicle to lead axle (exp-8)
SCRATCH = (r"C:\Users\richa\AppData\Local\Temp\claude"
           r"\D--AI-TEST-Agent-Developer-Project-006-JuniorSE-Agent"
           r"\238bd2e6-ebf3-4239-8620-221bf7f2d1b1\scratchpad")
SRC = SCRATCH + r"\experiment_multistep.$2k"
S2K = SCRATCH + r"\ms_test9.s2k"
SDB = SCRATCH + r"\ms_test9.sdb"

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


def exact_reactions(a_lead):
    """(R_left, R_right) for the train with lead axle at a_lead."""
    rl = rr = 0.0
    d = 0.0
    for i, p in enumerate(AXLES):
        if i:
            d += SPACINGS[i - 1]
        a = a_lead - d
        if 0.0 <= a <= SPAN:
            rr += p * a / SPAN
            rl += p * (SPAN - a) / SPAN
    return rl, rr


def main() -> int:
    text = open(SRC, encoding="ascii", errors="replace").read()
    text = text.replace(
        'TABLE:  "PROGRAM CONTROL"\n   SteelCode=',
        'TABLE:  "PROGRAM CONTROL"\n   ProgramName=SAP2000   '
        'Version=27.1.0   CurrUnits="Kip, ft, F"   SteelCode=')
    # strip exp-4 leftovers (MSV pattern, MSCASE, single-axle TESTVEH rows)
    lines = [ln for ln in text.splitlines()
             if "LoadPat=MSV " not in ln and "Case=MSCASE" not in ln
             and "TESTVEH" not in ln]
    text = "\n".join(lines)

    veh_general = "   VehName=TRAIN   StayInLane=Yes"
    veh_loads = ["   VehName=TRAIN   LoadType=\"Leading Load\"   "
                 f"InterUnif=0   InterAxle={AXLES[0]:g}"]
    for p, s in zip(AXLES[1:], SPACINGS):
        veh_loads.append(f"   VehName=TRAIN   LoadType=\"Fixed Length\"   "
                         f"InterUnif=0   InterAxle={p:g}   InterMinD={s:g}")
    veh_class = "   VehClass=TRAIN   VehName=TRAIN   ScaleFactor=1"

    text = text.replace(
        'TABLE:  "VEHICLES 2 - GENERAL VEHICLES 1 - GENERAL"',
        'TABLE:  "VEHICLES 2 - GENERAL VEHICLES 1 - GENERAL"\n' + veh_general)
    text = text.replace(
        'TABLE:  "VEHICLES 3 - GENERAL VEHICLES 2 - LOADS"',
        'TABLE:  "VEHICLES 3 - GENERAL VEHICLES 2 - LOADS"\n'
        + "\n".join(veh_loads))
    text = text.replace(
        'TABLE:  "VEHICLES 4 - VEHICLE CLASSES"',
        'TABLE:  "VEHICLES 4 - VEHICLE CLASSES"\n' + veh_class)
    text = text.replace(
        'TABLE:  "LOAD PATTERN DEFINITIONS"',
        'TABLE:  "LOAD PATTERN DEFINITIONS"\n'
        '   LoadPat=MST   DesignType="Vehicle Live"   SelfWtMult=0')
    text = text.replace(
        'TABLE:  "LOAD CASE DEFINITIONS"',
        'TABLE:  "LOAD CASE DEFINITIONS"\n'
        '   Case=CT   Type=LinMSStat   InitialCond=Zero   '
        'DesTypeOpt="Prog Det"   DesignType="Vehicle Live"   '
        'DesActOpt="Prog Det"   DesignAct="Short-Term Composite"   '
        'AutoType=None   RunCase=Yes')
    text = text.replace(
        'TABLE:  "CASE - MULTISTEP STATIC 1 - LOAD ASSIGNMENTS"',
        'TABLE:  "CASE - MULTISTEP STATIC 1 - LOAD ASSIGNMENTS"\n'
        '   Case=CT   LoadType="Load pattern"   LoadName=MST   LoadSF=1')
    block = (
        'TABLE:  "MULTI-STEP MOVING LOAD 1 - GENERAL"\n'
        f"   LoadPat=MST   LoadDur={LOAD_DUR:g}   LoadDisc={LOAD_DISC:g}   "
        "SpeedFrom=Vehicle\n\n"
        'TABLE:  "MULTI-STEP MOVING LOAD 2 - VEHICLE DATA"\n'
        f"   LoadPat=MST   Vehicle=TRAIN   Lane=LANE1   Station={STATION:g}   "
        f"StartTime={START_TIME:g}   Direction=Forward   Speed={SPEED:g}   "
        "VertSF=1\n\n"
        "END TABLE DATA")
    text = text.replace("END TABLE DATA", block)
    with open(S2K, "w", encoding="ascii") as f:
        f.write(text)
    print(f"wrote {S2K}")

    threading.Thread(target=watchdog, daemon=True).start()
    conn = SAP2000Connection()
    conn.connect(visible=True)
    m = conn.model
    print(f"OpenFile: ret={m.File.OpenFile(S2K)}")
    pats = [str(n) for n in m.LoadPatterns.GetNameList(0, [])[1]]
    print(f"patterns: {pats}")
    if "MST" not in pats:
        print("FAILED — pattern missing")
        return 1
    assert m.File.Save(SDB) == 0
    assert m.Analyze.RunAnalysis() == 0
    setup = m.Results.Setup
    assert setup.DeselectAllCasesAndCombosForOutput() == 0
    assert setup.SetCaseSelectedForOutput("CT") == 0
    assert setup.SetOptionMultiStepStatic(2) == 0

    r = m.Results.JointReact("ALL", 2, 0, [], [], [], [], [], [], [],
                             [], [], [], [])
    assert r[-1] == 0
    n = int(r[0])
    joint = [str(v) for v in (r[1] or ())]
    case = [str(v) for v in (r[3] or ())]
    stepnum = [float(v) for v in (r[5] or ())]
    f3 = [float(v) for v in (r[8] or ())]

    def joint_x(j):
        rr = m.PointObj.GetCoordCartesian(j, 0.0, 0.0, 0.0)
        return float(rr[0])

    xs = {j: joint_x(j) for j in {joint[i] for i in range(n)}}
    lj = [j for j, x in xs.items() if abs(x) < 1e-6][0]
    rj = [j for j, x in xs.items() if abs(x - SPAN) < 1e-6][0]

    R = {}
    for i in range(n):
        if case[i] == "CT" and joint[i] in (lj, rj):
            R.setdefault(stepnum[i], {})[joint[i]] = f3[i]

    print("\nstep | a_lead | RL sap | RL exact | RR sap | RR exact | ok")
    worst = 0.0
    for step in sorted(R):
        t = (step - 1) * LOAD_DISC
        a_lead = STATION + SPEED * max(0.0, t - START_TIME) - LEAD_OFFSET
        rl_e, rr_e = exact_reactions(a_lead)
        rl_s, rr_s = R[step].get(lj, 0.0), R[step].get(rj, 0.0)
        err = max(abs(rl_s - rl_e), abs(rr_s - rr_e))
        worst = max(worst, err)
        print(f"{step:4.0f} | {a_lead:6.1f} | {rl_s:7.3f} | {rl_e:8.3f} | "
              f"{rr_s:7.3f} | {rr_e:8.3f} | {'OK' if err < 1e-3 else 'X'}")
    print(f"\nworst reaction error = {worst:.6f} kip")
    print("EXPERIMENT 9 DONE — " + ("SUCCESS" if worst < 1e-3 else "MISMATCH"))
    _stop.set()
    return 0 if worst < 1e-3 else 1


if __name__ == "__main__":
    raise SystemExit(main())
