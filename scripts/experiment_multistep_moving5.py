"""
Experiment 5: s2k text-file round trip for the Multi-Step Moving Load data.

Learned so far (exp 2-4): lane/vehicle/pattern/case all build fine via COM +
interactive DB import, and pattern type 9 == DesignType "Vehicle Live"; but
the "Multi-Step Moving Load 1/2" interactive-table imports report success and
silently store NOTHING (confirmed by s2k export — no such block). The
interactive importer even rejects DesignType="Vehicle Live" that the s2k
exporter itself writes, so the TEXT importer is a different (richer) parser.

Here: append the two MULTI-STEP MOVING LOAD blocks to the exported s2k text,
reopen it, re-export, and check the blocks survived. Then run the multistep
case and compare per-step midspan M3 vs exact statics.

Run:  C:\\Python314\\python.exe scripts/experiment_multistep_moving5.py
"""
from __future__ import annotations

import shutil
import sys

P005_ROOT = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3"
sys.path.insert(0, P005_ROOT)

from src.backend.services.sap2000.connector import SAP2000Connection  # noqa: E402

SCRATCH = (r"C:\Users\richa\AppData\Local\Temp\claude"
           r"\D--AI-TEST-Agent-Developer-Project-006-JuniorSE-Agent"
           r"\238bd2e6-ebf3-4239-8620-221bf7f2d1b1\scratchpad")
SRC_TXT = SCRATCH + r"\experiment_multistep.$2k"     # exported by exp 4
S2K_IN = SCRATCH + r"\ms_test.s2k"
SDB_OUT = SCRATCH + r"\ms_test_reopened.sdb"

SPAN, P_AXLE, SPEED, LOAD_DUR, LOAD_DISC = 60.0, 30.0, 5.0, 12.0, 1.0

BLOCK = '''TABLE:  "MULTI-STEP MOVING LOAD 1 - GENERAL"
   LoadPat=MSV   LoadDur=12   LoadDisc=1   SpeedFrom=Vehicle

TABLE:  "MULTI-STEP MOVING LOAD 2 - VEHICLE DATA"
   LoadPat=MSV   Vehicle=TESTVEH   Lane=LANE1   Station=0   StartTime=0   Direction=Forward   Speed=5   VertSF=1

END TABLE DATA'''


def exact_midspan_m(x):
    if x < 0 or x > SPAN:
        return 0.0
    return P_AXLE * min(x, SPAN - x) * 0.5


def main() -> int:
    # 1. splice the block into the s2k text
    text = open(SRC_TXT, encoding="utf-8", errors="replace").read()
    assert "END TABLE DATA" in text
    assert "MULTI-STEP" not in text.upper().replace("CASE - MULTISTEP", "")
    out = text.replace("END TABLE DATA", BLOCK)
    with open(S2K_IN, "w", encoding="utf-8") as f:
        f.write(out)
    print(f"wrote {S2K_IN}")

    # 2. reopen it
    conn = SAP2000Connection()
    conn.connect(visible=True)
    m = conn.model
    ret = m.File.OpenFile(S2K_IN)
    print(f"OpenFile(s2k): ret={ret}")
    if ret != 0:
        return 1
    pats = [str(n) for n in m.LoadPatterns.GetNameList(0, [])[1]]
    print(f"patterns after reopen: {pats}")

    # 3. re-export and check the block survived
    reexport = SCRATCH + r"\ms_test_out.s2k"
    ret = m.File.Save(reexport)
    print(f"re-export ret={ret}")
    txt2 = open(SCRATCH + r"\ms_test_out.$2k", encoding="utf-8",
                errors="replace").read()
    keep = [ln for ln in txt2.splitlines()
            if "MULTI-STEP MOVING" in ln.upper()
            or ("LoadPat=MSV" in ln and "Veh" in ln)
            or ln.strip().startswith("LoadPat=MSV   LoadDur")]
    print("re-exported multi-step lines:")
    for ln in keep:
        print("   " + ln)
    survived = any("MULTI-STEP MOVING" in ln.upper() for ln in keep)
    print(f"multi-step data survived reopen: {survived}")
    if not survived:
        print("EXPERIMENT 5 DONE — TEXT IMPORT ALSO DROPS IT")
        return 1

    # 4. run + per-step check
    ret = m.File.Save(SDB_OUT)
    print(f"save sdb ret={ret}")
    assert m.Analyze.RunAnalysis() == 0
    setup = m.Results.Setup
    assert setup.DeselectAllCasesAndCombosForOutput() == 0
    assert setup.SetCaseSelectedForOutput("MSCASE") == 0
    assert setup.SetOptionMultiStepStatic(2) == 0

    r = m.Results.FrameForce("ALL", 2, 0, [], [], [], [], [], [], [],
                             [], [], [], [], [], [])
    assert r[-1] == 0
    n = int(r[0])
    obj = [str(v) for v in (r[1] or ())]
    sta = [float(v) for v in (r[2] or ())]
    stepnum = [float(v) for v in (r[7] or ())]
    m3 = [float(v) for v in (r[13] or ())]
    steps = sorted(set(stepnum))
    print(f"FrameForce rows={n}; unique steps={steps}")

    rows = sorted((stepnum[i], m3[i]) for i in range(n)
                  if obj[i] == "G4" and abs(sta[i]) < 1e-6)
    print("\nstep | t | x_axle | M3 SAP | M3 exact | diff")
    worst = 0.0
    for step, m3v in rows:
        t = step * LOAD_DISC
        x = SPEED * t
        ex = exact_midspan_m(x)
        worst = max(worst, abs(m3v - ex))
        print(f"{step:4.0f} | {t:4.1f} | {x:5.1f} | {m3v:9.3f} | {ex:9.3f} | {m3v-ex:+.4f}")
    print(f"worst |M3-exact| = {worst:.4f} kip-ft over {len(rows)} steps")
    print("EXPERIMENT 5 DONE — " + ("SUCCESS" if rows and worst < 1.0 else "CHECK"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
