"""
Experiment 7: s2k round trip, take 2 — with the PROGRAM CONTROL fix.

ROOT CAUSE of the exp-5 failure (found via the modal dialog text): the .$2k
auto text backup omits ProgramName/Version/CurrUnits from its PROGRAM CONTROL
table, so SAP2000's text importer reads "Version 0" and ABORTS the whole
import (leaving a blank model — that's why even MSV "disappeared").

Here:
  1. take the exp-4 backup (model with MSV Vehicle-Live pattern, MSCASE,
     lane, vehicle — everything but the multi-step data),
  2. repair PROGRAM CONTROL (ProgramName=SAP2000, Version=27.1.0, CurrUnits),
  3. splice in the MULTI-STEP MOVING LOAD 1/2 blocks,
  4. reopen, save (fresh .$2k backup), grep the backup for the blocks,
  5. run MSCASE and compare per-step midspan M3 vs exact statics.

A watchdog thread logs and dismisses any SAP2000 modal dialog (class #32770)
so a format error can't hang the run again.

Run:  C:\\Python314\\python.exe scripts/experiment_multistep_moving7.py
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
S2K = SCRATCH + r"\ms_test7.s2k"
SDB = SCRATCH + r"\ms_test7.sdb"

BLOCK = '''TABLE:  "MULTI-STEP MOVING LOAD 1 - GENERAL"
   LoadPat=MSV   LoadDur=12   LoadDisc=1   SpeedFrom=Vehicle

TABLE:  "MULTI-STEP MOVING LOAD 2 - VEHICLE DATA"
   LoadPat=MSV   Vehicle=TESTVEH   Lane=LANE1   Station=0   StartTime=0   Direction=Forward   Speed=5   VertSF=1

END TABLE DATA'''

# ── modal dialog watchdog ────────────────────────────────────────────────
user32 = ctypes.windll.user32
_ENUM = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)
dialog_log: list[str] = []


def _dismiss_dialogs() -> None:
    hits = []

    def top_cb(hwnd, _):
        cls = ctypes.create_unicode_buffer(256)
        user32.GetClassNameW(hwnd, cls, 256)
        if cls.value == "#32770" and user32.IsWindowVisible(hwnd):
            title = ctypes.create_unicode_buffer(256)
            user32.GetWindowTextW(hwnd, title, 256)
            if title.value == "SAP2000":
                hits.append(hwnd)
        return True

    user32.EnumWindows(_ENUM(top_cb), None)
    for hwnd in hits:
        texts, ok_btn = [], []

        def child_cb(child, _):
            cls = ctypes.create_unicode_buffer(256)
            user32.GetClassNameW(child, cls, 256)
            txt = ctypes.create_unicode_buffer(2048)
            user32.GetWindowTextW(child, txt, 2048)
            if cls.value == "Static" and txt.value.strip():
                texts.append(txt.value.strip())
            if cls.value == "Button" and txt.value in ("OK", "&OK", "Yes", "&Yes"):
                ok_btn.append(child)
            return True

        user32.EnumChildWindows(hwnd, _ENUM(child_cb), None)
        msg = " | ".join(texts) or "(no text)"
        dialog_log.append(msg)
        print(f"  [watchdog] DIALOG: {msg[:300]}")
        if ok_btn:
            user32.SendMessageW(ok_btn[0], 0x00F5, 0, 0)  # BM_CLICK


def watchdog():
    while not _stop.is_set():
        try:
            _dismiss_dialogs()
        except Exception:
            pass
        time.sleep(2)


_stop = threading.Event()


def backup_lines(sdb_path, pattern):
    path = sdb_path[:-4] + ".$2k"
    if not os.path.exists(path):
        return None
    txt = open(path, encoding="ascii", errors="replace").read()
    return [ln for ln in txt.splitlines() if pattern.lower() in ln.lower()]


def main() -> int:
    # 1-3. repair + splice
    text = open(SRC, encoding="ascii", errors="replace").read()
    old_pc = 'TABLE:  "PROGRAM CONTROL"\n   SteelCode='
    new_pc = ('TABLE:  "PROGRAM CONTROL"\n   ProgramName=SAP2000   '
              'Version=27.1.0   CurrUnits="Kip, ft, F"   SteelCode=')
    assert old_pc in text, "PROGRAM CONTROL block not in expected shape"
    text = text.replace(old_pc, new_pc)
    assert "END TABLE DATA" in text
    text = text.replace("END TABLE DATA", BLOCK)
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
    if "MSV" not in pats:
        print("EXPERIMENT 7 FAILED — import still dropped/aborted")
        return 1

    # 4. save; the fresh .$2k backup is the authoritative read-back
    assert m.File.Save(SDB) == 0
    blk = backup_lines(SDB, "MULTI-STEP MOVING")
    print(f"backup MULTI-STEP table headers: {blk}")
    for pat in ("LoadDur", "Vehicle=TESTVEH"):
        print(f"backup rows [{pat}]: {backup_lines(SDB, pat)}")
    if not blk:
        print("EXPERIMENT 7 — MSV survived but multi-step data dropped")
        return 1

    # 5. run + per-step check
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
    rows = sorted((stepnum[i], m3[i]) for i in range(n)
                  if obj[i] == "G4" and abs(sta[i]) < 1e-6)
    print("\nstep | t | x | M3 SAP | M3 exact | diff")
    worst = 0.0
    for step, m3v in rows:
        x = SPEED * step * LOAD_DISC
        ex = P_AXLE * min(x, SPAN - x) * 0.5 if 0 <= x <= SPAN else 0.0
        worst = max(worst, abs(m3v - ex))
        print(f"{step:4.0f} | {step*LOAD_DISC:4.1f} | {x:5.1f} | "
              f"{m3v:9.3f} | {ex:9.3f} | {m3v-ex:+.4f}")
    print(f"\nsteps={len(rows)} worst |M3-exact|={worst:.4f} kip-ft")
    ok = len(rows) >= 10 and worst < 0.5
    print("EXPERIMENT 7 DONE — " + ("SUCCESS" if ok else "CHECK RESULTS"))
    _stop.set()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
