"""
LIVE integration test for the sap_* MCP bridge tools (src/agent/sap2000_tools.py).

Exercises the FULL stack, exactly as Claude Code would use it:

    MCP client (this script, stdio) -> src/mcp_server.py (FastMCP)
      -> sap_* tool -> httpx -> P005 REST API -> SAP2000 COM

Model (IMPERIAL, kip_ft): 30 ft x 24 ft concrete deck (t = 6 in) on four
W24X94 girders spanning X at y = 0/8/16/24 ft, supported on eight 12-ft
W14X90 columns (fixed bases) at the girder ends. SDL = 20 psf, LL = 50 psf.

Statics gates (script-computed, no hand numbers):
  - sum LL reactions  == 0.050 ksf * 30 ft * 24 ft
  - sum SDL reactions == 0.020 ksf * 30 ft * 24 ft
  - sum DEAD reactions == slab + girder + column self-weight
    (steel SW = A_AISC/144 * 0.490 kcf, areas from data/AISC/sections_v16.json)
  - column symmetry groups equal under LL
  - column axial P (frame_forces) == its base reaction F3 (joint_reactions)
  - LC1/LC2 factored reaction sums == 1.4D / 1.2D+1.6L on the measured
    per-case sums (linear superposition through SAP2000 RespCombos)

Prerequisites:
  - P005 server running:  cd D:\\AI_TEST\\Agent_Developer\\Project_005_SAP2000api_v3
                          C:\\Python314\\python.exe -m uvicorn src.backend.main:app --port 8000
  - SAP2000 installed (script attaches or launches via sap_connect)

Run:  C:\\Python314\\python.exe scripts\\test_sap_bridge_live.py
"""
from __future__ import annotations

import asyncio
import json
import sys
from datetime import timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

from src.calcs.sections import get_section

PYTHON = r"C:\Python314\python.exe"
SERVER = str(ROOT / "src" / "mcp_server.py")
SAVE_PATH = r"D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3\outputs\sap_bridge_live_kipft.sdb"

# ── Model definition (kip_ft) ────────────────────────────────────────────────
SPAN_X = 30.0          # ft, girder span
WIDTH_Y = 24.0         # ft, three 8-ft bays
BAY_Y = 8.0
SLAB_T = 0.5           # ft (6 in)
CONC_WT = 0.150        # kip/ft^3
SDL_KSF = 0.020        # 20 psf
LL_KSF = 0.050         # 50 psf
GIRDER = "W24X94"
COLUMN = "W14X90"
COL_H = 12.0           # ft
STEEL_KCF = 0.490      # A992 default in SAP2000 (== 76.9729 kN/m^3, verified 2026-07-04)

GIRDER_YS = [0.0, 8.0, 16.0, 24.0]
COL_POSITIONS = [[x, y] for x in (0.0, SPAN_X) for y in GIRDER_YS]

MODEL = {
    "project": {
        "name": "sap_* bridge live integration test",
        "unit_system": "kip_ft",
        "structure_type": "bridge_deck",
    },
    "grid": {"x_spacings": [SPAN_X], "y_spacings": [BAY_Y, BAY_Y, BAY_Y]},
    "girders": {
        "direction": "X",
        "section": {"name": "GIRD", "section_type": GIRDER},
        "row_indices": [0, 1, 2, 3],
    },
    "piles": [],  # supports come from sap_add_columns
    "slab": {
        "thickness": SLAB_T,
        "concrete_fc": 4.0,       # ksi (imperial model)
        "unit_weight": CONC_WT,   # kip/ft^3
        "mesh_size": 2.0,         # ft
    },
    "loads": {"dead_load": SDL_KSF, "live_load": LL_KSF},
}

# ── Expected statics (computed, imperial) ────────────────────────────────────
DECK_AREA = SPAN_X * WIDTH_Y
EXP_LL = LL_KSF * DECK_AREA
EXP_SDL = SDL_KSF * DECK_AREA
_w_girder = get_section(GIRDER)["A"] / 144.0 * STEEL_KCF   # kip/ft
_w_column = get_section(COLUMN)["A"] / 144.0 * STEEL_KCF   # kip/ft
EXP_DEAD = (DECK_AREA * SLAB_T * CONC_WT
            + len(GIRDER_YS) * SPAN_X * _w_girder
            + len(COL_POSITIONS) * COL_H * _w_column)

SAP_TOOL_NAMES = [
    "sap_status", "sap_connect", "sap_close", "sap_build_model",
    "sap_list_operations", "sap_run_analysis", "sap_find_joints",
    "sap_add_columns", "sap_define_load_combos", "sap_list_load_cases",
    "sap_joint_reactions", "sap_joint_displacements", "sap_frame_forces",
    "sap_base_reactions", "sap_modal_periods",
]

PASS = True


def check(label: str, ok: bool, detail: str = "") -> None:
    global PASS
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f" — {detail}" if detail else ""))
    PASS = PASS and bool(ok)


def parse_payload(text: str) -> dict:
    """sap_* tools _dump() JSON, possibly followed by a truncation NOTE."""
    return json.loads(text.split("\n\nNOTE:")[0])


async def call(session: ClientSession, name: str, args: dict | None = None) -> str:
    res = await session.call_tool(name, args or {})
    text = "".join(c.text for c in res.content if getattr(c, "type", "") == "text")
    if res.isError:
        raise RuntimeError(f"{name} MCP error: {text[:500]}")
    if text.startswith("SAP2000 BRIDGE ERROR"):
        raise RuntimeError(f"{name}: {text[:500]}")
    return text


def sum_reactions(rows: list[dict], joint_key: str = "joint",
                  comp: str = "F3") -> float:
    """Sum a reaction component with one value per joint (combos may return
    Max/Min step rows — all equal for linear-additive combos of static cases,
    so the per-joint max is the value)."""
    per_joint: dict[str, float] = {}
    for r in rows:
        j = r[joint_key]
        per_joint[j] = max(per_joint.get(j, float("-inf")), r[comp])
    return sum(per_joint.values())


async def main() -> int:
    print("=" * 74)
    print("sap_* MCP bridge — LIVE integration test (kip_ft deck on columns)")
    print("=" * 74)
    print(f"Expected totals [kips]: LL={EXP_LL:.3f}  SDL={EXP_SDL:.3f}  "
          f"DEAD={EXP_DEAD:.3f} (slab {DECK_AREA * SLAB_T * CONC_WT:.2f} + "
          f"girders {len(GIRDER_YS) * SPAN_X * _w_girder:.3f} + "
          f"columns {len(COL_POSITIONS) * COL_H * _w_column:.3f})")

    params = StdioServerParameters(command=PYTHON, args=[SERVER])
    async with stdio_client(params) as (read, write):
        async with ClientSession(
                read, write,
                read_timeout_seconds=timedelta(seconds=900)) as s:
            await s.initialize()

            print("\n== MCP layer ==")
            tools = {t.name for t in (await s.list_tools()).tools}
            missing = [n for n in SAP_TOOL_NAMES if n not in tools]
            check("all 15 sap_* tools registered over MCP", not missing,
                  f"missing: {missing}" if missing else f"{len(tools)} tools total")

            print("\n== sap_status / sap_connect ==")
            text = await call(s, "sap_status")
            check("P005 API server is UP", "P005 API server: UP" in text, text.replace("\n", " | "))
            text = await call(s, "sap_connect", {"visible": True})
            check("sap_connect ok", "error" not in text.lower(), text)

            print("\n== sap_list_operations ==")
            text = await call(s, "sap_list_operations")
            ops_expected = ["add_columns", "base_reactions", "define_load_combos",
                            "find_joints", "frame_forces", "joint_displacements",
                            "joint_reactions", "list_load_cases", "modal_periods",
                            "run_analysis"]
            missing = [o for o in ops_expected if o not in text]
            check("all 10 P005 operations listed", not missing,
                  f"missing: {missing}" if missing else "10/10")

            print("\n== sap_build_model (kip_ft deck, no supports yet) ==")
            buildpay = parse_payload(await call(s, "sap_build_model", {"model": MODEL}))
            check("build status == success", buildpay.get("status") == "success",
                  str(buildpay.get("status")))
            report = buildpay.get("report", {})
            errors = report.get("errors") or []
            check("build report has no errors", not errors, str(errors)[:300])
            n_areas = len(report.get("areas") or [])
            check("slab meshed into area elements", n_areas > 0, f"{n_areas} areas")
            girders = report.get("frames") or []
            check("4 girders built", len(girders) == len(GIRDER_YS), str(girders)[:300])

            print("\n== sap_add_columns (8x W14X90, 12 ft, fixed bases) ==")
            colpay = parse_payload(await call(s, "sap_add_columns", {
                "positions": COL_POSITIONS, "height": COL_H,
                "section_shape": COLUMN, "section_name": "COLSEC",
                "material": "steel", "unit_system": "kip_ft",
                "fix_base": True}))
            cols = colpay.get("columns") or []
            check("8 columns created", len(cols) == len(COL_POSITIONS), f"{len(cols)} columns")
            check("bases fixed at z = -12 ft",
                  colpay.get("fixed_base") is True and colpay.get("base_z") == -COL_H,
                  f"base_z={colpay.get('base_z')}")

            print("\n== sap_run_analysis (save + run) ==")
            runpay = parse_payload(await call(s, "sap_run_analysis",
                                              {"save_path": SAVE_PATH}))
            status = {c["name"]: c["run_status"] for c in runpay.get("cases", [])}
            for case in ("DEAD", "SDL", "LL"):
                check(f"case {case} finished", status.get(case) == "finished",
                      str(status))

            print("\n== sap_list_load_cases ==")
            lcpay = parse_payload(await call(s, "sap_list_load_cases"))
            names = {c["name"] for c in lcpay.get("cases", [])}
            check("DEAD/SDL/LL cases exist", {"DEAD", "SDL", "LL"} <= names, str(names))

            print("\n== sap_base_reactions vs statics ==")
            basepay = parse_payload(await call(s, "sap_base_reactions",
                                               {"cases": ["DEAD", "SDL", "LL"]}))
            fz = {r["case"]: r["FZ"] for r in basepay.get("base_reactions", [])}
            check(f"LL base FZ == {EXP_LL:.3f} kips",
                  abs(fz.get("LL", 0) - EXP_LL) < 1e-6, f"{fz.get('LL'):.6f}")
            check(f"SDL base FZ == {EXP_SDL:.3f} kips",
                  abs(fz.get("SDL", 0) - EXP_SDL) < 1e-6, f"{fz.get('SDL'):.6f}")
            check(f"DEAD base FZ == {EXP_DEAD:.3f} kips (0.2% band, SW model)",
                  abs(fz.get("DEAD", 0) - EXP_DEAD) < 0.002 * EXP_DEAD,
                  f"{fz.get('DEAD'):.4f} vs {EXP_DEAD:.4f} "
                  f"(ratio {fz.get('DEAD', 0) / EXP_DEAD:.5f})")

            print("\n== sap_joint_reactions (LL): equilibrium + symmetry ==")
            jrpay = parse_payload(await call(s, "sap_joint_reactions", {"cases": ["LL"]}))
            rows = jrpay.get("reactions", [])
            joints = {r["joint"] for r in rows}
            check("8 restrained joints report reactions", len(joints) == 8, str(sorted(joints)))
            total = sum_reactions(rows)
            check(f"sum LL F3 == {EXP_LL:.3f} kips", abs(total - EXP_LL) < 1e-6,
                  f"{total:.6f}")
            # symmetry groups: corner columns (y=0,24) vs interior-edge (y=8,16)
            base_names = {c_["base_joint"]: (c_["x"], c_["y"]) for c_ in cols}
            f3 = {r["joint"]: r["F3"] for r in rows}
            corner = [f3[j] for j, (x, y) in base_names.items() if y in (0.0, WIDTH_Y)]
            interior = [f3[j] for j, (x, y) in base_names.items() if y in (BAY_Y, 2 * BAY_Y)]
            check("4 corner reactions equal (double symmetry)",
                  len(corner) == 4 and max(corner) - min(corner) < 1e-6,
                  f"{[f'{v:.4f}' for v in corner]}")
            check("4 interior-line reactions equal (double symmetry)",
                  len(interior) == 4 and max(interior) - min(interior) < 1e-6,
                  f"{[f'{v:.4f}' for v in interior]}")

            print("\n== sap_find_joints ==")
            fjpay = parse_payload(await call(s, "sap_find_joints", {
                "coords": [[0.0, 0.0, -COL_H]], "tol": 0.01}))
            hit = fjpay["joints"][0]
            col0 = next(c_ for c_ in cols if c_["x"] == 0.0 and c_["y"] == 0.0)
            check("column base joint found at (0, 0, -12)",
                  hit["matched"] and hit["joint"] == col0["base_joint"],
                  f"joint {hit['joint']} vs {col0['base_joint']}, d={hit['distance']:.4g}")
            fjpay = parse_payload(await call(s, "sap_find_joints", {
                "coords": [[SPAN_X / 2, WIDTH_Y / 2, 0.0]], "tol": 1.1}))
            center = fjpay["joints"][0]
            check("deck joint near center found (within half mesh)",
                  center["matched"], f"{center['joint']} at {center['coords']}")

            print("\n== sap_joint_displacements (LL) ==")
            jdpay = parse_payload(await call(s, "sap_joint_displacements", {
                "cases": ["LL"], "joints": [center["joint"], col0["base_joint"]]}))
            dz = {r["joint"]: r["U3"] for r in jdpay.get("displacements", [])}
            check("deck center deflects DOWN under LL", dz[center["joint"]] < -1e-9,
                  f"{dz[center['joint']] * 12:.5f} in")
            check("column base does not move", abs(dz[col0["base_joint"]]) < 1e-12)

            print("\n== sap_frame_forces (LL): column axial vs its reaction ==")
            ffpay = parse_payload(await call(s, "sap_frame_forces", {
                "cases": ["LL"], "frames": [col0["name"]]}))
            frows = ffpay.get("frame_forces", [])
            check("column force rows returned", len(frows) >= 2, f"{len(frows)} stations")
            p_vals = [r["P"] for r in frows]
            r_f3 = f3[col0["base_joint"]]
            check("column P constant along height (no load in LL)",
                  max(p_vals) - min(p_vals) < 1e-6,
                  f"P in [{min(p_vals):.6f}, {max(p_vals):.6f}]")
            check("|column P| == base reaction F3 (cross-op consistency)",
                  abs(abs(p_vals[0]) - r_f3) < 1e-6,
                  f"|P|={abs(p_vals[0]):.6f} vs F3={r_f3:.6f}")
            check("column P is compression (negative)", p_vals[0] < 0.0,
                  f"{p_vals[0]:.6f}")

            print("\n== sap_define_load_combos (D=[DEAD,SDL], L=LL) ==")
            cbpay = parse_payload(await call(s, "sap_define_load_combos", {
                "case_map": {"D": ["DEAD", "SDL"], "L": "LL"}}))
            created = {c["name"] for c in cbpay.get("combos", [])}
            check("LC1 and LC2 created", {"LC1", "LC2"} <= created, str(created))
            check("LRFD-ENV envelope created", cbpay.get("envelope") == "LRFD-ENV",
                  str(cbpay.get("envelope")))

            print("\n== factored combo reactions vs measured case sums ==")
            jr2 = parse_payload(await call(s, "sap_joint_reactions",
                                           {"cases": ["LC1", "LC2"]}))
            rows2 = jr2.get("reactions", [])
            lc1 = sum_reactions([r for r in rows2 if r["case"] == "LC1"])
            lc2 = sum_reactions([r for r in rows2 if r["case"] == "LC2"])
            exp_lc1 = 1.4 * (fz["DEAD"] + fz["SDL"])
            exp_lc2 = 1.2 * (fz["DEAD"] + fz["SDL"]) + 1.6 * fz["LL"]
            check(f"LC1 sum F3 == 1.4D == {exp_lc1:.3f}", abs(lc1 - exp_lc1) < 1e-5,
                  f"{lc1:.6f}")
            check(f"LC2 sum F3 == 1.2D + 1.6L == {exp_lc2:.3f}",
                  abs(lc2 - exp_lc2) < 1e-5, f"{lc2:.6f}")

            print("\n== sap_modal_periods (no modal case -> graceful) ==")
            try:
                text = await call(s, "sap_modal_periods")
                check("modal_periods handled gracefully", True,
                      text.replace("\n", " ")[:100])
            except RuntimeError as exc:
                check("modal_periods raised a clean bridge error", True, str(exc)[:100])

    print("\n" + "=" * 74)
    print("ALL CHECKS PASS" if PASS else "SOME CHECKS FAILED")
    print(f"Model saved to {SAVE_PATH} (SAP2000 left running)")
    return 0 if PASS else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
