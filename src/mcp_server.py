"""
Junior SE MCP server (FastMCP, stdio).

Exposes the deterministic structural-calc tools to Claude Code:
  Atomic   : get_section, find_lightest_W, solve_beam, check_beam, load_combos
  Workflow : design_beam  (load combos -> stiffness solve -> AISC check -> archive)

Run:  C:\\Python314\\python.exe src/mcp_server.py   (stdio transport)

stdio safety: this process speaks the MCP protocol on stdout, so NOTHING may be
print()ed to stdout. All diagnostics go to stderr via logging.
"""
from __future__ import annotations
import logging
import sys
from pathlib import Path

# --- make the project importable when run as a script -----------------------
_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from mcp.server.fastmcp import FastMCP

from src.calcs import sections, beam_stiffness, aisc360, asce7
from src.agent import project as proj

# stderr-only logging (never stdout)
logging.basicConfig(level=logging.INFO, stream=sys.stderr,
                    format="%(asctime)s junior-se %(levelname)s %(message)s")
log = logging.getLogger("junior-se")

_E_STEEL = 29_000.0   # ksi, AISC assumed modulus
_PHI_B   = 0.90       # LRFD flexure resistance factor (sizing)

_PERSONA = (Path(__file__).resolve().parent / "agent" / "prompts" / "junior_se.md").read_text(
    encoding="utf-8"
)

mcp = FastMCP("junior-se", instructions=_PERSONA)


# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------

class InputError(Exception):
    """Raised for out-of-bounds tool inputs; surfaced as a clean message."""


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise InputError(msg)


def _validate_beam_inputs(span_ft, Lb_ft, Fy_ksi, Cb) -> None:
    _require(span_ft > 0, f"span_ft must be > 0, got {span_ft}.")
    _require(span_ft <= 300, f"span_ft={span_ft} looks implausible (>300 ft) — check units.")
    _require(Lb_ft >= 0, f"Lb_ft must be >= 0, got {Lb_ft}.")
    _require(20 <= Fy_ksi <= 100, f"Fy_ksi={Fy_ksi} outside plausible range (20-100 ksi).")
    _require(Cb >= 1.0, f"Cb must be >= 1.0, got {Cb}.")


def _section_props_text(sec: dict) -> str:
    keys = ("Type", "AISC_Manual_Label", "W", "A", "d", "bf", "tf", "tw",
            "Ix", "Zx", "Sx", "rx", "Iy", "Zy", "Sy", "ry", "J", "Cw")
    return "\n".join(f"  {k:18}: {sec.get(k)}" for k in keys)


# ---------------------------------------------------------------------------
# Atomic tools
# ---------------------------------------------------------------------------

@mcp.tool()
def get_section(label: str, historical: bool = False) -> str:
    """Look up AISC section properties by designation (e.g. 'W16X40').

    Args:
        label: AISC designation, case-insensitive.
        historical: search the v16.0H historical database instead of modern v16.0.
    """
    try:
        sec = sections.get_section(label, historical=historical)
    except ValueError as e:
        return f"NOT FOUND: {e}"
    return f"AISC section {sec['AISC_Manual_Label']} (imperial units):\n" + _section_props_text(sec)


@mcp.tool()
def find_lightest_W(min_Zx_in3: float, historical: bool = False) -> str:
    """Find the lightest W-shape with plastic modulus Zx >= min_Zx_in3.

    Args:
        min_Zx_in3: minimum required plastic section modulus (in^3).
        historical: search the historical database instead of modern.
    """
    try:
        sec = sections.find_lightest_W(min_Zx_in3, historical=historical)
    except ValueError as e:
        return f"NO MATCH: {e}"
    return (f"Lightest W with Zx >= {min_Zx_in3:.1f} in^3 is "
            f"{sec['AISC_Manual_Label']} (W = {sec['W']} plf, Zx = {sec['Zx']} in^3):\n"
            + _section_props_text(sec))


@mcp.tool()
def load_combos(loads: dict[str, float]) -> str:
    """Compute ASCE 7-22 LRFD load combinations and the governing one.

    Args:
        loads: magnitudes (consistent unit) keyed by D, L, Lr, S, R, W, E (any subset).
    """
    try:
        combos = asce7.lrfd_combinations(loads)
        gov_val, gov = asce7.factored_envelope(loads)
    except ValueError as e:
        return f"INPUT ERROR: {e}"
    lines = ["ASCE 7-22 LRFD load combinations (input unit preserved):"]
    for c in combos:
        flag = "  <== GOVERNS" if c.name == gov.name else ""
        lines.append(f"  {c.name}: {c.equation} = {c.factored:.4g}   [{c.code_ref}]{flag}")
    lines.append(f"\nGoverning: {gov.name} = {gov_val:.4g}")
    return "\n".join(lines)


@mcp.tool()
def solve_beam(
    span_ft: float,
    E_ksi: float,
    I_in4: float,
    udl: list[list[float]] | None = None,
    point_loads: list[list[float]] | None = None,
) -> str:
    """Analyze a simply supported beam by the matrix stiffness method.

    Args:
        span_ft: clear span (ft).
        E_ksi: modulus of elasticity (ksi); use 29000 for steel.
        I_in4: moment of inertia about the bending axis (in^4).
        udl: list of [w_kip_per_ft, x_start_ft, x_end_ft] uniform loads (downward +).
        point_loads: list of [P_kips, x_ft] concentrated loads (downward +).
    """
    try:
        beam = beam_stiffness.SimpleBeam(span_ft, E_ksi, I_in4)
        for row in (udl or []):
            w = row[0]
            x0 = row[1] if len(row) > 1 else 0.0
            x1 = row[2] if len(row) > 2 else None
            beam.add_udl(w, x0, x1)
        for row in (point_loads or []):
            beam.add_point_load(row[0], row[1])
        res = beam.solve()
    except (ValueError, IndexError) as e:
        return f"INPUT ERROR: {e}"
    return res.summary() + "\n\n(diagram CSV available via design_beam)"


@mcp.tool()
def check_beam(
    section_label: str,
    Mu_kip_ft: float,
    Vu_kips: float,
    Lb_ft: float = 0.0,
    Fy_ksi: float = 50.0,
    Cb: float = 1.0,
) -> str:
    """AISC 360-22 flexure (Ch. F) and shear (Ch. G) check for a W-shape.

    Args:
        section_label: AISC W designation, e.g. 'W16X40'.
        Mu_kip_ft: factored flexural demand (LRFD, kip-ft).
        Vu_kips: factored shear demand (LRFD, kips).
        Lb_ft: unbraced length (ft); 0 = fully braced.
        Fy_ksi: yield stress (ksi); 50 = A992.
        Cb: moment gradient factor; 1.0 = conservative.
    """
    try:
        _validate_beam_inputs(1.0, Lb_ft, Fy_ksi, Cb)  # span not used here
        chk = aisc360.check_beam(section_label, Mu_kip_ft, Vu_kips,
                                 Lb_ft=Lb_ft, Fy_ksi=Fy_ksi, Cb=Cb)
    except NotImplementedError as e:
        return f"ESCALATE TO SENIOR ENGINEER: {e}"
    except (ValueError, InputError) as e:
        return f"INPUT ERROR: {e}"
    return chk.summary()


# ---------------------------------------------------------------------------
# Workflow tool
# ---------------------------------------------------------------------------

@mcp.tool()
def design_beam(
    span_ft: float,
    loads: dict[str, float],
    section: str | None = None,
    support: str = "simple",
    Lb_ft: float = 0.0,
    Fy_ksi: float = 50.0,
    Cb: float = 1.0,
    member_id: str = "B1",
    project: str | None = None,
    detail: str = "summary",
) -> str:
    """End-to-end simply supported steel beam design/check (the MVP workflow).

    Runs: self-weight + ASCE 7-22 LRFD combinations -> governing factored UDL ->
    matrix-stiffness analysis -> AISC 360-22 flexure & shear check -> archive the
    full calc + CSV diagram to the project's calcs/ folder.

    Args:
        span_ft: clear span (ft).
        loads: UNFACTORED distributed loads (kip/ft) keyed by D, L, Lr, S, R, W, E.
        section: AISC W designation; if omitted, the lightest adequate W is selected.
        support: support condition; only 'simple' is supported this phase.
        Lb_ft: unbraced length (ft); 0 = fully braced.
        Fy_ksi: yield stress (ksi); 50 = A992.
        Cb: moment gradient factor.
        member_id: id used for the inventory record and calc filename.
        project: project name (folder under Projects/); defaults to 'DEFAULT'.
        detail: 'summary' (default, compact + file path) or 'full' (markdown + CSV inline).
    """
    try:
        _validate_beam_inputs(span_ft, Lb_ft, Fy_ksi, Cb)
        if support != "simple":
            return (f"ESCALATE: support='{support}' not supported this phase "
                    "(only simply supported). Senior engineer to advise.")
        base_loads = asce7._norm(loads)   # validates keys/signs, fills missing with 0
        _require(any(base_loads.values()),
                 "No applied loads given — provide at least one of D, L, Lr, S, R, W, E (kip/ft).")

        # --- pick a section (size from loads without self-weight if not given) ---
        if section is None:
            w0, _ = asce7.factored_envelope(base_loads)
            _require(w0 > 0, "All factored loads are zero — nothing to design.")
            Mu0 = w0 * span_ft ** 2 / 8.0                 # kip-ft
            req_Zx = Mu0 * 12.0 / (_PHI_B * Fy_ksi)        # in^3
            sec = sections.find_lightest_W(req_Zx)
            label = sec["AISC_Manual_Label"]
            sized = True
        else:
            sec = sections.get_section(section)
            label = sec["AISC_Manual_Label"]
            sized = False

        # --- add section self-weight to dead load, recompute governing combo ---
        self_wt_klf = float(sec["W"]) / 1000.0
        loads_sw = dict(base_loads)
        loads_sw["D"] += self_wt_klf
        w, combo = asce7.factored_envelope(loads_sw)
        _require(w > 0, "Governing factored load is zero — nothing to design.")

        # --- analysis ---
        beam = beam_stiffness.SimpleBeam(span_ft, _E_STEEL, float(sec["Ix"]))
        beam.add_udl(w)
        res = beam.solve()
        Mmax = float(max(res.M_kip_ft))
        Vmax = float(max(abs(res.V_kips.min()), abs(res.V_kips.max())))

        # --- code check ---
        chk = aisc360.check_beam(label, Mmax, Vmax, Lb_ft=Lb_ft, Fy_ksi=Fy_ksi, Cb=Cb)

    except NotImplementedError as e:
        return f"ESCALATE TO SENIOR ENGINEER: {e}"
    except (ValueError, InputError) as e:
        return f"INPUT ERROR: {e}"

    # --- archive ---
    project_dir = proj.resolve_project_dir(project)
    md = _format_calc(label, span_ft, base_loads, self_wt_klf, sized,
                      Lb_ft, Fy_ksi, Cb, w, combo, res, chk, project_dir.name)
    calc_path = proj.write_calc(project_dir, member_id, md, csv=res.as_csv())
    proj.append_member(project_dir, member_id, {
        "section": label, "span_ft": span_ft, "w_klf": round(w, 4),
        "Mu_kip_ft": round(Mmax, 1), "Vu_kips": round(Vmax, 1),
        "DCR_flexure": round(chk.flexure.DCR, 3), "DCR_shear": round(chk.shear.DCR, 3),
        "limit_state": chk.flexure.limit_state, "governing_combo": combo.name,
        "calc_file": calc_path.name,
    }, bucket="beams")

    if detail == "full":
        return md + "\n\n## Diagram CSV (x, V, M, delta)\n```\n" + res.as_csv() + "\n```"

    tag = "OK" if max(chk.flexure.DCR, chk.shear.DCR) <= 1.0 else "*** OVERSTRESSED ***"
    sizing_note = " (auto-sized)" if sized else ""
    return (
        f"Beam {member_id} = {label}{sizing_note}  {tag}\n"
        f"  Governing combo : {combo.name}  {combo.equation} = {w:.3f} klf "
        f"(self-wt {self_wt_klf:.3f} klf incl.)\n"
        f"  Demand          : Mu = {Mmax:,.1f} kip-ft, Vu = {Vmax:,.1f} kips\n"
        f"  Flexure         : phiMn = {chk.flexure.phi_Mn_kip_ft:,.1f} kip-ft, "
        f"DCR = {chk.flexure.DCR:.3f} ({chk.flexure.limit_state}, {chk.flexure.code_ref})\n"
        f"  Shear           : phiVn = {chk.shear.phi_Vn_kips:,.1f} kips, "
        f"DCR = {chk.shear.DCR:.3f} ({chk.shear.code_ref})\n"
        f"  Full calc + CSV : {calc_path}\n"
        f"  (Self-weight in D; ASCE 7 §4.7 live-load reduction not applied; "
        f"single self-weight pass.)"
    )


def _format_calc(label, span_ft, base_loads, self_wt_klf, sized, Lb_ft, Fy_ksi, Cb,
                 w, combo, res, chk, project_name) -> str:
    from datetime import date
    load_str = ", ".join(f"{k}={v:.3f}" for k, v in base_loads.items() if v) or "none"
    return "\n".join([
        f"# Beam Design — {label}   (project: {project_name})",
        f"Date: {date.today().isoformat()}  |  Code: AISC 360-22, ASCE 7-22 LRFD  |  Units: imperial",
        "",
        "## Inputs",
        f"- Span: {span_ft:.2f} ft (simply supported)",
        f"- Unfactored distributed loads (kip/ft): {load_str}",
        f"- Section self-weight added to D: {self_wt_klf:.3f} kip/ft"
        + ("  [section auto-sized]" if sized else "  [section specified]"),
        f"- Unbraced length Lb = {Lb_ft:.2f} ft, Fy = {Fy_ksi:.0f} ksi, Cb = {Cb:.2f}",
        "",
        "## Governing load combination (ASCE 7-22)",
        f"- {combo.name}: {combo.equation} = **{w:.3f} kip/ft**  ({combo.code_ref})",
        "",
        "## Analysis — matrix stiffness method",
        "```",
        res.summary(),
        "```",
        "",
        "## AISC 360-22 capacity check",
        "```",
        chk.summary(),
        "```",
        "",
        "## Assumptions & limitations",
        "- Self-weight of the selected section is included in dead load.",
        "- ASCE 7 §4.7 live-load reduction not applied (pass reduced L if desired).",
        "- Single self-weight pass; reported DCR reflects the final selected section.",
        "- Confirmed: span, loads, support per engineer input. Review before use.",
    ])


if __name__ == "__main__":
    log.info("starting junior-se MCP server (stdio)")
    mcp.run()
