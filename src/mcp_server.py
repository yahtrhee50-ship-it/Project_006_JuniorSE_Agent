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
from src.calcs import rebar as _rebar
from src.calcs import aci318
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


# ===========================================================================
# ACI 318-19 Concrete tools
# ===========================================================================

_PHI_FLEX_CONCRETE = 0.90   # for sizing pass

_FYT_DEFAULT_KSI = 60.0     # stirrup yield strength default


def _validate_concrete_inputs(
    b_in, h_in, fc_ksi, fy_ksi, span_ft
) -> None:
    _require(b_in > 0,     f"b_in must be > 0, got {b_in}.")
    _require(h_in > b_in,  f"h_in ({h_in}) should exceed b_in ({b_in}) for a beam.")
    _require(0.5 <= fc_ksi <= 20.0,
             f"fc_ksi={fc_ksi} outside range 0.5–20 ksi — check units (enter ksi, not psi).")
    _require(40 <= fy_ksi <= 100,
             f"fy_ksi={fy_ksi} outside range 40–100 ksi.")
    _require(span_ft > 0, f"span_ft must be > 0, got {span_ft}.")
    _require(span_ft <= 200, f"span_ft={span_ft} seems implausible (>200 ft) — check units.")


def _effective_depth(h_in: float, cover_in: float, stirrup_bar: str, main_bar: str) -> float:
    """d = h − cover − stirrup_db − main_db/2."""
    s_db = _rebar.get_bar(stirrup_bar)["db_in"]
    m_db = _rebar.get_bar(main_bar)["db_in"]
    return h_in - cover_in - s_db - m_db / 2.0


def _concrete_calc_header(label, b_in, h_in, fc_ksi, fy_ksi, span_ft, project_name) -> str:
    from datetime import date
    return "\n".join([
        f"# Concrete Beam — {label}   (project: {project_name})",
        f"Date: {date.today().isoformat()}  |  Code: ACI 318-19, ASCE 7-22 LRFD  |  Units: imperial",
        f"Section: {b_in:.0f} in × {h_in:.0f} in,  f'c = {fc_ksi:.1f} ksi,  "
        f"fy = {fy_ksi:.0f} ksi,  span = {span_ft:.1f} ft",
    ])


@mcp.tool()
def get_rebar(designation: str) -> str:
    """Look up standard rebar properties by designation.

    Args:
        designation: bar number, e.g. '#8', '#10', '5'. Available: #3–#11, #14, #18.
    """
    try:
        bar = _rebar.get_bar(designation)
    except ValueError as e:
        return f"NOT FOUND: {e}"
    d   = designation.strip()
    return (
        f"Rebar {d} (ASTM A615/A706):\n"
        f"  db  = {bar['db_in']:.4f} in\n"
        f"  Ab  = {bar['Ab_in2']:.2f} in²\n"
        f"  wt  = {bar['wt_plf']:.3f} lb/ft\n"
        f"  T-head eligible: {'Yes' if d in _rebar.THEAD_ELIGIBLE else 'No (#14/#18 excluded)'}"
    )


@mcp.tool()
def check_concrete_beam(
    b_in: float,
    h_in: float,
    fc_ksi: float,
    fy_ksi: float,
    main_bars: str,
    stirrup_bar: str,
    stirrup_spacing_in: float,
    span_ft: float,
    dead_load_klf: float,
    live_load_klf: float,
    cover_in: float = 1.5,
    stirrup_legs: int = 2,
    member_id: str = "C1",
    project: str | None = None,
) -> str:
    """ACI 318-19 full check: flexure, shear, deflection, development, and curtailment.

    Loads are UNFACTORED service loads; ASCE 7-22 LRFD governs the factored demand.
    Support condition: simply supported.

    Args:
        b_in: beam width (in).
        h_in: total beam height (in).
        fc_ksi: concrete compressive strength (ksi), e.g. 4.0 for 4000 psi.
        fy_ksi: rebar yield strength (ksi), e.g. 60.0.
        main_bars: main tension reinforcement, e.g. '4#8' or '3#10'.
        stirrup_bar: stirrup bar designation, e.g. '#3' or '#4'.
        stirrup_spacing_in: stirrup spacing (in).
        span_ft: clear span (ft).
        dead_load_klf: unfactored dead load (kip/ft), excluding beam self-weight.
        live_load_klf: unfactored live load (kip/ft).
        cover_in: clear cover to stirrup face (in); default 1.5.
        stirrup_legs: number of stirrup legs; default 2.
        member_id: label for the archived calc.
        project: project folder name; defaults to 'DEFAULT'.
    """
    try:
        _validate_concrete_inputs(b_in, h_in, fc_ksi, fy_ksi, span_ft)
        _require(stirrup_spacing_in > 0,
                 f"stirrup_spacing_in must be > 0, got {stirrup_spacing_in}.")
        _require(stirrup_legs in (2, 4, 6),
                 f"stirrup_legs must be 2, 4, or 6, got {stirrup_legs}.")

        n_main, bar_desig = _rebar.parse_bar_string(main_bars)
        main_bar_props   = _rebar.get_bar(bar_desig)
        stirrup_props    = _rebar.get_bar(stirrup_bar.strip())

        d_in   = _effective_depth(h_in, cover_in, stirrup_bar.strip(), bar_desig)
        _require(d_in > 0, f"Effective depth d = {d_in:.2f} in ≤ 0 — section is too shallow.")
        As_in2 = n_main * main_bar_props["Ab_in2"]
        Av_in2 = stirrup_legs * stirrup_props["Ab_in2"]

        # Factored loads — ASCE 7-22 LRFD
        loads = asce7._norm({"D": dead_load_klf, "L": live_load_klf})
        _require(any(loads.values()), "No applied loads — provide dead_load_klf and/or live_load_klf.")
        wu_klf, combo = asce7.factored_envelope(loads)
        _require(wu_klf > 0, "Governing factored load is zero.")

        # Demands (simply supported UDL)
        Mu_kip_ft = wu_klf * span_ft**2 / 8.0
        Vu_kips   = wu_klf * span_ft / 2.0

        # ACI 318-19 checks
        flex  = aci318.flexure_check(b_in, d_in, As_in2, fc_ksi, fy_ksi, Mu_kip_ft)
        shear = aci318.shear_check(
            b_in, d_in, As_in2, h_in, fc_ksi, fy_ksi,
            Av_in2, stirrup_spacing_in, _FYT_DEFAULT_KSI, Vu_kips,
        )
        defl  = aci318.deflection_check(
            b_in, h_in, d_in, As_in2, fc_ksi,
            dead_load_klf, live_load_klf, span_ft,
        )
        dev   = aci318.development_length_straight(bar_desig, fc_ksi, fy_ksi, cover_in=cover_in)
        curt  = aci318.bar_curtailment(
            b_in, d_in, n_main, n_main // 2, bar_desig,
            fc_ksi, fy_ksi, wu_klf, span_ft,
        ) if n_main >= 2 else None

    except NotImplementedError as e:
        return f"ESCALATE TO SENIOR ENGINEER: {e}"
    except (ValueError, InputError) as e:
        return f"INPUT ERROR: {e}"

    # Build and archive the full calc
    chk = aci318.ConcreteBeamCheck(
        b_in=b_in, h_in=h_in, d_in=d_in,
        n_bars=n_main, bar_designation=bar_desig, As_in2=As_in2,
        fc_ksi=fc_ksi, fy_ksi=fy_ksi,
        Mu_kip_ft=Mu_kip_ft, Vu_kips=Vu_kips,
        wu_klf=wu_klf, governing_combo=combo.name,
        flexure=flex, shear=shear, deflection=defl, dev_straight=dev,
    )
    project_dir = proj.resolve_project_dir(project)
    header = _concrete_calc_header(
        f"{n_main}{bar_desig}", b_in, h_in, fc_ksi, fy_ksi, span_ft, project_dir.name
    )
    md = header + "\n\n" + chk.summary()
    if curt:
        md += "\n\n## Curtailment (cut ~half bars near supports)\n" + "\n".join(curt.summary_lines())
    calc_path = proj.write_calc(project_dir, member_id, md, csv=None)
    proj.append_member(project_dir, member_id, {
        "section": f"{b_in:.0f}x{h_in:.0f}",
        "bars": f"{n_main}{bar_desig}", "span_ft": span_ft,
        "wu_klf": round(wu_klf, 4),
        "Mu_kip_ft": round(Mu_kip_ft, 1), "Vu_kips": round(Vu_kips, 1),
        "DCR_flexure": round(flex.DCR, 3), "DCR_shear": round(shear.DCR, 3),
        "governing_combo": combo.name, "calc_file": calc_path.name,
    }, bucket="concrete_beams")

    max_dcr = max(flex.DCR, shear.DCR)
    tag = "OK" if max_dcr <= 1.0 else "*** OVERSTRESSED ***"
    defl_flag = ("PASS" if defl.delta_i_L_in <= defl.limit_L_360_in else "EXCEEDS L/360")
    return (
        f"Concrete beam {member_id} = {n_main}{bar_desig}  {tag}\n"
        f"  Governing combo  : {combo.name} = {wu_klf:.3f} klf\n"
        f"  Demands          : Mu = {Mu_kip_ft:.1f} kip-ft,  Vu = {Vu_kips:.1f} kips\n"
        f"  Flexure          : φMn = {flex.phi_Mn_kip_ft:.1f} kip-ft,  "
        f"DCR = {flex.DCR:.3f}  ({flex.code_ref})\n"
        f"  Shear            : φVn = {shear.phi_Vn_kips:.1f} kips,  "
        f"DCR = {shear.DCR:.3f}  ({shear.code_ref})\n"
        f"  Deflection       : δi,L = {defl.delta_i_L_in:.4f} in,  "
        f"δtot = {defl.delta_total_in:.4f} in  [{defl_flag}]\n"
        f"  Dev. length (str): {dev.ld_in:.2f} in  ({dev.code_ref})\n"
        + (f"  Curtailment      : cut {n_main // 2} bars at ≥ {curt.x_cutoff_ft:.2f} ft from support\n"
           if curt else "")
        + f"  Full calc        : {calc_path}\n"
        f"  (ASCE 7 §4.7 LL reduction not applied; stirrup fyt = 60 ksi assumed.)"
    )


@mcp.tool()
def design_concrete_beam(
    b_in: float,
    h_in: float,
    fc_ksi: float,
    fy_ksi: float,
    n_bars: int,
    span_ft: float,
    dead_load_klf: float,
    live_load_klf: float,
    cover_in: float = 1.5,
    stirrup_bar: str = "#3",
    stirrup_legs: int = 2,
    member_id: str = "C1",
    project: str | None = None,
) -> str:
    """Size main tension reinforcement and suggest stirrups for a rectangular concrete beam.

    Finds the lightest (smallest) standard bar such that n_bars of that size satisfy
    ACI 318-19 flexure, shear, and deflection checks.  Stirrups are designed for the
    critical section shear (Vu at d from face of support, conservatively taken as Vu,max).

    Args:
        b_in: beam width (in).
        h_in: total beam height (in).
        fc_ksi: concrete compressive strength (ksi).
        fy_ksi: rebar yield strength (ksi).
        n_bars: number of main tension bars.
        span_ft: clear span (ft).
        dead_load_klf: unfactored dead load (kip/ft).
        live_load_klf: unfactored live load (kip/ft).
        cover_in: clear cover to stirrup face (in).
        stirrup_bar: stirrup bar designation for shear design, e.g. '#3'.
        stirrup_legs: number of stirrup legs; default 2.
        member_id: label for archived calc.
        project: project folder name.
    """
    try:
        _validate_concrete_inputs(b_in, h_in, fc_ksi, fy_ksi, span_ft)
        _require(n_bars >= 2, f"n_bars must be ≥ 2, got {n_bars}.")
        stirrup_props = _rebar.get_bar(stirrup_bar.strip())

        # Use #4 as proxy for d estimate (refined after bar selection)
        d_est = h_in - cover_in - stirrup_props["db_in"] - 0.500 / 2.0

        loads  = asce7._norm({"D": dead_load_klf, "L": live_load_klf})
        _require(any(loads.values()), "No applied loads provided.")
        wu_klf, combo = asce7.factored_envelope(loads)
        _require(wu_klf > 0, "Governing factored load is zero.")

        Mu_kip_ft = wu_klf * span_ft**2 / 8.0
        Vu_kips   = wu_klf * span_ft / 2.0

        # Required As from flexure (quadratic)
        As_req = aci318.required_As(b_in, d_est, fc_ksi, fy_ksi, Mu_kip_ft)
        As_min = aci318.min_As(b_in, d_est, fc_ksi, fy_ksi)
        As_req = max(As_req, As_min)

        Ab_req = As_req / n_bars   # required area per bar

        # Find smallest standard bar that meets the requirement
        chosen_desig = None
        for desig in _rebar.DESIGNATIONS:
            bar = _rebar.get_bar(desig)
            if bar["Ab_in2"] >= Ab_req:
                chosen_desig = desig
                break
        if chosen_desig is None:
            return (
                f"INPUT ERROR: No standard bar size provides As ≥ {As_req:.3f} in² "
                f"with {n_bars} bars. Increase n_bars or increase beam depth."
            )

        # Refine d with actual bar size
        main_props = _rebar.get_bar(chosen_desig)
        d_in  = h_in - cover_in - stirrup_props["db_in"] - main_props["db_in"] / 2.0
        _require(d_in > 0,
                 f"Effective depth d = {d_in:.2f} in ≤ 0 with {chosen_desig} — section too shallow.")
        As_in2 = n_bars * main_props["Ab_in2"]

        # Design stirrups — find spacing for Vu at critical section
        Av_in2_pair = stirrup_legs * stirrup_props["Ab_in2"]

        # Vc at critical section (conservative: use Vu at support)
        shear_trial = aci318.shear_check(
            b_in, d_in, As_in2, h_in, fc_ksi, fy_ksi,
            Av_in2_pair, 1.0, _FYT_DEFAULT_KSI, Vu_kips,   # s=1 placeholder
        )
        Vc = shear_trial.Vc_kips
        Vs_req = max(Vu_kips / _PHI_SHEAR - Vc, 0.0)   # required Vs

        if Vs_req > 0:
            s_req = Av_in2_pair * _FYT_DEFAULT_KSI * d_in / Vs_req
        else:
            s_req = 24.0   # Vc alone is enough; use practical max

        # Apply spacing limits §9.7.6.2.2 and §9.6.3.4 (min reinf.)
        Av_min_per_s = shear_trial.Av_min_per_s
        s_min_Av = Av_in2_pair / Av_min_per_s   # spacing to just meet Av,min
        fc_psi = fc_ksi * 1000.0
        Vs_design = Av_in2_pair * _FYT_DEFAULT_KSI * d_in / s_req if Vs_req > 0 else 0.0
        Vs_limit = 4.0 * (fc_psi**0.5) * b_in * d_in / 1000.0
        s_max_code = min(d_in / 2.0, 24.0) if Vs_design <= Vs_limit else min(d_in / 4.0, 12.0)

        s_design = min(s_req, s_min_Av, s_max_code)
        s_design = max(3.0, round(s_design / 0.5) * 0.5)   # round down to nearest 0.5 in, min 3 in

        # Final check with designed stirrups
        flex  = aci318.flexure_check(b_in, d_in, As_in2, fc_ksi, fy_ksi, Mu_kip_ft)
        shear = aci318.shear_check(
            b_in, d_in, As_in2, h_in, fc_ksi, fy_ksi,
            Av_in2_pair, s_design, _FYT_DEFAULT_KSI, Vu_kips,
        )
        defl  = aci318.deflection_check(
            b_in, h_in, d_in, As_in2, fc_ksi,
            dead_load_klf, live_load_klf, span_ft,
        )
        dev   = aci318.development_length_straight(chosen_desig, fc_ksi, fy_ksi, cover_in=cover_in)

    except NotImplementedError as e:
        return f"ESCALATE TO SENIOR ENGINEER: {e}"
    except (ValueError, InputError) as e:
        return f"INPUT ERROR: {e}"

    # Archive
    project_dir = proj.resolve_project_dir(project)
    chk = aci318.ConcreteBeamCheck(
        b_in=b_in, h_in=h_in, d_in=d_in,
        n_bars=n_bars, bar_designation=chosen_desig, As_in2=As_in2,
        fc_ksi=fc_ksi, fy_ksi=fy_ksi,
        Mu_kip_ft=Mu_kip_ft, Vu_kips=Vu_kips,
        wu_klf=wu_klf, governing_combo=combo.name,
        flexure=flex, shear=shear, deflection=defl, dev_straight=dev,
    )
    header = _concrete_calc_header(
        f"{n_bars}{chosen_desig} [designed]",
        b_in, h_in, fc_ksi, fy_ksi, span_ft, project_dir.name,
    )
    md = header + "\n\n" + chk.summary()
    calc_path = proj.write_calc(project_dir, member_id, md, csv=None)
    proj.append_member(project_dir, member_id, {
        "section": f"{b_in:.0f}x{h_in:.0f}",
        "bars": f"{n_bars}{chosen_desig}",
        "stirrups": f"{stirrup_legs}L{stirrup_bar}@{s_design:.1f}in",
        "span_ft": span_ft,
        "wu_klf": round(wu_klf, 4),
        "Mu_kip_ft": round(Mu_kip_ft, 1), "Vu_kips": round(Vu_kips, 1),
        "DCR_flexure": round(flex.DCR, 3), "DCR_shear": round(shear.DCR, 3),
        "governing_combo": combo.name, "calc_file": calc_path.name,
    }, bucket="concrete_beams")

    max_dcr = max(flex.DCR, shear.DCR)
    tag = "OK" if max_dcr <= 1.0 else "*** OVERSTRESSED ***"
    return (
        f"Concrete beam {member_id} [designed]  {tag}\n"
        f"  Main steel      : {n_bars}{chosen_desig}  (As = {As_in2:.3f} in²,  "
        f"d = {d_in:.3f} in)\n"
        f"  Stirrups        : {stirrup_legs}L {stirrup_bar} @ {s_design:.1f} in\n"
        f"  Governing combo : {combo.name} = {wu_klf:.3f} klf\n"
        f"  Demands         : Mu = {Mu_kip_ft:.1f} kip-ft,  Vu = {Vu_kips:.1f} kips\n"
        f"  Flexure         : φMn = {flex.phi_Mn_kip_ft:.1f} kip-ft,  DCR = {flex.DCR:.3f}\n"
        f"  Shear           : φVn = {shear.phi_Vn_kips:.1f} kips,  DCR = {shear.DCR:.3f}\n"
        f"  Dev. length     : ld = {dev.ld_in:.2f} in  ({dev.code_ref})\n"
        f"  Deflection      : δi,L = {defl.delta_i_L_in:.4f} in,  "
        f"δtot = {defl.delta_total_in:.4f} in\n"
        f"  Full calc       : {calc_path}\n"
        f"  (ASCE 7 §4.7 LL reduction not applied; stirrup fyt = 60 ksi assumed.)"
    )


@mcp.tool()
def concrete_development(
    bar_designation: str,
    fc_ksi: float,
    fy_ksi: float,
    end_type: str = "straight",
    bar_location: str = "other",
    cover_in: float = 1.5,
    clear_spacing_in: float | None = None,
    coated: bool = False,
    confining_reinf: bool = False,
    cover_ok: bool = True,
    enclosed_in_ties: bool = False,
    generous_bearing: bool = False,
) -> str:
    """ACI 318-19 development length for a single bar — straight, hook, or T-head.

    Args:
        bar_designation: e.g. '#8', '#10', '5'.
        fc_ksi: f'c (ksi).
        fy_ksi: fy (ksi).
        end_type: 'straight' (§25.4.2), 'hook_90' or 'hook_180' (§25.4.3), 'thead' (§25.4.4).
        bar_location: 'top' (>12 in fresh concrete below) or 'other' — for straight ld ψt.
        cover_in: clear cover to bar face (in).
        clear_spacing_in: clear spacing to adjacent parallel bar (in); None = conservative.
        coated: True for epoxy-coated bar.
        confining_reinf: hook only — ties/stirrups ≤ 3db around hook → ψr = 0.8.
        cover_ok: hook only — side cover ≥ 2.5 in, end cover ≥ 2 in → ψo = 0.8.
        enclosed_in_ties: hook only — hook enclosed within 0-3db ties → ψc = 0.8.
        generous_bearing: T-head only — Abrg ≥ 4Ab → ψp = 0.8.
    """
    try:
        et = end_type.lower().strip()
        if et == "straight":
            result = aci318.development_length_straight(
                bar_designation, fc_ksi, fy_ksi,
                bar_location=bar_location, cover_in=cover_in,
                clear_spacing_in=clear_spacing_in, coated=coated,
            )
        elif et in ("hook_90", "hook_180"):
            angle = 90 if et == "hook_90" else 180
            result = aci318.development_length_hook(
                bar_designation, fc_ksi, fy_ksi,
                hook_angle=angle, coated=coated,
                confining_reinf=confining_reinf,
                cover_ok=cover_ok, enclosed_in_ties=enclosed_in_ties,
            )
        elif et == "thead":
            result = aci318.development_length_thead(
                bar_designation, fc_ksi, fy_ksi,
                coated=coated, generous_bearing=generous_bearing,
            )
        else:
            return (
                f"INPUT ERROR: end_type='{end_type}' not recognized. "
                "Use 'straight', 'hook_90', 'hook_180', or 'thead'."
            )
    except NotImplementedError as e:
        return f"ESCALATE TO SENIOR ENGINEER: {e}"
    except (ValueError, InputError) as e:
        return f"INPUT ERROR: {e}"

    return "\n".join(result.summary_lines())


@mcp.tool()
def concrete_splice(
    bar_designation: str,
    fc_ksi: float,
    fy_ksi: float,
    splice_class: str = "B",
    bar_location: str = "other",
    cover_in: float = 1.5,
    clear_spacing_in: float | None = None,
    coated: bool = False,
) -> str:
    """ACI 318-19 §25.5.2 tension lap splice length.

    Class A (1.0·ld): As,prov/As,req ≥ 2.0 AND ≤ 50% of bars spliced at section.
    Class B (1.3·ld): all other cases (conservative default).

    Args:
        bar_designation: e.g. '#8'.
        fc_ksi: f'c (ksi).
        fy_ksi: fy (ksi).
        splice_class: 'A' or 'B'.
        bar_location: 'top' or 'other'.
        cover_in: clear cover to bar face (in).
        clear_spacing_in: clear spacing to adjacent bar (in); None = conservative.
        coated: True for epoxy-coated.
    """
    try:
        sc = splice_class.upper().strip()
        if sc not in ("A", "B"):
            return f"INPUT ERROR: splice_class must be 'A' or 'B', got '{splice_class}'."
        ls, dev = aci318.splice_length(
            bar_designation, fc_ksi, fy_ksi,
            splice_class=sc, bar_location=bar_location,
            cover_in=cover_in, clear_spacing_in=clear_spacing_in, coated=coated,
        )
    except (ValueError, InputError) as e:
        return f"INPUT ERROR: {e}"

    factor = 1.0 if sc == "A" else 1.3
    return "\n".join([
        f"ACI 318-19 §25.5.2 Tension Lap Splice — Class {sc}:",
        f"  Bar                : {bar_designation}",
        f"  Base ld            = {dev.ld_in:.2f} in  ({dev.code_ref})",
        f"  Splice factor      = {factor:.1f}",
        f"  Splice length ls   = {ls:.2f} in  ({ls / 12:.2f} ft)",
        f"  Minimum            : 12 in per §25.5.2.1(b)",
        f"  Note: Class A applies only when As,prov/As,req ≥ 2.0 AND ≤ 50% of bars "
        f"spliced at the section — §25.5.2.1.",
    ])


if __name__ == "__main__":
    log.info("starting junior-se MCP server (stdio)")
    mcp.run()
