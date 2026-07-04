"""
SAP2000 bridge tools — thin MCP wrappers over Project_005's REST API.

Project_005 (SAP2000 AI Model Builder) exposes the live SAP2000 COM connection
behind FastAPI at http://127.0.0.1:8000. Its modular operations
(`services/sap2000/operations/`) were designed with a JSON-in/JSON-out contract
precisely so each one converts 1:1 into an agent tool — this module is that
conversion. No SAP2000/COM logic lives here: every function is an HTTP call.

UNITS: imperial only (engineer directive 2026-07-04). Build models with
`units: "kip_ft"` (kips, ft, ksf, kip/ft^3) or `"kip_in"` (kips, in, ksi).
All operation values are in the CURRENT MODEL UNITS — for a kip_ft model,
forces come back in kips, moments in kip-ft, lengths/deflections in ft.

Prerequisite: the P005 server must be running::

    cd D:\\AI_TEST\\Agent_Developer\\Project_005_SAP2000api_v3
    C:\\Python314\\python.exe -m uvicorn src.backend.main:app --port 8000

Override the base URL with the SAP2000_API_URL environment variable.
"""
from __future__ import annotations

import json
import os

import httpx

_BASE_URL = os.environ.get("SAP2000_API_URL", "http://127.0.0.1:8000")
_PREFIX = "/api/sap2000"

# SAP2000 launch / model build / analysis runs are slow COM operations.
_TIMEOUT_S = 600.0

# Swappable for tests (httpx.MockTransport); None = real network.
_transport: httpx.BaseTransport | None = None

# Cap on result rows echoed back into the conversation (full data stays in
# SAP2000; re-query with filters for more).
_MAX_ROWS = 300


class SAPBridgeError(Exception):
    """P005 API unreachable or returned an error; message is user-facing."""


def _request(method: str, path: str, *, body: dict | None = None,
             params: dict | None = None) -> dict:
    """One HTTP round-trip to the P005 API; returns the parsed JSON payload."""
    try:
        with httpx.Client(base_url=_BASE_URL, timeout=_TIMEOUT_S,
                          transport=_transport) as client:
            r = client.request(method, path, json=body, params=params)
    except httpx.ConnectError as exc:
        raise SAPBridgeError(
            f"Project_005 SAP2000 API is not reachable at {_BASE_URL} ({exc}). "
            "Start it with:\n"
            "  cd D:\\AI_TEST\\Agent_Developer\\Project_005_SAP2000api_v3\n"
            "  C:\\Python314\\python.exe -m uvicorn src.backend.main:app --port 8000"
        ) from exc
    except httpx.TimeoutException as exc:
        raise SAPBridgeError(
            f"Request to {path} timed out after {_TIMEOUT_S:.0f}s — SAP2000 may "
            "be stuck or the model is very large."
        ) from exc

    if r.status_code >= 400:
        try:
            detail = r.json().get("detail", r.text)
        except ValueError:
            detail = r.text
        raise SAPBridgeError(f"P005 API error (HTTP {r.status_code}): {detail}")
    return r.json()


def _op(name: str, **params) -> dict:
    """Dispatch a modular operation: POST /api/sap2000/op/{name}."""
    body = {k: v for k, v in params.items() if v is not None}
    return _request("POST", f"{_PREFIX}/op/{name}", body=body)


def _dump(payload: dict) -> str:
    """JSON-format a payload, truncating any list of row-dicts at _MAX_ROWS."""
    trimmed = dict(payload)
    notes = []
    for key, val in payload.items():
        if isinstance(val, list) and len(val) > _MAX_ROWS:
            trimmed[key] = val[:_MAX_ROWS]
            notes.append(f"'{key}' truncated to first {_MAX_ROWS} of {len(val)} "
                         "rows — re-query with case/joint/frame filters for the rest")
    text = json.dumps(trimmed, indent=1)
    if notes:
        text += "\n\nNOTE: " + "; ".join(notes)
    return text


# ---------------------------------------------------------------------------
# Connection / model lifecycle
# ---------------------------------------------------------------------------

def sap_status() -> str:
    """Check whether the Project_005 API server is up and SAP2000 is connected.

    Call this first when starting SAP2000 work. Reports both layers: the REST
    server (must be running) and the SAP2000 COM connection (sap_connect).
    """
    try:
        payload = _request("GET", f"{_PREFIX}/status")
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    connected = payload.get("connected", False)
    return (f"P005 API server: UP at {_BASE_URL}\n"
            f"SAP2000 COM connection: {'CONNECTED' if connected else 'NOT CONNECTED'}"
            + ("" if connected else "  (call sap_connect to attach/launch)"))


def sap_connect(visible: bool = True) -> str:
    """Connect to a running SAP2000 instance, or launch one.

    Args:
        visible: show the SAP2000 window (True recommended so the engineer
            can watch the model).
    """
    try:
        payload = _request("POST", f"{_PREFIX}/connect", body={"visible": visible})
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return f"SAP2000: {payload.get('status', 'unknown')}"


def sap_close() -> str:
    """Close SAP2000 cleanly (ApplicationExit, unsaved changes discarded).

    Ask the engineer before closing if a model may have unsaved work.
    """
    try:
        payload = _request("POST", f"{_PREFIX}/close")
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return f"SAP2000: {payload.get('status', 'unknown')}"


def sap_build_model(model: dict, save_path: str = "",
                    run_analysis: bool = False) -> str:
    """Build a SAP2000 model from a StructuralModel JSON dict (P005 schema).

    IMPERIAL ONLY: set model["units"] = "kip_ft" (kips, ft, ksf, kip/ft^3) or
    "kip_in" (kips, in, ksi) and give every value in those consistent units —
    convert psf/pcf/psi before emitting JSON (e.g. 50 psf -> 0.050 ksf).

    The build report maps requested member names to SAP2000's ACTUAL names
    ("requested -> actual") — use the actual names for sap_frame_forces.

    Args:
        model: StructuralModel dict (units, geometry, girders/beams/slab,
            loads, supports ...) per the P005 /api/sap2000/build-from-json schema.
        save_path: absolute .sdb path to save after building (required if
            run_analysis=True — SAP2000 cannot run an unsaved file).
        run_analysis: also run the analysis after building.
    """
    try:
        payload = _request("POST", f"{_PREFIX}/build-from-json", body=model,
                           params={"save_path": save_path,
                                   "run_analysis": run_analysis})
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return _dump(payload)


# ---------------------------------------------------------------------------
# Modular operations (1:1 with P005 services/sap2000/operations)
# ---------------------------------------------------------------------------

def sap_list_operations() -> str:
    """List the modular SAP2000 operations available on the P005 server."""
    try:
        payload = _request("GET", f"{_PREFIX}/op")
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    lines = [f"  {name}: {doc}" for name, doc in sorted(payload.items())]
    return "P005 modular operations:\n" + "\n".join(lines)


def sap_run_analysis(save_path: str = "") -> str:
    """Save (optionally) and run the SAP2000 analysis; report per-case status.

    Raises a clear error if no case actually finished (guards against the
    orphaned-instance silent failure).

    Args:
        save_path: absolute .sdb path to save to before running; "" keeps the
            current file (the model must have been saved at least once).
    """
    try:
        payload = _op("run_analysis", save_path=save_path)
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return _dump(payload)


def sap_find_joints(coords: list[list[float]], tol: float = 0.01) -> str:
    """Find the SAP2000 joint closest to each [x, y, z] coordinate.

    SAP2000 renames joints on merge — use this to get ACTUAL joint names for
    displacement/reaction queries. Coordinates in current model units (ft for
    a kip_ft model).

    Args:
        coords: list of [x, y, z] coordinates ([x, y] implies z=0).
        tol: match tolerance in model length units; `matched` is False beyond it.
    """
    try:
        payload = _op("find_joints", coords=coords, tol=tol)
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return _dump(payload)


def sap_add_columns(positions: list[list[float]], height: float,
                    top_z: float = 0.0, section_shape: str = "W14X90",
                    section_name: str = "COLSEC", material: str = "steel",
                    fc_ksi: float = 4.0, unit_weight: float = 0.150,
                    unit_system: str = "kip_ft", rect_depth: float = 1.5,
                    rect_width: float = 1.5, fix_base: bool = True) -> str:
    """Add vertical columns below (x, y) plan positions in the current model.

    Tops auto-merge with existing deck joints; bases are restrained. All
    geometry in current model units (ft for kip_ft). IMPERIAL defaults.

    Args:
        positions: [[x, y], ...] plan positions (model length units).
        height: column height (model length units), tops at top_z.
        top_z: elevation of column tops.
        section_shape: AISC W label (e.g. "W14X90") or "rect".
        section_name: SAP2000 property name to create.
        material: "steel" (A992) or "concrete".
        fc_ksi: concrete f'c in KSI (converted per unit_system internally).
        unit_weight: concrete unit weight in model units (kip/ft^3 for kip_ft;
            0.150 kcf = 150 pcf normal weight).
        unit_system: model unit system — keep "kip_ft" (imperial policy).
        rect_depth: rectangle depth (model length units) when shape is "rect".
        rect_width: rectangle width (model length units).
        fix_base: True = fixed base, False = pinned.
    """
    try:
        payload = _op("add_columns", positions=positions, height=height,
                      top_z=top_z, section_shape=section_shape,
                      section_name=section_name, material=material,
                      fc=fc_ksi, unit_weight=unit_weight,
                      unit_system=unit_system, rect_depth=rect_depth,
                      rect_width=rect_width, fix_base=fix_base)
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return _dump(payload)


def sap_define_load_combos(case_map: dict, prefix: str = "",
                           envelope: bool = True, replace: bool = True) -> str:
    """Create ASCE 7-22 LRFD strength combos in the current SAP2000 model.

    "or" alternatives expand into variants (LC2a/LC2b...); combos whose
    distinguishing load type is unmapped are skipped with reasons. Also
    creates an "<prefix>LRFD-ENV" envelope unless envelope=False.

    Assumptions carried in the payload: no ASCE 7 §4.7 live-load reduction;
    companion L at 1.0 (0.5L exception not taken); W/E direction reversal is
    the engineer's job (map directional cases or call once per direction with
    a prefix).

    Args:
        case_map: ASCE load type -> model load case name(s), e.g.
            {"D": "DEAD", "L": "LL", "S": "SNOW", "W": ["WX"]}. "D" required.
        prefix: prepended to combo names (e.g. "WX-").
        envelope: also create the LRFD-ENV envelope combo.
        replace: delete same-named existing combos first (idempotent).
    """
    try:
        payload = _op("define_load_combos", case_map=case_map, prefix=prefix,
                      envelope=envelope, replace=replace)
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return _dump(payload)


def sap_list_load_cases() -> str:
    """List load cases in the current model with each case's last run status."""
    try:
        payload = _op("list_load_cases")
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return _dump(payload)


def sap_joint_reactions(cases: list[str] | None = None,
                        joints: list[str] | None = None) -> str:
    """Support reactions (F1,F2,F3,M1,M2,M3) per joint per case/combo step.

    Current model units (kips / kip-ft for a kip_ft model). Without a joints
    list, only restrained (support) joints are returned. Combos must be named
    explicitly in `cases`.

    Args:
        cases: case/combo names to report; None = all load CASES.
        joints: ACTUAL SAP2000 joint names (see sap_find_joints); None = all
            restrained joints.
    """
    try:
        payload = _op("joint_reactions", cases=cases, joints=joints)
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return _dump(payload)


def sap_joint_displacements(cases: list[str] | None = None,
                            joints: list[str] | None = None) -> str:
    """Joint displacements/rotations (U1,U2,U3,R1,R2,R3) per case/combo step.

    Current model units (ft / rad for a kip_ft model — multiply U by 12 for
    inches when reporting).

    Args:
        cases: case/combo names; None = all load CASES.
        joints: ACTUAL joint names; None = every joint (can be large — prefer
            targeted joints via sap_find_joints).
    """
    try:
        payload = _op("joint_displacements", cases=cases, joints=joints)
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return _dump(payload)


def sap_frame_forces(cases: list[str] | None = None,
                     frames: list[str] | None = None) -> str:
    """Frame internal forces (P,V2,V3,T,M2,M3) at output stations per case.

    Current model units (kips / kip-ft for a kip_ft model). Frame names are
    SAP2000's ACTUAL names — use the build report's "requested -> actual"
    mapping. Prefer an explicit frames list; ALL on a big model is huge.

    Args:
        cases: case/combo names; None = all load CASES.
        frames: ACTUAL frame names; None = all frames.
    """
    try:
        payload = _op("frame_forces", cases=cases, frames=frames)
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return _dump(payload)


def sap_base_reactions(cases: list[str] | None = None) -> str:
    """Global base reaction totals (FX,FY,FZ,MX,MY,MZ) per case/combo step.

    The fastest statics sanity check: total FZ must equal total applied
    gravity load in kips (kip_ft model).

    Args:
        cases: case/combo names; None = all load CASES.
    """
    try:
        payload = _op("base_reactions", cases=cases)
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return _dump(payload)


def sap_modal_periods() -> str:
    """Modal periods/frequencies from the last run MODAL case."""
    try:
        payload = _op("modal_periods")
    except SAPBridgeError as e:
        return f"SAP2000 BRIDGE ERROR: {e}"
    return _dump(payload)


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

SAP_TOOLS = [
    sap_status, sap_connect, sap_close, sap_build_model,
    sap_list_operations, sap_run_analysis, sap_find_joints, sap_add_columns,
    sap_define_load_combos, sap_list_load_cases, sap_joint_reactions,
    sap_joint_displacements, sap_frame_forces, sap_base_reactions,
    sap_modal_periods,
]


def register(mcp) -> None:
    """Register every SAP2000 bridge function as a FastMCP tool."""
    for fn in SAP_TOOLS:
        mcp.add_tool(fn)
