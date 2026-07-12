"""
Unit tests for the SAP2000 bridge tools (src/agent/sap2000_tools.py).

These use httpx.MockTransport — no P005 server, no SAP2000, no network.
They verify the HTTP contract (paths, bodies, params), error surfacing, and
FastMCP registration. Live end-to-end verification against a running P005
server is a separate integration script (scripts/test_sap_mcp_tools.py) and
is NOT covered here.
"""
from __future__ import annotations

import asyncio
import json

import httpx
import pytest

from src.agent import sap2000_tools as st


@pytest.fixture
def capture(monkeypatch):
    """Install a MockTransport; returns a dict capturing the last request and
    letting each test set the canned response."""
    box = {"request": None, "response": httpx.Response(200, json={"status": "ok"})}

    def handler(request: httpx.Request) -> httpx.Response:
        box["request"] = request
        return box["response"]

    monkeypatch.setattr(st, "_transport", httpx.MockTransport(handler))
    return box


# ---------------------------------------------------------------------------
# HTTP contract
# ---------------------------------------------------------------------------

def test_status_connected(capture):
    capture["response"] = httpx.Response(200, json={"connected": True})
    out = st.sap_status()
    assert "CONNECTED" in out and "UP" in out
    assert capture["request"].url.path == "/api/sap2000/status"
    assert capture["request"].method == "GET"


def test_status_not_connected(capture):
    capture["response"] = httpx.Response(200, json={"connected": False})
    out = st.sap_status()
    assert "NOT CONNECTED" in out and "sap_connect" in out


def test_connect_posts_visible_flag(capture):
    capture["response"] = httpx.Response(200, json={"status": "connected"})
    out = st.sap_connect(visible=False)
    assert out == "SAP2000: connected"
    assert capture["request"].url.path == "/api/sap2000/connect"
    assert json.loads(capture["request"].content) == {"visible": False}


def test_build_model_sends_body_and_query_params(capture):
    capture["response"] = httpx.Response(200, json={"status": "ok", "report": {}})
    model = {"units": "kip_ft", "geometry": {}}
    st.sap_build_model(model, save_path="C:/tmp/x.sdb", run_analysis=True)
    req = capture["request"]
    assert req.url.path == "/api/sap2000/build-from-json"
    assert json.loads(req.content) == model
    assert req.url.params["save_path"] == "C:/tmp/x.sdb"
    assert req.url.params["run_analysis"] == "true"


def test_op_dispatch_path_and_none_params_dropped(capture):
    capture["response"] = httpx.Response(200, json={"status": "ok", "reactions": []})
    st.sap_joint_reactions(cases=["DEAD"], joints=None)
    req = capture["request"]
    assert req.url.path == "/api/sap2000/op/joint_reactions"
    assert json.loads(req.content) == {"cases": ["DEAD"]}  # joints=None dropped


def test_add_columns_maps_fc_ksi_and_imperial_defaults(capture):
    capture["response"] = httpx.Response(200, json={"status": "ok", "columns": []})
    st.sap_add_columns([[0.0, 0.0]], height=12.0)
    body = json.loads(capture["request"].content)
    assert body["fc"] == 4.0                 # wrapper arg fc_ksi -> op param fc
    assert body["unit_system"] == "kip_ft"   # imperial default
    assert body["unit_weight"] == 0.150      # kcf


def test_define_load_combos_body(capture):
    capture["response"] = httpx.Response(200, json={"status": "ok", "created": []})
    st.sap_define_load_combos({"D": "DEAD", "L": "LL"}, prefix="X-")
    body = json.loads(capture["request"].content)
    assert body == {"case_map": {"D": "DEAD", "L": "LL"}, "prefix": "X-",
                    "envelope": True, "replace": True}


# ---------------------------------------------------------------------------
# Error surfacing
# ---------------------------------------------------------------------------

def test_http_error_detail_surfaced(capture):
    capture["response"] = httpx.Response(
        500, json={"detail": "Operation 'run_analysis' failed: no case finished"})
    out = st.sap_run_analysis()
    assert out.startswith("SAP2000 BRIDGE ERROR:")
    assert "no case finished" in out and "HTTP 500" in out


def test_unknown_op_404_detail(capture):
    capture["response"] = httpx.Response(404, json={"detail": "Unknown operation 'nope'"})
    out = st.sap_modal_periods()
    assert "Unknown operation" in out


def test_server_unreachable_message(monkeypatch):
    def handler(request):
        raise httpx.ConnectError("connection refused", request=request)
    monkeypatch.setattr(st, "_transport", httpx.MockTransport(handler))
    out = st.sap_status()
    assert "not reachable" in out
    assert "uvicorn" in out  # tells the user how to start the P005 server


def test_non_json_error_body(capture):
    capture["response"] = httpx.Response(502, text="Bad Gateway")
    out = st.sap_list_load_cases()
    assert "HTTP 502" in out and "Bad Gateway" in out


# ---------------------------------------------------------------------------
# Output formatting
# ---------------------------------------------------------------------------

def test_dump_truncates_long_row_lists():
    rows = [{"i": i} for i in range(st._MAX_ROWS + 50)]
    text = st._dump({"status": "ok", "frame_forces": rows})
    assert f"first {st._MAX_ROWS} of {st._MAX_ROWS + 50}" in text
    payload = json.loads(text.split("\n\nNOTE:")[0])
    assert len(payload["frame_forces"]) == st._MAX_ROWS


def test_dump_short_lists_untouched():
    text = st._dump({"status": "ok", "cases": [{"name": "DEAD"}]})
    assert "NOTE" not in text
    assert json.loads(text)["cases"] == [{"name": "DEAD"}]


# ---------------------------------------------------------------------------
# FastMCP registration
# ---------------------------------------------------------------------------

def test_all_tools_register_on_fastmcp():
    from mcp.server.fastmcp import FastMCP
    server = FastMCP("test-sap")
    st.register(server)
    names = {t.name for t in asyncio.run(server.list_tools())}
    expected = {fn.__name__ for fn in st.SAP_TOOLS}
    assert expected <= names
    assert len(expected) == 16


def test_mcp_server_module_registers_sap_tools():
    """The real junior-se server must expose the sap_* tools."""
    from src import mcp_server
    names = {t.name for t in asyncio.run(mcp_server.mcp.list_tools())}
    assert {"sap_status", "sap_build_model", "sap_joint_reactions",
            "sap_define_load_combos"} <= names
