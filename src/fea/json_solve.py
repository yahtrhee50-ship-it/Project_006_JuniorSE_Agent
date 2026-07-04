"""
JSON front door for the FEA engine — powers the `solve_model` MCP tool.

Takes a Tier-1 model as plain JSON-style dicts/lists, builds a `fea.Model`,
runs the linear static analysis, and returns a Markdown-ish results text:
node displacements, support reactions, element end forces, and a global
statics check (sum of reactions vs sum of applied loads — the plan's §15
"sanity-check reactions against statics" gate).

Schema (2D; ndf=3 frame DOFs unless set to 2 for pure truss):
    nodes:     [{"id": 1, "x": 0.0, "y": 0.0, "fix": [1, 1, 0]}, ...]
    sections:  [{"id": 1, "E": 29000.0, "A": 20.0, "I": 800.0}, ...]
    materials: [{"id": 1, "E": 29000.0}, ...]              (Truss elements)
    elements:  [{"id": 1, "type": "Frame", "i": 1, "j": 2, "section": 1,
                 "release_i": false, "release_j": false,
                 "offset_i": [0,0], "offset_j": [0,0]},
                {"id": 2, "type": "Truss", "i": 2, "j": 3, "A": 2.0, "mat": 1}]
    nodal_loads: [{"node": 2, "Fx": 0, "Fy": -25.0, "Mz": 0}, ...]
    ele_loads:   [{"element": 1, "wx": 0, "wy": -0.1667},            (uniform)
                  {"element": 1, "Px": 0, "Py": -10, "distance_from_i": 60},
                  {"element": 2, "delta_L0": 0.05},                  (Truss)
                  ...]

Units: one consistent set in -> same set out (imperial: kips, in, ksi).
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

from src.fea.api import Model


def solve_model_json(nodes: List[dict], elements: List[dict],
                     sections: Optional[List[dict]] = None,
                     materials: Optional[List[dict]] = None,
                     nodal_loads: Optional[List[dict]] = None,
                     ele_loads: Optional[List[dict]] = None,
                     ndf: int = 3, numberer: str = "Plain") -> str:
    if not nodes:
        raise ValueError("nodes is empty")
    if not elements:
        raise ValueError("elements is empty")
    if ndf not in (2, 3):
        raise ValueError("ndf must be 2 (truss-only) or 3 (frame DOFs)")

    m = Model(ndm=2, ndf=ndf)
    m.numberer(numberer)

    fixed_nodes: List[int] = []
    for nd in nodes:
        tag = int(nd["id"])
        m.node(tag, float(nd["x"]), float(nd["y"]))
        fix = nd.get("fix")
        if fix:
            if len(fix) != ndf:
                raise ValueError(f"node {tag}: fix needs {ndf} flags")
            if any(fix):
                m.fix(tag, *[int(bool(f)) for f in fix])
                fixed_nodes.append(tag)

    for s in (sections or []):
        m.section("Elastic", int(s["id"]), E=float(s["E"]), A=float(s["A"]),
                  Iz=float(s["I"]))
    for mt in (materials or []):
        m.uniaxial_material("Elastic", int(mt["id"]), E=float(mt["E"]))

    lengths: Dict[int, float] = {}
    for el in elements:
        tag, kind = int(el["id"]), str(el["type"])
        i, j = int(el["i"]), int(el["j"])
        (xi, yi), (xj, yj) = (m.domain.get_node(i).coords,
                              m.domain.get_node(j).coords)
        lengths[tag] = math.hypot(xj - xi, yj - yi)
        if kind == "Frame":
            if ndf != 3:
                raise ValueError("Frame elements need ndf=3")
            m.element("Frame", tag, i, j, section=int(el["section"]),
                      release_i=bool(el.get("release_i", False)),
                      release_j=bool(el.get("release_j", False)),
                      offset_i=tuple(el.get("offset_i", (0.0, 0.0))),
                      offset_j=tuple(el.get("offset_j", (0.0, 0.0))))
        elif kind == "Truss":
            m.element("Truss", tag, i, j, A=float(el["A"]), mat=int(el["mat"]))
        else:
            raise ValueError(f"element {tag}: unknown type '{kind}' "
                             "(Frame|Truss)")

    ends: Dict[int, tuple] = {int(el["id"]): (int(el["i"]), int(el["j"]))
                              for el in elements}

    def _coords(node_tag: int) -> tuple:
        c = m.domain.get_node(node_tag).coords
        return float(c[0]), float(c[1])

    m.pattern("Plain", 1, "Linear")
    # applied-load totals for the statics check (moments about the origin)
    sum_fx = sum_fy = sum_m = 0.0
    for ld in (nodal_loads or []):
        tag = int(ld["node"])
        fx, fy = float(ld.get("Fx", 0.0)), float(ld.get("Fy", 0.0))
        mz = float(ld.get("Mz", 0.0)) if ndf == 3 else 0.0
        vals = (fx, fy) if ndf == 2 else (fx, fy, mz)
        m.load(tag, *vals)
        x, y = _coords(tag)
        sum_fx += fx
        sum_fy += fy
        sum_m += mz + x * fy - y * fx
    for ld in (ele_loads or []):
        tag = int(ld["element"])
        i, j = ends[tag]
        (xi, yi), (xj, yj) = _coords(i), _coords(j)
        if "distance_from_i" in ld:
            px, py = float(ld.get("Px", 0.0)), float(ld.get("Py", 0.0))
            a = float(ld["distance_from_i"])
            m.ele_load(tag, Px=px, Py=py, distance_from_i=a)
            t = a / lengths[tag]
            x, y = xi + t * (xj - xi), yi + t * (yj - yi)
            sum_fx += px
            sum_fy += py
            sum_m += x * py - y * px
        elif "wx" in ld or "wy" in ld:
            wx, wy = float(ld.get("wx", 0.0)), float(ld.get("wy", 0.0))
            m.ele_load(tag, wx=wx, wy=wy)
            fx, fy = wx * lengths[tag], wy * lengths[tag]
            xm, ym = (xi + xj) / 2.0, (yi + yj) / 2.0   # uniform resultant
            sum_fx += fx
            sum_fy += fy
            sum_m += xm * fy - ym * fx
        else:   # truss initial strain — internal, no net applied force
            m.ele_load(tag, delta_L0=float(ld.get("delta_L0", 0.0)),
                       delta_T=float(ld.get("delta_T", 0.0)),
                       alpha=float(ld.get("alpha", 0.0)))

    m.analyze(1)

    # ---- report ------------------------------------------------------------
    disp_hdr = ("  {:>6}  {:>14}  {:>14}".format("Node", "ux", "uy")
                if ndf == 2 else
                "  {:>6}  {:>14}  {:>14}  {:>14}".format("Node", "ux", "uy",
                                                         "rz"))
    lines = [
        f"Linear static solve — {len(nodes)} nodes, {len(elements)} elements "
        f"(ndf={ndf}, numberer={numberer})",
        "Units: consistent set as input (imperial: kips, in, ksi).",
        "",
        "## Node Displacements",
        disp_hdr,
    ]
    for nd in nodes:
        tag = int(nd["id"])
        d = m.node_disp(tag)
        lines.append("  {:>6}  ".format(tag)
                     + "  ".join(f"{v:14.6e}" for v in d))

    lines += ["", "## Support Reactions",
              ("  {:>6}  {:>12}  {:>12}".format("Node", "Rx", "Ry")
               if ndf == 2 else
               "  {:>6}  {:>12}  {:>12}  {:>12}".format("Node", "Rx", "Ry",
                                                        "Mz"))]
    rx_sum = ry_sum = rm_sum = 0.0
    for tag in fixed_nodes:
        r = m.node_reaction(tag)
        x, y = _coords(tag)
        rx_sum += float(r[0])
        ry_sum += float(r[1])
        rm_sum += (float(r[2]) if ndf == 3 else 0.0) \
            + x * float(r[1]) - y * float(r[0])
        lines.append("  {:>6}  ".format(tag)
                     + "  ".join(f"{v:12.4f}" for v in r))

    lines += ["", "## Element End Forces (global components, i then j)"]
    for el in elements:
        tag = int(el["id"])
        f = m.ele_force(tag)
        lines.append(f"  ele {tag:>4}: " + "  ".join(f"{v:11.4f}" for v in f))

    res_fx, res_fy = rx_sum + sum_fx, ry_sum + sum_fy
    res_m = rm_sum + sum_m
    scale = max(1.0, abs(sum_fx), abs(sum_fy), abs(sum_m))
    ok = (abs(res_fx) <= 1e-8 * scale and abs(res_fy) <= 1e-8 * scale
          and abs(res_m) <= 1e-8 * scale)
    lines += [
        "",
        "## Global Statics Check (reactions + applied loads; M about origin)",
        f"  sum Fx: reactions {rx_sum:+.6f}  + applied {sum_fx:+.6f}  "
        f"= {res_fx:+.3e}",
        f"  sum Fy: reactions {ry_sum:+.6f}  + applied {sum_fy:+.6f}  "
        f"= {res_fy:+.3e}",
        f"  sum Mz: reactions {rm_sum:+.6f}  + applied {sum_m:+.6f}  "
        f"= {res_m:+.3e}",
        f"  equilibrium: {'OK' if ok else '*** FAILED — DO NOT TRUST ***'}",
    ]
    if not ok:
        lines.append("  (Check supports/loads; a failed statics check means "
                     "the results are unusable.)")
    return "\n".join(lines)
