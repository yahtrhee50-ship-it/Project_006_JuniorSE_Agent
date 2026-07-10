"""
Structured conforming mesh generation, quality gates, and edge-constraint
stitching (Phase 2, step 3 — docs/fea_phase2_plan.md).

Generators build a `Mesh` (pure data: node coords, element connectivity,
named boundary groups) with fully deterministic numbering — same inputs,
same tags, always. `Model.mesh_*` hooks (api.py) run `check_quality` and
refuse to emit a failing mesh into the domain.

Generators
----------
- line_mesh          : n segments along a straight line
- rect_grid          : structured quad grid on an axis-aligned rectangle
- transfinite_patch  : mapped (Coons) quad patch on four possibly-curved
                       boundary edges, with optional grading — this is what
                       NAFEMS LE1's elliptic geometry needs
- box_grid           : structured hex grid on an axis-aligned box (Step 5/6
                       solids consume the connectivity; no Hex8 element yet)

Quality gate (brief §11.3, non-negotiable)
------------------------------------------
`check_quality` raises MeshQualityError naming the offending element and
metric: positive Jacobian at every integration point AND corner, side
aspect ratio, corner angles, and (3D quads) warp out of the mean plane.
Limits are configurable per call; the defaults are conservative — graded
meshes (e.g. LE1) legitimately need a higher aspect allowance.

Edge stitching (brief §11.2)
----------------------------
`stitch_edge` ties a fine mesh edge to a coarse one along a shared straight
interface: coincident nodes get equal-DOF constraints, hanging nodes get
linear-interpolation MP constraints on the two ends of the coarse segment
they sit on. This is an APPROXIMATION (the fine edge is forced to deform
linearly between coarse nodes) — keep tied interfaces away from regions of
interest.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from src.fea.constraints.mp import equal_dof, hanging_node
from src.fea.elements.isoparam import (H8_CORNERS, Q4_CORNERS, gauss_2d,
                                       gauss_3d, shape_h8, shape_q4)

Curve = Callable[[float], Sequence[float]]


class MeshQualityError(ValueError):
    """A generated element violates a quality limit. Carries the offending
    element tag, the metric name, its value and the limit."""

    def __init__(self, ele_tag: int, metric: str, value: float,
                 limit: float, detail: str = "") -> None:
        self.ele_tag = ele_tag
        self.metric = metric
        self.value = value
        self.limit = limit
        msg = (f"Mesh quality gate: element {ele_tag} fails '{metric}' "
               f"(value {value:.4g}, limit {limit:.4g})")
        if detail:
            msg += f" — {detail}"
        super().__init__(msg)


@dataclass
class Mesh:
    """Pure mesh data (no Domain/Model coupling).

    nodes    : tag -> coords tuple (insertion order = deterministic order)
    elements : tag -> node-tag connectivity (CCW quads; hexes = bottom face
               CCW then top face)
    kind     : 'line' | 'quad' | 'hex'
    groups   : named ORDERED node-tag tuples for boundary sets (rect/
               transfinite: 'bottom'/'right'/'top'/'left' walked in
               increasing parameter; box: 'xmin'.. 'zmax')
    sides    : named (ele_tag, local_edge) tuples matching the groups, for
               applying edge tractions (quad meshes only)
    """
    ndm: int
    kind: str
    nodes: Dict[int, Tuple[float, ...]]
    elements: Dict[int, Tuple[int, ...]]
    groups: Dict[str, Tuple[int, ...]] = field(default_factory=dict)
    sides: Dict[str, Tuple[Tuple[int, int], ...]] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# generators
# ---------------------------------------------------------------------------

def line_mesh(p0: Sequence[float], p1: Sequence[float], n: int,
              first_node: int = 1, first_ele: int = 1) -> Mesh:
    """n equal segments from p0 to p1. Groups: 'start', 'end', 'all'."""
    if n < 1:
        raise ValueError("line_mesh: n must be >= 1")
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    if p0.shape != p1.shape:
        raise ValueError("line_mesh: p0 and p1 must have the same dimension")
    if not np.linalg.norm(p1 - p0) > 0.0:
        raise ValueError("line_mesh: zero-length line")
    nodes: Dict[int, Tuple[float, ...]] = {}
    tags = []
    for i in range(n + 1):
        tag = first_node + i
        nodes[tag] = tuple((p0 + (p1 - p0) * (i / n)).tolist())
        tags.append(tag)
    elements = {first_ele + i: (tags[i], tags[i + 1]) for i in range(n)}
    return Mesh(ndm=len(p0), kind="line", nodes=nodes, elements=elements,
                groups={"start": (tags[0],), "end": (tags[-1],),
                        "all": tuple(tags)})


def _quad_mesh_from_grid(pts: np.ndarray, n_u: int, n_v: int,
                         first_node: int, first_ele: int) -> Mesh:
    """Assemble a quad Mesh from an (n_u+1, n_v+1, ndm) grid of points.
    Numbering is row-major over j (v) then i (u); element (i, j) is CCW
    (i,j)-(i+1,j)-(i+1,j+1)-(i,j+1)."""
    tag = {}
    nodes: Dict[int, Tuple[float, ...]] = {}
    k = first_node
    for j in range(n_v + 1):
        for i in range(n_u + 1):
            tag[(i, j)] = k
            nodes[k] = tuple(pts[i, j].tolist())
            k += 1
    elements: Dict[int, Tuple[int, ...]] = {}
    ele = {}
    e = first_ele
    for j in range(n_v):
        for i in range(n_u):
            elements[e] = (tag[(i, j)], tag[(i + 1, j)],
                           tag[(i + 1, j + 1)], tag[(i, j + 1)])
            ele[(i, j)] = e
            e += 1
    groups = {
        "bottom": tuple(tag[(i, 0)] for i in range(n_u + 1)),
        "right": tuple(tag[(n_u, j)] for j in range(n_v + 1)),
        "top": tuple(tag[(i, n_v)] for i in range(n_u + 1)),
        "left": tuple(tag[(0, j)] for j in range(n_v + 1)),
    }
    # local edges of the CCW quad: 0 = bottom, 1 = right, 2 = top, 3 = left
    sides = {
        "bottom": tuple((ele[(i, 0)], 0) for i in range(n_u)),
        "right": tuple((ele[(n_u - 1, j)], 1) for j in range(n_v)),
        "top": tuple((ele[(i, n_v - 1)], 2) for i in range(n_u)),
        "left": tuple((ele[(0, j)], 3) for j in range(n_v)),
    }
    return Mesh(ndm=pts.shape[2], kind="quad", nodes=nodes,
                elements=elements, groups=groups, sides=sides)


def rect_grid(Lx: float, Ly: float, nx: int, ny: int, *,
              x0: float = 0.0, y0: float = 0.0,
              first_node: int = 1, first_ele: int = 1) -> Mesh:
    """Structured quad grid on [x0, x0+Lx] x [y0, y0+Ly]."""
    if Lx <= 0 or Ly <= 0:
        raise ValueError("rect_grid: Lx and Ly must be positive")
    if nx < 1 or ny < 1:
        raise ValueError("rect_grid: nx and ny must be >= 1")
    pts = np.empty((nx + 1, ny + 1, 2))
    for j in range(ny + 1):
        for i in range(nx + 1):
            pts[i, j] = (x0 + Lx * i / nx, y0 + Ly * j / ny)
    return _quad_mesh_from_grid(pts, nx, ny, first_node, first_ele)


def _check_grading(fn: Optional[Callable[[float], float]], name: str
                   ) -> Callable[[float], float]:
    if fn is None:
        return lambda t: t
    if abs(fn(0.0)) > 1e-12 or abs(fn(1.0) - 1.0) > 1e-12:
        raise ValueError(f"{name}: grading must map 0 -> 0 and 1 -> 1")
    samples = [fn(t) for t in np.linspace(0.0, 1.0, 33)]
    if any(b <= a for a, b in zip(samples, samples[1:])):
        raise ValueError(f"{name}: grading must be strictly increasing")
    return fn


def transfinite_patch(bottom: Curve, right: Curve, top: Curve, left: Curve,
                      n_u: int, n_v: int, *,
                      grade_u: Optional[Callable[[float], float]] = None,
                      grade_v: Optional[Callable[[float], float]] = None,
                      first_node: int = 1, first_ele: int = 1,
                      tol: float = 1e-8) -> Mesh:
    """Mapped (Coons) quad patch on four boundary curves.

    Parameterization: bottom/top run u = 0 -> 1, left/right run v = 0 -> 1,
    and the corners must agree: bottom(0) == left(0), bottom(1) == right(0),
    top(0) == left(1), top(1) == right(1) (within tol x patch size).
    grade_u / grade_v: strictly-increasing [0,1] -> [0,1] maps applied to
    the uniform parameter samples (mesh grading toward an edge).
    """
    if n_u < 1 or n_v < 1:
        raise ValueError("transfinite_patch: n_u and n_v must be >= 1")
    gu = _check_grading(grade_u, "grade_u")
    gv = _check_grading(grade_v, "grade_v")
    P00 = np.asarray(bottom(0.0), dtype=float)
    P10 = np.asarray(bottom(1.0), dtype=float)
    P01 = np.asarray(top(0.0), dtype=float)
    P11 = np.asarray(top(1.0), dtype=float)
    corners = {"bottom(0) vs left(0)": (P00, np.asarray(left(0.0), float)),
               "bottom(1) vs right(0)": (P10, np.asarray(right(0.0), float)),
               "top(0) vs left(1)": (P01, np.asarray(left(1.0), float)),
               "top(1) vs right(1)": (P11, np.asarray(right(1.0), float))}
    scale = max(np.linalg.norm(P10 - P00), np.linalg.norm(P01 - P00),
                np.linalg.norm(P11 - P00), 1.0)
    for name, (a, b) in corners.items():
        if np.linalg.norm(a - b) > tol * scale:
            raise ValueError(
                f"transfinite_patch: corner mismatch {name}: "
                f"{a.tolist()} vs {b.tolist()}")
    ndm = P00.shape[0]
    pts = np.empty((n_u + 1, n_v + 1, ndm))
    us = [gu(i / n_u) for i in range(n_u + 1)]
    vs = [gv(j / n_v) for j in range(n_v + 1)]
    B = [np.asarray(bottom(u), float) for u in us]
    T = [np.asarray(top(u), float) for u in us]
    L = [np.asarray(left(v), float) for v in vs]
    R = [np.asarray(right(v), float) for v in vs]
    for j, v in enumerate(vs):
        for i, u in enumerate(us):
            pts[i, j] = ((1 - v) * B[i] + v * T[i]
                         + (1 - u) * L[j] + u * R[j]
                         - ((1 - u) * (1 - v) * P00 + u * (1 - v) * P10
                            + u * v * P11 + (1 - u) * v * P01))
    return _quad_mesh_from_grid(pts, n_u, n_v, first_node, first_ele)


def box_grid(Lx: float, Ly: float, Lz: float, nx: int, ny: int, nz: int, *,
             x0: float = 0.0, y0: float = 0.0, z0: float = 0.0,
             first_node: int = 1, first_ele: int = 1) -> Mesh:
    """Structured hex grid on an axis-aligned box. Connectivity per hex:
    bottom face CCW viewed from +z, then the top face in the same pattern
    (matches isoparam.H8_CORNERS). Groups: 'xmin'/'xmax'/'ymin'/'ymax'/
    'zmin'/'zmax' face node sets (ordered i, then j, then k)."""
    if Lx <= 0 or Ly <= 0 or Lz <= 0:
        raise ValueError("box_grid: Lx, Ly, Lz must be positive")
    if nx < 1 or ny < 1 or nz < 1:
        raise ValueError("box_grid: nx, ny, nz must be >= 1")
    tag = {}
    nodes: Dict[int, Tuple[float, ...]] = {}
    t = first_node
    for k in range(nz + 1):
        for j in range(ny + 1):
            for i in range(nx + 1):
                tag[(i, j, k)] = t
                nodes[t] = (x0 + Lx * i / nx, y0 + Ly * j / ny,
                            z0 + Lz * k / nz)
                t += 1
    elements: Dict[int, Tuple[int, ...]] = {}
    e = first_ele
    for k in range(nz):
        for j in range(ny):
            for i in range(nx):
                elements[e] = (
                    tag[(i, j, k)], tag[(i + 1, j, k)],
                    tag[(i + 1, j + 1, k)], tag[(i, j + 1, k)],
                    tag[(i, j, k + 1)], tag[(i + 1, j, k + 1)],
                    tag[(i + 1, j + 1, k + 1)], tag[(i, j + 1, k + 1)])
                e += 1

    def _face(pred) -> Tuple[int, ...]:
        return tuple(tag[(i, j, k)]
                     for k in range(nz + 1)
                     for j in range(ny + 1)
                     for i in range(nx + 1) if pred(i, j, k))

    groups = {"xmin": _face(lambda i, j, k: i == 0),
              "xmax": _face(lambda i, j, k: i == nx),
              "ymin": _face(lambda i, j, k: j == 0),
              "ymax": _face(lambda i, j, k: j == ny),
              "zmin": _face(lambda i, j, k: k == 0),
              "zmax": _face(lambda i, j, k: k == nz)}
    return Mesh(ndm=3, kind="hex", nodes=nodes, elements=elements,
                groups=groups)


# ---------------------------------------------------------------------------
# quality gate
# ---------------------------------------------------------------------------

def _quad_metrics(coords: np.ndarray) -> Dict[str, float]:
    """Worst-case metrics of one quad: min detJ (2D only, at 2x2 Gauss
    points and the 4 corners), side aspect ratio, min/max corner angle
    (deg), warp (3D only: max node distance from the mean plane / mean
    side length)."""
    out: Dict[str, float] = {}
    sides = [coords[(a + 1) % 4] - coords[a] for a in range(4)]
    lengths = [float(np.linalg.norm(s)) for s in sides]
    if min(lengths) <= 0.0:
        out["aspect"] = np.inf
        out["min_angle_deg"] = 0.0
        out["max_angle_deg"] = 180.0
        out["min_detJ"] = -np.inf if coords.shape[1] == 2 else np.nan
        out["warp"] = 0.0
        return out
    out["aspect"] = max(lengths) / min(lengths)
    angles = []
    for a in range(4):
        v1 = sides[a] / lengths[a]
        v0 = -sides[(a - 1) % 4] / lengths[(a - 1) % 4]
        angles.append(np.degrees(np.arccos(np.clip(np.dot(v0, v1), -1, 1))))
    out["min_angle_deg"] = float(min(angles))
    out["max_angle_deg"] = float(max(angles))
    if coords.shape[1] == 2:
        pts, _ = gauss_2d(2)
        min_det = np.inf
        for xi, eta in np.vstack([pts, Q4_CORNERS]):
            _, dN = shape_q4(xi, eta)
            min_det = min(min_det, float(np.linalg.det(coords.T @ dN)))
        out["min_detJ"] = min_det
        out["warp"] = 0.0
    else:
        d1 = coords[2] - coords[0]
        d2 = coords[3] - coords[1]
        n = np.cross(d1, d2)
        nn = np.linalg.norm(n)
        if nn == 0.0:
            out["warp"] = np.inf
        else:
            n = n / nn
            c = coords.mean(axis=0)
            out["warp"] = float(max(abs((p - c) @ n) for p in coords)
                                / np.mean(lengths))
        out["min_detJ"] = np.nan   # no 2D-parametric detJ for a 3D quad
    return out


def _hex_metrics(coords: np.ndarray) -> Dict[str, float]:
    """Min detJ at 2x2x2 Gauss points + the 8 corners, and edge aspect."""
    pts, _ = gauss_3d(2)
    min_det = np.inf
    for xi, eta, ze in np.vstack([pts, H8_CORNERS]):
        _, dN = shape_h8(xi, eta, ze)
        min_det = min(min_det, float(np.linalg.det(coords.T @ dN)))
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    lengths = [float(np.linalg.norm(coords[b] - coords[a])) for a, b in edges]
    aspect = max(lengths) / min(lengths) if min(lengths) > 0 else np.inf
    return {"min_detJ": min_det, "aspect": aspect}


def check_quality(mesh: Mesh, *, max_aspect: float = 10.0,
                  min_angle_deg: float = 20.0, max_angle_deg: float = 160.0,
                  max_warp: float = 0.05) -> Dict[str, float]:
    """Gate every element; raise MeshQualityError on the first violation.

    Checks by kind — quad: detJ > 0 at all 2x2 Gauss points AND corners
    (2D), aspect, corner angles, warp (3D quads); hex: detJ > 0 at 2x2x2
    Gauss points + corners, edge aspect; line: positive length.
    Returns the worst-case metrics over the whole mesh (for reporting).
    """
    worst: Dict[str, float] = {}

    def _worse(name: str, value: float, larger_is_worse: bool) -> None:
        if name not in worst:
            worst[name] = value
        else:
            worst[name] = (max if larger_is_worse else min)(worst[name], value)

    for tag, conn in mesh.elements.items():
        coords = np.array([mesh.nodes[t] for t in conn], dtype=float)
        if mesh.kind == "line":
            L = float(np.linalg.norm(coords[1] - coords[0]))
            _worse("min_length", L, False)
            if L <= 0.0:
                raise MeshQualityError(tag, "length", L, 0.0,
                                       "zero-length segment")
            continue
        if mesh.kind == "quad":
            m = _quad_metrics(coords)
            _worse("min_detJ", m["min_detJ"], False)
            _worse("aspect", m["aspect"], True)
            _worse("min_angle_deg", m["min_angle_deg"], False)
            _worse("max_angle_deg", m["max_angle_deg"], True)
            _worse("warp", m["warp"], True)
            if mesh.ndm == 2 and not m["min_detJ"] > 0.0:
                raise MeshQualityError(
                    tag, "jacobian", m["min_detJ"], 0.0,
                    "element is inverted or numbered clockwise")
            if m["aspect"] > max_aspect:
                raise MeshQualityError(tag, "aspect", m["aspect"], max_aspect)
            if m["min_angle_deg"] < min_angle_deg:
                raise MeshQualityError(tag, "min_angle_deg",
                                       m["min_angle_deg"], min_angle_deg,
                                       "sliver / collapsed corner")
            if m["max_angle_deg"] > max_angle_deg:
                raise MeshQualityError(tag, "max_angle_deg",
                                       m["max_angle_deg"], max_angle_deg)
            if mesh.ndm == 3 and m["warp"] > max_warp:
                raise MeshQualityError(tag, "warp", m["warp"], max_warp,
                                       "non-planar 3D quad")
            continue
        if mesh.kind == "hex":
            m = _hex_metrics(coords)
            _worse("min_detJ", m["min_detJ"], False)
            _worse("aspect", m["aspect"], True)
            if not m["min_detJ"] > 0.0:
                raise MeshQualityError(
                    tag, "jacobian", m["min_detJ"], 0.0,
                    "hex is inverted or badly numbered")
            if m["aspect"] > max_aspect:
                raise MeshQualityError(tag, "aspect", m["aspect"], max_aspect)
            continue
        raise ValueError(f"check_quality: unknown mesh kind '{mesh.kind}'")
    return worst


# ---------------------------------------------------------------------------
# edge stitching (hanging nodes -> MP constraints)
# ---------------------------------------------------------------------------

def stitch_edge(coarse_tags: Sequence[int], fine_tags: Sequence[int],
                coords: Mapping[int, Sequence[float]], *,
                dofs: Sequence[int] = (0, 1), tol: float = 1e-8) -> List:
    """Constraints tying the fine edge to the coarse edge (both node-tag
    sequences walked along the SAME geometric interface).

    Per fine node: if it coincides with a coarse node -> equal-DOF; if it
    lies strictly inside a coarse segment -> hanging-node interpolation on
    that segment's end nodes; otherwise raise. Fine tags already present in
    `coarse_tags` (a genuinely shared node) are skipped. Returns the
    MP-constraint list — callers add them to the domain (Model.stitch_edge
    does this)."""
    if len(coarse_tags) < 2:
        raise ValueError("stitch_edge: coarse edge needs >= 2 nodes")
    dofs = tuple(dofs)
    cpts = [np.asarray(coords[t], dtype=float) for t in coarse_tags]
    scale = sum(np.linalg.norm(b - a) for a, b in zip(cpts, cpts[1:]))
    if scale <= 0.0:
        raise ValueError("stitch_edge: coarse edge has zero length")
    atol = tol * scale
    out: List = []
    coarse_set = set(coarse_tags)
    for ft in fine_tags:
        if ft in coarse_set:
            continue
        p = np.asarray(coords[ft], dtype=float)
        # coincident with a coarse node?
        hit = next((ct for ct, cp in zip(coarse_tags, cpts)
                    if np.linalg.norm(p - cp) <= atol), None)
        if hit is not None:
            out.append(equal_dof(hit, ft, dofs))
            continue
        # inside a coarse segment?
        placed = False
        for (ta, pa), (tb, pb) in zip(zip(coarse_tags, cpts),
                                      zip(coarse_tags[1:], cpts[1:])):
            seg = pb - pa
            L2 = float(seg @ seg)
            s = float((p - pa) @ seg) / L2
            if -tol < s < 1.0 + tol:
                perp = np.linalg.norm(p - (pa + np.clip(s, 0, 1) * seg))
                if perp <= atol:
                    out.append(hanging_node(ta, tb, ft, dofs,
                                            float(np.clip(s, 0.0, 1.0))))
                    placed = True
                    break
        if not placed:
            raise ValueError(
                f"stitch_edge: fine node {ft} at {p.tolist()} does not lie "
                f"on the coarse edge (tol {atol:.3g})")
    return out
