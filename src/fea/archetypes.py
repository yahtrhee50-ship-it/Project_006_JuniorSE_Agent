"""
Tier-2 archetypes (plan §Phase 1 item 5) — generators that expand a terse
structural description into a meshed Tier-1 Model.

Each generator returns an `Archetype`: the built (unanalyzed) Model plus
named key nodes, support nodes, and element tags, so callers can analyze and
pull results without knowing the meshing. Loads land in load pattern 1
("Plain"/Linear); add more patterns to `arch.model` directly for multi-case
work with `Model.analyze_cases`.

Conventions
-----------
- Units: any one consistent set (imperial examples: kips & inches, E in ksi).
- Load signs are GLOBAL components: gravity acts in -y, so a downward UDL is
  `udl_wy = -w` and a downward point load is `Py = -P`.
- Point loads are applied by inserting a mesh node at the load position and
  loading it nodally — exact for Euler-Bernoulli elements.
- 2D models, ndf = 3 (ux, uy, rz).

Example (30-ft simple beam, W14x30-ish, 2 klf down)::

    from src.fea.archetypes import simply_supported_beam
    a = simply_supported_beam(L=360.0, E=29000.0, A=8.85, I=291.0,
                              udl_wy=-2.0 / 12.0)          # kip/in
    a.model.analyze(1)
    mid_defl = a.model.node_disp(a.key_nodes["mid"], 1)     # 5wL^4/384EI
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

from src.fea.api import Model

_TOL = 1e-9


@dataclass
class Archetype:
    """A generated model plus the handles needed to use it."""
    model: Model
    key_nodes: Dict[str, int]
    support_nodes: List[int]
    element_tags: List[int]
    node_coords: Dict[int, Tuple[float, float]] = field(default_factory=dict)

    def node_near(self, x: float, y: float = 0.0, tol: float = 1e-6) -> int:
        """Tag of the node at (x, y); raises if none is within tol."""
        for tag, (nx, ny) in self.node_coords.items():
            if abs(nx - x) <= tol and abs(ny - y) <= tol:
                return tag
        raise KeyError(f"No node at ({x}, {y})")


def _merge_positions(L: float, n_ele: int, extra: Sequence[float]) -> List[float]:
    """Evenly spaced stations 0..L plus the extra positions, deduplicated."""
    if L <= 0:
        raise ValueError("length must be positive")
    if n_ele < 1:
        raise ValueError("n_ele must be >= 1")
    xs = [L * i / n_ele for i in range(n_ele + 1)]
    for x in extra:
        if not -_TOL <= x <= L + _TOL:
            raise ValueError(f"position {x} outside [0, {L}]")
        xs.append(min(max(x, 0.0), L))
    xs.sort()
    out: List[float] = []
    for x in xs:
        if not out or x - out[-1] > _TOL * max(1.0, L):
            out.append(x)
    out[-1] = L
    return out


def _line_of_frames(m: Model, xs: List[float], sec_tag: int,
                    first_node: int, first_ele: int,
                    y: float = 0.0) -> Tuple[List[int], List[int]]:
    """Nodes along y=const at stations xs, chained with Frame elements."""
    node_tags = []
    for i, x in enumerate(xs):
        tag = first_node + i
        m.node(tag, x, y)
        node_tags.append(tag)
    ele_tags = []
    for i in range(len(xs) - 1):
        tag = first_ele + i
        m.element("Frame", tag, node_tags[i], node_tags[i + 1], section=sec_tag)
        ele_tags.append(tag)
    return node_tags, ele_tags


def _apply_line_loads(m: Model, arch_nodes: List[int], xs: List[float],
                      ele_tags: List[int], udl_wy: float, udl_wx: float,
                      point_loads: Sequence[Tuple[float, float, float]]) -> None:
    """UDL on every element + nodal point loads at their (pre-meshed) x."""
    if udl_wy or udl_wx:
        for tag in ele_tags:
            m.ele_load(tag, wx=udl_wx, wy=udl_wy)
    for (x, Px, Py) in point_loads:
        idx = min(range(len(xs)), key=lambda i: abs(xs[i] - x))
        m.load(arch_nodes[idx], Px, Py, 0.0)


def simply_supported_beam(L: float, E: float, A: float, I: float,
                          n_ele: int = 10, udl_wy: float = 0.0,
                          udl_wx: float = 0.0,
                          point_loads: Sequence[Tuple[float, float, float]] = (),
                          ) -> Archetype:
    """Pin (x=0) + roller (x=L) beam meshed into n_ele Frame elements.

    point_loads: (x, Px, Py) triples, GLOBAL components (Py<0 = down).
    Key nodes: 'left', 'right', 'mid' (node closest to midspan).
    """
    xs = _merge_positions(L, n_ele, [x for x, _, _ in point_loads] + [L / 2.0])
    m = Model(ndm=2, ndf=3)
    m.section("Elastic", 1, E=E, A=A, Iz=I)
    nodes, eles = _line_of_frames(m, xs, 1, first_node=1, first_ele=1)
    m.fix(nodes[0], 1, 1, 0)            # pin
    m.fix(nodes[-1], 0, 1, 0)           # roller
    m.pattern("Plain", 1, "Linear")
    _apply_line_loads(m, nodes, xs, eles, udl_wy, udl_wx, point_loads)
    mid = nodes[min(range(len(xs)), key=lambda i: abs(xs[i] - L / 2.0))]
    return Archetype(m, {"left": nodes[0], "right": nodes[-1], "mid": mid},
                     [nodes[0], nodes[-1]], eles,
                     {t: (x, 0.0) for t, x in zip(nodes, xs)})


def cantilever(L: float, E: float, A: float, I: float, n_ele: int = 10,
               tip_Px: float = 0.0, tip_Py: float = 0.0, udl_wy: float = 0.0,
               point_loads: Sequence[Tuple[float, float, float]] = (),
               ) -> Archetype:
    """Fixed at x=0, free at x=L. Key nodes: 'base', 'tip'."""
    xs = _merge_positions(L, n_ele, [x for x, _, _ in point_loads])
    m = Model(ndm=2, ndf=3)
    m.section("Elastic", 1, E=E, A=A, Iz=I)
    nodes, eles = _line_of_frames(m, xs, 1, first_node=1, first_ele=1)
    m.fix(nodes[0], 1, 1, 1)
    m.pattern("Plain", 1, "Linear")
    _apply_line_loads(m, nodes, xs, eles, udl_wy, 0.0, point_loads)
    if tip_Px or tip_Py:
        m.load(nodes[-1], tip_Px, tip_Py, 0.0)
    return Archetype(m, {"base": nodes[0], "tip": nodes[-1]},
                     [nodes[0]], eles,
                     {t: (x, 0.0) for t, x in zip(nodes, xs)})


def continuous_beam(spans: Sequence[float], E: float, A: float, I: float,
                    n_ele_per_span: int = 8, udl_wy: float = 0.0,
                    point_loads: Sequence[Tuple[float, float, float]] = (),
                    ) -> Archetype:
    """Multi-span beam: pin at x=0, roller at every other support.

    spans: span lengths left to right. point_loads use GLOBAL x from the
    left end. Key nodes: 'support_0'..'support_n' plus 'mid_span_1'... .
    """
    if not spans or any(s <= 0 for s in spans):
        raise ValueError("spans must be positive lengths")
    L = float(sum(spans))
    support_xs = [0.0]
    for s in spans:
        support_xs.append(support_xs[-1] + float(s))
    mids = [support_xs[i] + spans[i] / 2.0 for i in range(len(spans))]
    # global even mesh per span so supports always land on nodes
    xs: List[float] = []
    for i in range(len(spans)):
        xs += [support_xs[i] + spans[i] * k / n_ele_per_span
               for k in range(n_ele_per_span)]
    xs.append(L)
    xs += [min(max(x, 0.0), L) for x, _, _ in point_loads] + mids
    xs.sort()
    deduped: List[float] = []
    for x in xs:
        if not deduped or x - deduped[-1] > _TOL * max(1.0, L):
            deduped.append(x)
    deduped[-1] = L
    xs = deduped
    m = Model(ndm=2, ndf=3)
    m.section("Elastic", 1, E=E, A=A, Iz=I)
    nodes, eles = _line_of_frames(m, xs, 1, first_node=1, first_ele=1)

    def _at(x: float) -> int:
        return nodes[min(range(len(xs)), key=lambda i: abs(xs[i] - x))]

    key = {}
    supports = []
    for i, sx in enumerate(support_xs):
        tag = _at(sx)
        supports.append(tag)
        key[f"support_{i}"] = tag
        if i == 0:
            m.fix(tag, 1, 1, 0)
        else:
            m.fix(tag, 0, 1, 0)
    for i, mx in enumerate(mids):
        key[f"mid_span_{i + 1}"] = _at(mx)
    m.pattern("Plain", 1, "Linear")
    _apply_line_loads(m, nodes, xs, eles, udl_wy, 0.0, point_loads)
    return Archetype(m, key, supports, eles,
                     {t: (x, 0.0) for t, x in zip(nodes, xs)})


def portal_frame(width: float, height: float, E: float,
                 A_col: float, I_col: float, A_beam: float, I_beam: float,
                 n_ele_per_member: int = 4, fixed_base: bool = True,
                 lateral_Px: float = 0.0, beam_udl_wy: float = 0.0,
                 ) -> Archetype:
    """Single-bay single-story rectangular portal frame.

    Columns x=0 and x=width from y=0 to y=height; beam across the top.
    lateral_Px is applied at the top-left corner. Key nodes: 'base_left',
    'base_right', 'top_left', 'top_right', 'beam_mid'.
    """
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    if n_ele_per_member < 1:
        raise ValueError("n_ele_per_member must be >= 1")
    n = n_ele_per_member
    m = Model(ndm=2, ndf=3)
    m.section("Elastic", 1, E=E, A=A_col, Iz=I_col)     # columns
    m.section("Elastic", 2, E=E, A=A_beam, Iz=I_beam)   # beam

    coords: Dict[int, Tuple[float, float]] = {}
    tag = 0

    def _node(x: float, y: float) -> int:
        nonlocal tag
        for t, (nx, ny) in coords.items():
            if abs(nx - x) < _TOL and abs(ny - y) < _TOL:
                return t
        tag += 1
        m.node(tag, x, y)
        coords[tag] = (x, y)
        return tag

    ele = 0
    eles: List[int] = []

    def _member(p0: Tuple[float, float], p1: Tuple[float, float],
                sec: int) -> List[int]:
        nonlocal ele
        pts = [(_node(p0[0] + (p1[0] - p0[0]) * k / n,
                      p0[1] + (p1[1] - p0[1]) * k / n)) for k in range(n + 1)]
        out = []
        for a, b in zip(pts, pts[1:]):
            ele += 1
            m.element("Frame", ele, a, b, section=sec)
            out.append(ele)
        eles.extend(out)
        return out

    _member((0.0, 0.0), (0.0, height), 1)                 # left column
    _member((width, 0.0), (width, height), 1)             # right column
    beam_eles = _member((0.0, height), (width, height), 2)

    base_l, base_r = _node(0.0, 0.0), _node(width, 0.0)
    top_l, top_r = _node(0.0, height), _node(width, height)
    if fixed_base:
        m.fix(base_l, 1, 1, 1)
        m.fix(base_r, 1, 1, 1)
    else:
        m.fix(base_l, 1, 1, 0)
        m.fix(base_r, 1, 1, 0)

    m.pattern("Plain", 1, "Linear")
    if lateral_Px:
        m.load(top_l, lateral_Px, 0.0, 0.0)
    if beam_udl_wy:
        for t in beam_eles:
            m.ele_load(t, wy=beam_udl_wy)

    beam_mid = _node(width / 2.0, height) if n % 2 == 0 else \
        min(coords, key=lambda t: (abs(coords[t][0] - width / 2.0)
                                   + abs(coords[t][1] - height)))
    key = {"base_left": base_l, "base_right": base_r,
           "top_left": top_l, "top_right": top_r, "beam_mid": beam_mid}
    return Archetype(m, key, [base_l, base_r], eles, coords)
