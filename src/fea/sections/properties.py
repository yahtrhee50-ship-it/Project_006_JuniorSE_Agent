"""
Elastic section property tool (plan §Phase 1 item 4; §5.5 elastic part).

Computes A, centroid, second moments (Ix, Iy, Ixy about the centroid),
principal moments/axes, and radii of gyration for sections built from
primitive shapes (rectangles, circles, arbitrary polygons), with holes as
negative shapes. Feeds ElasticSection.

Torsion constant J and shear areas are EXACT only for single solid
primitives (rectangle/circle). For composites, J is the thin-walled OPEN
section approximation sum(J_i) — flag ``j_is_approximate``. Polygons carry
no J/Av (returned as 0.0 with the approximate flag set). Closed/thin-walled
cell sections (HSS) are NOT handled — compute J by hand or use catalogue
values (e.g. AISC database via src.calcs.sections).

Units: any consistent length unit in -> same unit powers out (in imperial,
inches -> in^2 / in^4).

Nonlinear moment-curvature / P-M interaction is Phase 5 (fiber sections).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

from src.fea.sections.section import ElasticSection

# Saint-Venant torsion beta for solid rectangle J = beta * a * b^3 (a >= b),
# interpolated on a/b (Roark / Timoshenko table).
_RECT_BETA = [(1.0, 0.141), (1.2, 0.166), (1.5, 0.196), (2.0, 0.229),
              (2.5, 0.249), (3.0, 0.263), (4.0, 0.281), (5.0, 0.291),
              (10.0, 0.312)]
_RECT_BETA_INF = 1.0 / 3.0


def _rect_beta(aspect: float) -> float:
    if aspect >= _RECT_BETA[-1][0]:
        # blend from the a/b=10 value toward 1/3 as 1/aspect -> 0
        return _RECT_BETA_INF - (_RECT_BETA_INF - _RECT_BETA[-1][1]) \
            * _RECT_BETA[-1][0] / aspect
    for (a0, b0), (a1, b1) in zip(_RECT_BETA, _RECT_BETA[1:]):
        if aspect <= a1:
            t = (aspect - a0) / (a1 - a0)
            return b0 + t * (b1 - b0)
    return _RECT_BETA[0][1]


@dataclass(frozen=True)
class Shape:
    """One primitive: area, centroid, and self-centroidal second moments.

    sign=+1 solid, -1 hole. J/Av are 0 where no closed form applies.
    """
    A: float
    cx: float
    cy: float
    Ix: float          # about own centroid, x (horizontal) axis
    Iy: float
    Ixy: float
    J: float = 0.0
    Avx: float = 0.0   # shear area for shear parallel to x
    Avy: float = 0.0   # shear area for shear parallel to y
    has_exact_J: bool = False
    sign: int = 1


def rectangle(b: float, h: float, cx: float = 0.0, cy: float = 0.0,
              angle_deg: float = 0.0, hole: bool = False) -> Shape:
    """Solid rectangle, width b (x) by height h (y), rotated angle_deg CCW."""
    if b <= 0 or h <= 0:
        raise ValueError("rectangle: b and h must be positive")
    A = b * h
    Ix0, Iy0 = b * h ** 3 / 12.0, h * b ** 3 / 12.0
    th = math.radians(angle_deg)
    c, s = math.cos(th), math.sin(th)
    # tensor rotation of the SHAPE by +theta (axes by -theta); Ixy0 = 0.
    # Sanity anchor: a tall rectangle tilted CCW leans its mass into
    # quadrants II/IV, so Ixy must come out negative.
    Ix = Ix0 * c * c + Iy0 * s * s
    Iy = Ix0 * s * s + Iy0 * c * c
    Ixy = -(Ix0 - Iy0) * s * c
    long_side, short_side = max(b, h), min(b, h)
    J = _rect_beta(long_side / short_side) * long_side * short_side ** 3
    return Shape(A, cx, cy, Ix, Iy, Ixy, J=J,
                 Avx=5.0 / 6.0 * A, Avy=5.0 / 6.0 * A,
                 has_exact_J=True, sign=-1 if hole else 1)


def circle(d: float, cx: float = 0.0, cy: float = 0.0,
           hole: bool = False) -> Shape:
    """Solid circle of diameter d."""
    if d <= 0:
        raise ValueError("circle: d must be positive")
    A = math.pi * d ** 2 / 4.0
    I = math.pi * d ** 4 / 64.0
    return Shape(A, cx, cy, I, I, 0.0, J=math.pi * d ** 4 / 32.0,
                 Avx=0.9 * A, Avy=0.9 * A, has_exact_J=True,
                 sign=-1 if hole else 1)


def polygon(points: Sequence[Tuple[float, float]], hole: bool = False) -> Shape:
    """Simple (non-self-intersecting) polygon by its vertices, any order
    (CW or CCW both accepted). No closed-form J/Av."""
    if len(points) < 3:
        raise ValueError("polygon: need at least 3 points")
    # shoelace with the standard cross-product weights
    A2 = Sx = Sy = Ixx = Iyy = Ixy = 0.0
    n = len(points)
    for i in range(n):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        cross = x0 * y1 - x1 * y0
        A2 += cross
        Sx += (x0 + x1) * cross
        Sy += (y0 + y1) * cross
        Ixx += (y0 * y0 + y0 * y1 + y1 * y1) * cross
        Iyy += (x0 * x0 + x0 * x1 + x1 * x1) * cross
        Ixy += (x0 * y1 + 2 * x0 * y0 + 2 * x1 * y1 + x1 * y0) * cross
    A = A2 / 2.0
    if abs(A) < 1e-300:
        raise ValueError("polygon: zero area")
    cx, cy = Sx / (3.0 * A2), Sy / (3.0 * A2)
    # about origin -> about centroid (parallel axis), sign-normalized
    Ixx, Iyy, Ixy = Ixx / 12.0, Iyy / 12.0, Ixy / 24.0
    if A < 0:
        A, Ixx, Iyy, Ixy = -A, -Ixx, -Iyy, -Ixy
    Ixx -= A * cy * cy
    Iyy -= A * cx * cx
    Ixy -= A * cx * cy
    return Shape(A, cx, cy, Ixx, Iyy, Ixy, sign=-1 if hole else 1)


def i_shape(d: float, bf: float, tf: float, tw: float) -> List[Shape]:
    """Doubly symmetric I: two bf x tf flanges + (d-2tf) x tw web, centered
    at the origin. Fillets ignored (catalogue values run ~1-2% higher)."""
    if d <= 2 * tf:
        raise ValueError("i_shape: d must exceed 2*tf")
    return [
        rectangle(bf, tf, 0.0, (d - tf) / 2.0),
        rectangle(bf, tf, 0.0, -(d - tf) / 2.0),
        rectangle(tw, d - 2 * tf, 0.0, 0.0),
    ]


@dataclass(frozen=True)
class SectionProperties:
    A: float
    cx: float
    cy: float
    Ix: float           # about centroidal x axis
    Iy: float
    Ixy: float
    I1: float           # principal moments, I1 >= I2
    I2: float
    theta_p_deg: float  # CCW angle from x axis to the I1 principal axis
    rx: float
    ry: float
    J: float            # 0.0 when unavailable
    Avx: float
    Avy: float
    j_is_approximate: bool

    def summary_lines(self) -> List[str]:
        return [
            f"A    = {self.A:.4g}",
            f"c    = ({self.cx:.4g}, {self.cy:.4g})",
            f"Ix   = {self.Ix:.6g}   Iy = {self.Iy:.6g}   Ixy = {self.Ixy:.4g}",
            f"I1   = {self.I1:.6g}   I2 = {self.I2:.6g}   "
            f"theta_p = {self.theta_p_deg:.2f} deg",
            f"rx   = {self.rx:.4g}   ry = {self.ry:.4g}",
            f"J    = {self.J:.6g}"
            + ("  (thin-walled-open approximation)" if self.j_is_approximate
               else ""),
            f"Av,x = {self.Avx:.4g}   Av,y = {self.Avy:.4g}"
            + ("  (0 = not available for this composite)"
               if self.Avx == 0.0 and self.Avy == 0.0 else ""),
        ]


def compute(shapes: Sequence[Shape]) -> SectionProperties:
    """Composite properties of the shape set (holes as sign=-1 shapes)."""
    if not shapes:
        raise ValueError("compute: no shapes")
    A = sum(s.sign * s.A for s in shapes)
    if A <= 0:
        raise ValueError(f"compute: net area {A} must be positive")
    cx = sum(s.sign * s.A * s.cx for s in shapes) / A
    cy = sum(s.sign * s.A * s.cy for s in shapes) / A
    Ix = Iy = Ixy = 0.0
    for s in shapes:
        dx, dy = s.cx - cx, s.cy - cy
        Ix += s.sign * (s.Ix + s.A * dy * dy)
        Iy += s.sign * (s.Iy + s.A * dx * dx)
        Ixy += s.sign * (s.Ixy + s.A * dx * dy)

    # principal values (Mohr's circle)
    avg = (Ix + Iy) / 2.0
    R = math.hypot((Ix - Iy) / 2.0, Ixy)
    I1, I2 = avg + R, avg - R
    theta_p = 0.5 * math.degrees(math.atan2(-2.0 * Ixy, Ix - Iy)) \
        if R > 1e-14 * max(abs(Ix), abs(Iy), 1.0) else 0.0

    solids = [s for s in shapes if s.sign > 0]
    has_hole = any(s.sign < 0 for s in shapes)
    single = len(shapes) == 1 and shapes[0].sign > 0
    if single:
        s0 = shapes[0]
        J, Avx, Avy = s0.J, s0.Avx, s0.Avy
        approx = not s0.has_exact_J
    else:
        # thin-walled OPEN sum; meaningless for closed cells (any hole) or
        # when a part (polygon) has no J of its own.
        if has_hole or not all(s.J > 0 for s in solids):
            J = 0.0
        else:
            J = sum(s.J for s in solids)
        Avx = Avy = 0.0
        approx = True

    return SectionProperties(
        A=A, cx=cx, cy=cy, Ix=Ix, Iy=Iy, Ixy=Ixy, I1=I1, I2=I2,
        theta_p_deg=theta_p, rx=math.sqrt(Ix / A), ry=math.sqrt(Iy / A),
        J=J, Avx=Avx, Avy=Avy, j_is_approximate=approx)


def to_elastic_section(tag: int, E: float, props: SectionProperties,
                       G: float = 0.0) -> ElasticSection:
    """Build an ElasticSection from computed properties: bending about the
    centroidal x axis (Iz = Ix, the 2D frame convention), Iy = Iy."""
    return ElasticSection(tag, E=E, A=props.A, Iz=props.Ix, Iy=props.Iy,
                          J=props.J, G=G, Avy=props.Avy, Avz=props.Avx)
