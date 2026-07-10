"""
Load patterns: a TimeSeries (load factor vs pseudo-time) applied to a set of
loads. In static analysis, pseudo-time is the integrator's load factor
lambda; a Linear series then gives proportional loading.

Element loads are interpreted by the element itself (OpenSees eleLoad
style): the pattern hands each loaded element its factored load data, and
the element folds it into its resisting force. The equivalent nodal loads
then reach the residual through -F_resisting, and reactions through the
internal-force side of internal - external — no separate RHS scatter.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

import numpy as np


class TimeSeries(ABC):
    @abstractmethod
    def get_factor(self, time: float) -> float: ...


class ConstantTimeSeries(TimeSeries):
    def __init__(self, factor: float = 1.0) -> None:
        self.factor = float(factor)

    def get_factor(self, time: float) -> float:
        return self.factor


class LinearTimeSeries(TimeSeries):
    def __init__(self, slope: float = 1.0) -> None:
        self.slope = float(slope)

    def get_factor(self, time: float) -> float:
        return self.slope * time


class NodalLoad:
    """Reference load on one node: full ndf-length vector of force values."""

    def __init__(self, node_tag: int, values) -> None:
        self.node_tag = int(node_tag)
        self.values = np.asarray(values, dtype=float)


class ElementLoad:
    """Reference load on one element; the element interprets the data."""

    def __init__(self, ele_tag: int) -> None:
        self.ele_tag = int(ele_tag)


class TrussStrainLoad(ElementLoad):
    """Initial (stress-free) axial strain on a Truss: fabrication misfit
    delta_L0 (+ = fabricated too long) and/or thermal alpha * delta_T.
    The element converts delta_L0 to strain with its own length."""

    def __init__(self, ele_tag: int, delta_L0: float = 0.0,
                 delta_T: float = 0.0, alpha: float = 0.0) -> None:
        super().__init__(ele_tag)
        self.delta_L0 = float(delta_L0)
        self.delta_T = float(delta_T)
        self.alpha = float(alpha)


class FrameUniformLoad(ElementLoad):
    """Uniform load per unit length (wx, wy) in GLOBAL directions, over the
    full length of a Frame element. The element projects these onto its own
    local axial/transverse directions using its direction cosines."""

    def __init__(self, ele_tag: int, wx: float = 0.0, wy: float = 0.0) -> None:
        super().__init__(ele_tag)
        self.wx = float(wx)
        self.wy = float(wy)


class FramePointLoad(ElementLoad):
    """Point load (Px, Py) in GLOBAL directions at distance_from_i along a
    Frame element's axis. The element projects these onto its own local
    axial/transverse directions using its direction cosines."""

    def __init__(self, ele_tag: int, Px: float = 0.0, Py: float = 0.0,
                 distance_from_i: float = 0.0) -> None:
        super().__init__(ele_tag)
        self.Px = float(Px)
        self.Py = float(Py)
        self.distance_from_i = float(distance_from_i)


class FrameUniformLoad3D(ElementLoad):
    """Uniform load per unit length (wx, wy, wz) in GLOBAL directions, over
    the full length of a Frame3D element. The element projects these onto
    its own local axial/transverse directions."""

    def __init__(self, ele_tag: int, wx: float = 0.0, wy: float = 0.0,
                 wz: float = 0.0) -> None:
        super().__init__(ele_tag)
        self.wx = float(wx)
        self.wy = float(wy)
        self.wz = float(wz)


class FramePointLoad3D(ElementLoad):
    """Point load (Px, Py, Pz) in GLOBAL directions at distance_from_i along
    a Frame3D element's axis. The element projects these onto its own local
    axial/transverse directions."""

    def __init__(self, ele_tag: int, Px: float = 0.0, Py: float = 0.0,
                 Pz: float = 0.0, distance_from_i: float = 0.0) -> None:
        super().__init__(ele_tag)
        self.Px = float(Px)
        self.Py = float(Py)
        self.Pz = float(Pz)
        self.distance_from_i = float(distance_from_i)


class QuadBodyLoad(ElementLoad):
    """Body force (bx, by) per unit VOLUME in GLOBAL directions on a 2D
    continuum element (e.g. self-weight by = -unit_weight). The element
    converts it to consistent nodal loads with its own shape functions and
    thickness."""

    def __init__(self, ele_tag: int, bx: float = 0.0, by: float = 0.0) -> None:
        super().__init__(ele_tag)
        self.bx = float(bx)
        self.by = float(by)


class QuadEdgeLoad(ElementLoad):
    """Constant traction per unit AREA on one edge of a 2D continuum
    element. edge = local edge index (edge k runs local node k -> k+1,
    CCW). Components: (tx, ty) in GLOBAL directions, plus `pressure`
    along the edge's outward normal with POSITIVE = pushing INWARD on the
    element (classic pressure); use a negative pressure for an outward
    (tension) load such as NAFEMS LE1's outer-edge loading."""

    def __init__(self, ele_tag: int, edge: int, tx: float = 0.0,
                 ty: float = 0.0, pressure: float = 0.0) -> None:
        super().__init__(ele_tag)
        self.edge = int(edge)
        self.tx = float(tx)
        self.ty = float(ty)
        self.pressure = float(pressure)


class ShellPressureLoad(ElementLoad):
    """Uniform pressure per unit AREA on a shell element, acting along the
    element's LOCAL +z normal (CCW node order defines +z). Use a negative
    p for load against the normal (e.g. gravity on a floor whose +z is up)."""

    def __init__(self, ele_tag: int, p: float) -> None:
        super().__init__(ele_tag)
        self.p = float(p)


class ShellBodyLoad(ElementLoad):
    """Body force (bx, by, bz) per unit VOLUME in GLOBAL directions on a
    shell element (self-weight: bz = -unit_weight). The element multiplies
    by its thickness and integrates over the mid-surface."""

    def __init__(self, ele_tag: int, bx: float = 0.0, by: float = 0.0,
                 bz: float = 0.0) -> None:
        super().__init__(ele_tag)
        self.bx = float(bx)
        self.by = float(by)
        self.bz = float(bz)


class LoadPattern:
    def __init__(self, tag: int, time_series: TimeSeries) -> None:
        self.tag = int(tag)
        self.time_series = time_series
        self.nodal_loads: List[NodalLoad] = []
        self.element_loads: List[ElementLoad] = []

    def add_nodal_load(self, load: NodalLoad) -> None:
        self.nodal_loads.append(load)

    def add_element_load(self, load: ElementLoad) -> None:
        self.element_loads.append(load)

    def apply(self, domain, model, soe, time: float) -> None:
        """Add factor * nodal loads into the RHS at free equations, and hand
        factor * element loads to their elements (elements must have been
        zeroed via zero_loads() before any pattern applies)."""
        factor = self.time_series.get_factor(time)
        for load in self.nodal_loads:
            model.add_nodal_load_to_rhs(soe, load.node_tag,
                                        factor * load.values)
        for eload in self.element_loads:
            domain.get_element(eload.ele_tag).add_load(eload, factor)
