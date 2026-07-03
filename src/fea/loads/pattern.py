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
            soe.add_vector(factor * load.values,
                           model.eq_numbers(load.node_tag))
        for eload in self.element_loads:
            domain.get_element(eload.ele_tag).add_load(eload, factor)
