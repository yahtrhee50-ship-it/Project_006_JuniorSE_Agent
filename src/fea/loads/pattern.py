"""
Load patterns: a TimeSeries (load factor vs pseudo-time) applied to a set of
loads. In static analysis, pseudo-time is the integrator's load factor
lambda; a Linear series then gives proportional loading.

Element loads (distributed, point-on-member, thermal) arrive in Phase 1 as
equivalent nodal loads computed by the element.
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


class LoadPattern:
    def __init__(self, tag: int, time_series: TimeSeries) -> None:
        self.tag = int(tag)
        self.time_series = time_series
        self.nodal_loads: List[NodalLoad] = []

    def add_nodal_load(self, load: NodalLoad) -> None:
        self.nodal_loads.append(load)

    def apply(self, model, soe, time: float) -> None:
        """Add factor * reference loads into the RHS at free equations."""
        factor = self.time_series.get_factor(time)
        for load in self.nodal_loads:
            soe.add_vector(factor * load.values,
                           model.eq_numbers(load.node_tag))
