"""
UniaxialMaterial — one of the four core contracts of the engine.

strain in -> stress + tangent out, with committable state. The tangent must
be the true derivative of the returned stress (finite-difference verified).
"""
from __future__ import annotations

from abc import ABC, abstractmethod


class UniaxialMaterial(ABC):
    def __init__(self, tag: int) -> None:
        self.tag = int(tag)

    @abstractmethod
    def set_trial_strain(self, strain: float) -> None: ...

    @abstractmethod
    def get_stress(self) -> float: ...

    @abstractmethod
    def get_tangent(self) -> float: ...

    @abstractmethod
    def commit_state(self) -> None: ...

    @abstractmethod
    def revert_to_last_commit(self) -> None: ...


class ElasticMaterial(UniaxialMaterial):
    """Linear elastic: stress = E * strain."""

    def __init__(self, tag: int, E: float) -> None:
        super().__init__(tag)
        self.E = float(E)
        self._trial_strain = 0.0
        self._committed_strain = 0.0

    def set_trial_strain(self, strain: float) -> None:
        self._trial_strain = float(strain)

    def get_stress(self) -> float:
        return self.E * self._trial_strain

    def get_tangent(self) -> float:
        return self.E

    def commit_state(self) -> None:
        self._committed_strain = self._trial_strain

    def revert_to_last_commit(self) -> None:
        self._trial_strain = self._committed_strain
