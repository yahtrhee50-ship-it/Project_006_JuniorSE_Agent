"""
Section — cross-section resultant behavior (generalized strain -> stress
resultants). Phase 0 needs only the elastic container consumed by the
Phase-1 frame element; fiber sections arrive in Phase 5.
"""
from __future__ import annotations

from abc import ABC


class Section(ABC):
    def __init__(self, tag: int) -> None:
        self.tag = int(tag)


class ElasticSection(Section):
    """Elastic section rigidities for a (2D or 3D) frame member.

    E, A, Iz are required. Iy, J, G matter only for 3D; shear areas (Avy,
    Avz) are stored for the later Timoshenko option and unused by
    Euler-Bernoulli elements.
    """

    def __init__(self, tag: int, E: float, A: float, Iz: float,
                 Iy: float = 0.0, J: float = 0.0, G: float = 0.0,
                 Avy: float = 0.0, Avz: float = 0.0) -> None:
        super().__init__(tag)
        self.E = float(E)
        self.A = float(A)
        self.Iz = float(Iz)
        self.Iy = float(Iy)
        self.J = float(J)
        self.G = float(G)
        self.Avy = float(Avy)
        self.Avz = float(Avz)

    @property
    def EA(self) -> float:
        return self.E * self.A

    @property
    def EIz(self) -> float:
        return self.E * self.Iz

    @property
    def EIy(self) -> float:
        return self.E * self.Iy

    @property
    def GJ(self) -> float:
        return self.G * self.J
