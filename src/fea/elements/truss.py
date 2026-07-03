"""
Truss — linear-kinematics axial element (2D or 3D), stress from a
UniaxialMaterial so the material contract is exercised end to end.

Phase 0 provides this element so the full component stack can be smoke-
tested; Phase 1 validates it against the PlaneTruss solver and the Hibbeler
Ch 14 benchmarks.
"""
from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np

from src.fea.elements.element import Element
from src.fea.loads.pattern import TrussStrainLoad
from src.fea.materials.uniaxial import UniaxialMaterial


class Truss(Element):
    """2-node axial-only element. Small displacements (engineering strain
    measured along the undeformed axis); geometric nonlinearity arrives in
    Phase 4 as a corotational transformation, not a change to this element.
    """

    def __init__(self, tag: int, node_tags: Sequence[int], A: float,
                 material: UniaxialMaterial) -> None:
        super().__init__(tag, node_tags)
        if len(self.node_tags) != 2:
            raise ValueError(f"Truss {tag}: needs exactly 2 nodes")
        self.A = float(A)
        self.material = material
        self.L = 0.0
        self._g = np.zeros(0)   # unit vector i -> j (undeformed)
        self._eps0 = 0.0        # initial (stress-free) strain from element loads

    def set_domain(self, domain) -> None:
        super().set_domain(domain)
        ni, nj = self.nodes
        if ni.ndm != nj.ndm:
            raise ValueError(f"Truss {self.tag}: nodes have different ndm")
        dx = nj.coords - ni.coords
        self.L = float(np.linalg.norm(dx))
        if self.L == 0.0:
            raise ValueError(f"Truss {self.tag}: zero length")
        self._g = dx / self.L

    def get_node_dofs(self) -> Sequence[Sequence[int]]:
        ndm = self.nodes[0].ndm
        trans = tuple(range(ndm))
        return (trans, trans)

    # -- element loads ------------------------------------------------------

    def zero_loads(self) -> None:
        self._eps0 = 0.0

    def add_load(self, load, factor: float) -> None:
        """TrussStrainLoad: initial strain eps0 = delta_L0/L + alpha*dT.
        The material sees the mechanical strain (total - eps0), so the
        equivalent nodal loads (PlaneTruss's F0 = EA*eps0 pushing the ends
        apart) emerge through the resisting force."""
        if not isinstance(load, TrussStrainLoad):
            super().add_load(load, factor)
        self._eps0 += factor * (load.delta_L0 / self.L
                                + load.alpha * load.delta_T)

    # -- state ------------------------------------------------------------

    def _trial_strain(self) -> float:
        """Mechanical strain: total axis strain minus the initial strain."""
        ni, nj = self.nodes
        ndm = ni.ndm
        du = nj.get_trial_disp()[:ndm] - ni.get_trial_disp()[:ndm]
        return float(self._g @ du) / self.L - self._eps0

    def _b_vector(self) -> np.ndarray:
        """d(strain)/d(element dofs): [-g, g] / L."""
        return np.concatenate([-self._g, self._g]) / self.L

    def get_resisting_force(self) -> np.ndarray:
        self.material.set_trial_strain(self._trial_strain())
        N = self.A * self.material.get_stress()
        return N * self.L * self._b_vector()   # = N * [-g, g]

    def get_tangent_stiff(self) -> np.ndarray:
        self.material.set_trial_strain(self._trial_strain())
        b = self._b_vector()
        return self.A * self.material.get_tangent() * self.L * np.outer(b, b)

    def commit_state(self) -> None:
        self.material.commit_state()

    def revert_to_last_commit(self) -> None:
        self.material.revert_to_last_commit()

    # -- responses ----------------------------------------------------------

    def get_axial_force(self) -> float:
        """Axial force, + = tension."""
        self.material.set_trial_strain(self._trial_strain())
        return self.A * self.material.get_stress()

    def get_response(self, name: str):
        if name in ("axial", "force"):
            return self.get_axial_force()
        raise ValueError(f"Truss {self.tag}: unknown response '{name}'")
