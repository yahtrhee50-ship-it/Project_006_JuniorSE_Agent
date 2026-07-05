"""
NDMaterial — the multi-dimensional material contract (Phase 2): a strain
VECTOR in engineering (Voigt) notation goes in, a stress vector and a
tangent (D) matrix come out, with committable state. As with
UniaxialMaterial, the tangent must be the true derivative of the returned
stress.

Voigt conventions used engine-wide:
    plane_stress / plane_strain : [eps_xx, eps_yy, gamma_xy]
                                  -> [sig_xx, sig_yy, tau_xy]
    solid3d                     : [eps_xx, eps_yy, eps_zz,
                                   gamma_xy, gamma_yz, gamma_zx]

Elements hold one material COPY per integration point (`copy()`), so
history-dependent materials (Phase 5) drop in without element changes.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class NDMaterial(ABC):
    """Base class: n_strain-sized trial strain -> stress + tangent."""

    def __init__(self, tag: int, n_strain: int) -> None:
        self.tag = int(tag)
        self.n_strain = int(n_strain)

    @abstractmethod
    def set_trial_strain(self, strain) -> None: ...

    @abstractmethod
    def get_stress(self) -> np.ndarray: ...

    @abstractmethod
    def get_tangent(self) -> np.ndarray: ...

    @abstractmethod
    def commit_state(self) -> None: ...

    @abstractmethod
    def revert_to_last_commit(self) -> None: ...

    @abstractmethod
    def copy(self) -> "NDMaterial":
        """Fresh instance with the same parameters and virgin state (one
        per integration point)."""


class ElasticIsotropic(NDMaterial):
    """Linear elastic isotropic: stress = D @ strain.

    formulation: 'plane_stress' (default — thin membranes, the Quad4/LE1
    case), 'plane_strain', or 'solid3d'. Plane-strain out-of-plane sig_zz
    and axisymmetric formulations are deferred until an element needs them.
    """

    _PLANE = ("plane_stress", "plane_strain")

    def __init__(self, tag: int, E: float, nu: float,
                 formulation: str = "plane_stress") -> None:
        if formulation in self._PLANE:
            n_strain = 3
        elif formulation == "solid3d":
            n_strain = 6
        else:
            raise ValueError(
                f"ElasticIsotropic {tag}: unknown formulation "
                f"'{formulation}' (use plane_stress, plane_strain, solid3d)")
        super().__init__(tag, n_strain)
        self.E = float(E)
        self.nu = float(nu)
        self.formulation = formulation
        self._D = self._build_D()
        self._trial = np.zeros(self.n_strain)
        self._committed = np.zeros(self.n_strain)

    def _build_D(self) -> np.ndarray:
        E, nu = self.E, self.nu
        if self.formulation == "plane_stress":
            c = E / (1.0 - nu ** 2)
            return c * np.array([
                [1.0, nu, 0.0],
                [nu, 1.0, 0.0],
                [0.0, 0.0, (1.0 - nu) / 2.0],
            ])
        if self.formulation == "plane_strain":
            c = E / ((1.0 + nu) * (1.0 - 2.0 * nu))
            return c * np.array([
                [1.0 - nu, nu, 0.0],
                [nu, 1.0 - nu, 0.0],
                [0.0, 0.0, (1.0 - 2.0 * nu) / 2.0],
            ])
        # solid3d
        c = E / ((1.0 + nu) * (1.0 - 2.0 * nu))
        g = (1.0 - 2.0 * nu) / 2.0
        D = np.zeros((6, 6))
        D[:3, :3] = nu
        np.fill_diagonal(D[:3, :3], 1.0 - nu)
        D[3, 3] = D[4, 4] = D[5, 5] = g
        return c * D

    def set_trial_strain(self, strain) -> None:
        strain = np.asarray(strain, dtype=float)
        if strain.shape != (self.n_strain,):
            raise ValueError(
                f"ElasticIsotropic {self.tag}: expected {self.n_strain} "
                f"strain components, got shape {strain.shape}")
        self._trial = strain.copy()

    def get_stress(self) -> np.ndarray:
        return self._D @ self._trial

    def get_tangent(self) -> np.ndarray:
        return self._D.copy()

    def commit_state(self) -> None:
        self._committed = self._trial.copy()

    def revert_to_last_commit(self) -> None:
        self._trial = self._committed.copy()

    def copy(self) -> "ElasticIsotropic":
        return ElasticIsotropic(self.tag, self.E, self.nu, self.formulation)
