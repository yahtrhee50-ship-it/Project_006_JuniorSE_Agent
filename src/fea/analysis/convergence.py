"""
ConvergenceTest — decides when a nonlinear iteration has converged.

Not used by the Linear algorithm (one exact solve); Newton-Raphson consumes
these from Phase 4 on. Defined now so the aggregation interface is complete.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from src.fea.system.soe import LinearSOE


class ConvergenceTest(ABC):
    def __init__(self, tol: float = 1e-8, max_iter: int = 25) -> None:
        self.tol = float(tol)
        self.max_iter = int(max_iter)

    def start(self) -> None:
        self._iter = 0

    @abstractmethod
    def test(self, soe: LinearSOE, delta_u: np.ndarray) -> bool:
        """True = converged. Raises RuntimeError past max_iter."""

    def _count(self, norm: float) -> bool:
        self._iter += 1
        if norm <= self.tol:
            return True
        if self._iter >= self.max_iter:
            raise RuntimeError(
                f"{type(self).__name__}: no convergence in {self.max_iter} "
                f"iterations (norm={norm:.3e}, tol={self.tol:.3e})")
        return False


class NormUnbalance(ConvergenceTest):
    def test(self, soe: LinearSOE, delta_u: np.ndarray) -> bool:
        return self._count(float(np.linalg.norm(soe.b)))


class NormDispIncr(ConvergenceTest):
    def test(self, soe: LinearSOE, delta_u: np.ndarray) -> bool:
        return self._count(float(np.linalg.norm(delta_u)))
