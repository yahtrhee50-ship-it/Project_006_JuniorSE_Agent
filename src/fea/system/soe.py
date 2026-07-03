"""
SystemOfEqn + Solver — thin interfaces over the numerical backends (guiding
principle #4: wrap, don't rewrite).

Phase 0 assembles into a dense numpy matrix; DenseSolver needs numpy only,
SparseSolver lazily imports scipy. True sparse *assembly* (and CHOLMOD/
MUMPS/PARDISO backends) is a Phase-2 swap behind these same interfaces.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class Solver(ABC):
    @abstractmethod
    def solve(self, A: np.ndarray, b: np.ndarray) -> np.ndarray: ...


class DenseSolver(Solver):
    def solve(self, A: np.ndarray, b: np.ndarray) -> np.ndarray:
        try:
            return np.linalg.solve(A, b)
        except np.linalg.LinAlgError as exc:
            raise ValueError(
                "Singular stiffness matrix — the model is likely a mechanism "
                f"(unrestrained rigid-body mode or unstable arrangement): {exc}"
            ) from exc


class SparseSolver(Solver):
    """scipy.sparse direct solve. Import is deferred so the engine runs on
    numpy alone when scipy is absent."""

    def solve(self, A: np.ndarray, b: np.ndarray) -> np.ndarray:
        from scipy.sparse import csr_matrix
        from scipy.sparse.linalg import spsolve
        x = spsolve(csr_matrix(A), b)
        if np.any(~np.isfinite(x)):
            raise ValueError(
                "Singular stiffness matrix — the model is likely a mechanism")
        return np.asarray(x)


class LinearSOE:
    """Holds K (A) and the RHS (b) for the free DOFs; scatter targets with
    negative equation numbers (constrained DOFs) are ignored."""

    def __init__(self, solver: Solver | None = None) -> None:
        self.solver = solver if solver is not None else DenseSolver()
        self.A = np.zeros((0, 0))
        self.b = np.zeros(0)

    def set_size(self, n: int) -> None:
        self.A = np.zeros((n, n))
        self.b = np.zeros(n)

    @property
    def size(self) -> int:
        return self.b.shape[0]

    def zero_A(self) -> None:
        self.A[:] = 0.0

    def zero_b(self) -> None:
        self.b[:] = 0.0

    def add_matrix(self, ke: np.ndarray, ids: np.ndarray) -> None:
        ke = np.asarray(ke, dtype=float)
        if ke.shape != (len(ids), len(ids)):
            raise ValueError(
                f"Element matrix shape {ke.shape} does not match "
                f"{len(ids)} DOFs")
        free = np.flatnonzero(ids >= 0)
        if free.size:
            rows = ids[free]
            self.A[np.ix_(rows, rows)] += ke[np.ix_(free, free)]

    def add_vector(self, fe: np.ndarray, ids: np.ndarray) -> None:
        fe = np.asarray(fe, dtype=float)
        if fe.shape != (len(ids),):
            raise ValueError(
                f"Element vector shape {fe.shape} does not match "
                f"{len(ids)} DOFs")
        free = np.flatnonzero(ids >= 0)
        if free.size:
            np.add.at(self.b, ids[free], fe[free])

    def solve(self) -> np.ndarray:
        return self.solver.solve(self.A, self.b)
