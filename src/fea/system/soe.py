"""
SystemOfEqn + Solver — thin interfaces over the numerical backends (guiding
principle #4: wrap, don't rewrite).

LinearSOE owns TWO storage modes behind one interface (Phase 2 Step 7):

- dense  : numpy (n, n) array + np.linalg solve — the no-scipy fallback and
           the default for small systems.
- sparse : COO triplet accumulation -> CSR (scipy), factorized once via
           SuperLU (``splu``, default) or CHOLMOD (scikit-sparse, optional
           lazy import — skipped cleanly when absent).

storage="auto" picks sparse when the equation count reaches
SPARSE_AUTO_THRESHOLD and scipy is importable. ``factorize()`` /
``solve_rhs(b)`` form the multi-RHS path used by ``Model.analyze_cases``
(assemble + factor K once, one back-substitution per load case) — the same
machinery Phase 3 influence lines build on.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

#: auto-storage switchover: systems with at least this many free equations
#: assemble sparse (when scipy is available).
SPARSE_AUTO_THRESHOLD = 500

_SINGULAR_MSG = ("Singular stiffness matrix — the model is likely a mechanism "
                 "(unrestrained rigid-body mode or unstable arrangement)")


def _has_scipy() -> bool:
    try:
        import scipy.sparse  # noqa: F401
        return True
    except ImportError:
        return False


def has_cholmod() -> bool:
    """True when scikit-sparse (CHOLMOD) is importable."""
    try:
        from sksparse.cholmod import cholesky  # noqa: F401
        return True
    except ImportError:
        return False


class Solver(ABC):
    @abstractmethod
    def solve(self, A: np.ndarray, b: np.ndarray) -> np.ndarray: ...


class DenseSolver(Solver):
    def solve(self, A: np.ndarray, b: np.ndarray) -> np.ndarray:
        try:
            return np.linalg.solve(A, b)
        except np.linalg.LinAlgError as exc:
            raise ValueError(f"{_SINGULAR_MSG}: {exc}") from exc


class SparseSolver(Solver):
    """Legacy dense-in/sparse-solve backend (kept for direct users; the
    sparse *assembly* path below supersedes it). Import is deferred so the
    engine runs on numpy alone when scipy is absent."""

    def solve(self, A: np.ndarray, b: np.ndarray) -> np.ndarray:
        from scipy.sparse import csr_matrix
        from scipy.sparse.linalg import spsolve
        x = spsolve(csr_matrix(A), b)
        if np.any(~np.isfinite(x)):
            raise ValueError(_SINGULAR_MSG)
        return np.asarray(x)


class LinearSOE:
    """Holds K (A) and the RHS (b) for the free DOFs; scatter targets with
    negative equation numbers (constrained DOFs) are ignored.

    Parameters
    ----------
    solver : dense-mode backend (DenseSolver default). Ignored in sparse mode.
    storage : "auto" (default) | "dense" | "sparse". Auto goes sparse at
        ``sparse_threshold`` equations when scipy is importable.
    sparse_threshold : override for SPARSE_AUTO_THRESHOLD.
    backend : sparse factorization — "splu" (scipy SuperLU, default) or
        "cholmod" (scikit-sparse; SPD systems, optional dependency).
    """

    def __init__(self, solver: Solver | None = None, *,
                 storage: str = "auto",
                 sparse_threshold: int | None = None,
                 backend: str = "splu") -> None:
        if storage not in ("auto", "dense", "sparse"):
            raise ValueError(f"Unknown storage '{storage}'")
        if backend not in ("splu", "cholmod"):
            raise ValueError(f"Unknown sparse backend '{backend}'")
        self.solver = solver if solver is not None else DenseSolver()
        self._storage = storage
        self._threshold = (SPARSE_AUTO_THRESHOLD if sparse_threshold is None
                           else int(sparse_threshold))
        self._backend = backend
        self._n = 0
        self._mode = "dense"
        self._A: np.ndarray | None = np.zeros((0, 0))
        self._rows: list = []
        self._cols: list = []
        self._vals: list = []
        self._csr = None        # cached assembled CSR (sparse mode)
        self._factor = None     # cached factorization: (kind, object)
        self.b = np.zeros(0)

    def set_size(self, n: int) -> None:
        self._n = int(n)
        self.b = np.zeros(n)
        self._csr = None
        self._factor = None
        use_sparse = (self._storage == "sparse"
                      or (self._storage == "auto"
                          and n >= self._threshold and _has_scipy()))
        if use_sparse and not _has_scipy():
            raise ImportError("storage='sparse' requires scipy")
        if use_sparse:
            self._mode = "sparse"
            self._A = None
            self._rows, self._cols, self._vals = [], [], []
        else:
            self._mode = "dense"
            self._A = np.zeros((n, n))

    @property
    def size(self) -> int:
        return self.b.shape[0]

    @property
    def is_sparse(self) -> bool:
        return self._mode == "sparse"

    @property
    def A(self):
        """Dense ndarray (dense mode) or the assembled scipy CSR matrix
        (sparse mode — duplicates summed)."""
        if self._mode == "dense":
            return self._A
        return self._assemble()

    def zero_A(self) -> None:
        if self._mode == "dense":
            self._A[:] = 0.0
        else:
            self._rows.clear()
            self._cols.clear()
            self._vals.clear()
            self._csr = None
        self._factor = None

    def zero_b(self) -> None:
        self.b[:] = 0.0

    def add_matrix(self, ke: np.ndarray, ids: np.ndarray) -> None:
        ke = np.asarray(ke, dtype=float)
        if ke.shape != (len(ids), len(ids)):
            raise ValueError(
                f"Element matrix shape {ke.shape} does not match "
                f"{len(ids)} DOFs")
        free = np.flatnonzero(ids >= 0)
        if not free.size:
            return
        rows = np.asarray(ids)[free]
        sub = ke[np.ix_(free, free)]
        if self._mode == "dense":
            self._A[np.ix_(rows, rows)] += sub
        else:
            self._rows.append(np.repeat(rows, rows.size))
            self._cols.append(np.tile(rows, rows.size))
            self._vals.append(sub.ravel())
            self._csr = None
        self._factor = None

    def add_vector(self, fe: np.ndarray, ids: np.ndarray) -> None:
        fe = np.asarray(fe, dtype=float)
        if fe.shape != (len(ids),):
            raise ValueError(
                f"Element vector shape {fe.shape} does not match "
                f"{len(ids)} DOFs")
        free = np.flatnonzero(ids >= 0)
        if free.size:
            np.add.at(self.b, np.asarray(ids)[free], fe[free])

    # -- solve paths ---------------------------------------------------------

    def _assemble(self):
        from scipy.sparse import coo_matrix
        if self._csr is None:
            if self._rows:
                r = np.concatenate(self._rows)
                c = np.concatenate(self._cols)
                v = np.concatenate(self._vals)
            else:
                r = np.zeros(0, dtype=int)
                c = np.zeros(0, dtype=int)
                v = np.zeros(0)
            self._csr = coo_matrix(
                (v, (r, c)), shape=(self._n, self._n)).tocsr()
        return self._csr

    def factorize(self) -> None:
        """Factor the current K once; solve_rhs() then back-substitutes per
        RHS. Invalidated by set_size/zero_A/add_matrix."""
        if self._mode == "sparse":
            A = self._assemble().tocsc()
            if self._backend == "cholmod":
                from sksparse.cholmod import cholesky
                try:
                    self._factor = ("cholmod", cholesky(A))
                except Exception as exc:
                    raise ValueError(f"{_SINGULAR_MSG}: {exc}") from exc
            else:
                from scipy.sparse.linalg import splu
                try:
                    self._factor = ("splu", splu(A))
                except RuntimeError as exc:
                    raise ValueError(f"{_SINGULAR_MSG}: {exc}") from exc
        else:
            try:
                from scipy.linalg import lu_factor
                self._factor = ("lu", lu_factor(self._A))
            except ImportError:
                # no scipy: keep a frozen copy, re-solve per RHS (correct,
                # just no factorize-once speedup)
                self._factor = ("dense-copy", self._A.copy())

    def solve_rhs(self, b: np.ndarray) -> np.ndarray:
        """x = K^-1 b through the cached factorization (factorizes first if
        needed). b is not modified."""
        if self._factor is None:
            self.factorize()
        kind, fac = self._factor
        b = np.asarray(b, dtype=float)
        if kind == "cholmod":
            x = fac(b)
        elif kind == "splu":
            x = fac.solve(b)
        elif kind == "lu":
            from scipy.linalg import lu_solve
            x = lu_solve(fac, b)
        else:  # dense-copy fallback
            return self.solver.solve(fac, b)
        x = np.asarray(x, dtype=float).ravel()
        if x.size and not np.all(np.isfinite(x)):
            raise ValueError(_SINGULAR_MSG)
        return x

    def solve(self) -> np.ndarray:
        if self._mode == "sparse":
            return self.solve_rhs(self.b)
        return self.solver.solve(self._A, self.b)
