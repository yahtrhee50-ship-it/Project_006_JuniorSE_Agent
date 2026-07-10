"""
Multi-point constraints and the TransformationHandler (Phase 2, step 1).

An MP_Constraint slaves DOFs of one node to DOFs of another through a linear
relation in GLOBAL components:

    u_c[constrained_dofs] = C @ u_r[retained_dofs]

Enforcement is the transformation method (research brief §6, production
choice): slaved DOFs get no equation; every node's global displacement
components resolve to a linear form  u = g + T @ u_hat  over the free
equations, and AnalysisModel assembles K_hat = T_e^T K_e T_e element by
element. Exact, no conditioning penalty.

v1 hygiene rules (fail loudly, no silent workarounds):
- a DOF may not be slaved twice, nor slaved and SP-constrained;
- a retained node may not itself be slaved anywhere (no chains);
- a constrained (slaved) node may not carry skew nodal axes
  (a retained node MAY be skewed — resolution composes through it).
"""
from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np

from src.fea.analysis_model import AnalysisModel
from src.fea.constraints.sp import ConstraintHandler, collect_sp_values
from src.fea.domain import Domain
from src.fea.numberer import DOF_Numberer


class MP_Constraint:
    def __init__(self, retained_tag: int, constrained_tag: int,
                 constrained_dofs: Sequence[int],
                 retained_dofs: Sequence[int],
                 matrix) -> None:
        self.retained_tag = int(retained_tag)
        self.constrained_tag = int(constrained_tag)
        self.constrained_dofs: Tuple[int, ...] = tuple(
            int(d) for d in constrained_dofs)
        self.retained_dofs: Tuple[int, ...] = tuple(
            int(d) for d in retained_dofs)
        self.matrix = np.asarray(matrix, dtype=float)
        if self.retained_tag == self.constrained_tag:
            raise ValueError("MP constraint: retained and constrained node "
                             "must differ")
        if self.matrix.shape != (len(self.constrained_dofs),
                                 len(self.retained_dofs)):
            raise ValueError(
                f"MP constraint matrix shape {self.matrix.shape} does not "
                f"match {len(self.constrained_dofs)} constrained x "
                f"{len(self.retained_dofs)} retained dofs")

    @property
    def terms(self) -> Tuple[Tuple[int, Tuple[int, ...], np.ndarray], ...]:
        """(retained_tag, retained_dofs, matrix) triples — the general
        multi-retained form AnalysisModel resolves against."""
        return ((self.retained_tag, self.retained_dofs, self.matrix),)


class MP_ConstraintMulti:
    """Linear MP over SEVERAL retained nodes:

        u_c[constrained_dofs] = sum_k  C_k @ u_rk[retained_dofs_k]

    (the single-retained MP_Constraint is the one-term special case). The
    canonical use is a mesh hanging node slaved to linear interpolation of
    the two end nodes of the coarse edge it sits on (Step 3 edge stitching).
    """

    def __init__(self, constrained_tag: int,
                 constrained_dofs: Sequence[int],
                 terms: Sequence[Tuple[int, Sequence[int], "np.ndarray"]]
                 ) -> None:
        self.constrained_tag = int(constrained_tag)
        self.constrained_dofs: Tuple[int, ...] = tuple(
            int(d) for d in constrained_dofs)
        if not terms:
            raise ValueError("MP multi constraint: give at least one "
                             "(retained_tag, retained_dofs, matrix) term")
        packed = []
        seen = set()
        for rtag, rdofs, mat in terms:
            rtag = int(rtag)
            rdofs = tuple(int(d) for d in rdofs)
            mat = np.asarray(mat, dtype=float)
            if rtag == self.constrained_tag:
                raise ValueError("MP multi constraint: retained and "
                                 "constrained node must differ")
            if rtag in seen:
                raise ValueError(
                    f"MP multi constraint: retained node {rtag} appears in "
                    f"more than one term — merge the matrices")
            seen.add(rtag)
            if mat.shape != (len(self.constrained_dofs), len(rdofs)):
                raise ValueError(
                    f"MP multi constraint matrix shape {mat.shape} does not "
                    f"match {len(self.constrained_dofs)} constrained x "
                    f"{len(rdofs)} retained dofs")
            packed.append((rtag, rdofs, mat))
        self._terms: Tuple = tuple(packed)

    @property
    def terms(self) -> Tuple[Tuple[int, Tuple[int, ...], np.ndarray], ...]:
        return self._terms


def hanging_node(retained_a: int, retained_b: int, constrained_tag: int,
                 dofs: Sequence[int], s: float) -> MP_ConstraintMulti:
    """Hanging node at parameter s in [0, 1] along the straight edge from
    retained node a (s=0) to b (s=1):  u_c = (1-s) u_a + s u_b per dof.
    Exact interface compatibility for elements with linear edges (Q4)."""
    if not 0.0 <= s <= 1.0:
        raise ValueError(f"hanging_node: s={s} must be in [0, 1]")
    dofs = tuple(dofs)
    eye = np.eye(len(dofs))
    return MP_ConstraintMulti(constrained_tag, dofs,
                              [(retained_a, dofs, (1.0 - s) * eye),
                               (retained_b, dofs, s * eye)])


def equal_dof(retained_tag: int, constrained_tag: int,
              dofs: Sequence[int]) -> MP_Constraint:
    """u_c[dof] = u_r[dof] for each dof (identity coupling)."""
    dofs = tuple(dofs)
    return MP_Constraint(retained_tag, constrained_tag, dofs, dofs,
                         np.eye(len(dofs)))


def rigid_link(kind: str, retained: "Node", constrained: "Node"
               ) -> MP_Constraint:
    """2D rigid link between two 3-DOF (ux, uy, rz) frame nodes.

    kind='beam': translations AND rotation slaved (fully rigid connection);
    kind='bar' : translations slaved through the retained node's rotation,
                 constrained node's own rotation stays free.

        u_cx = u_rx - dy * rz_r
        u_cy = u_ry + dx * rz_r      (d = x_c - x_r, small rotations)
    """
    if kind not in ("beam", "bar"):
        raise ValueError("rigid_link kind must be 'beam' or 'bar'")
    if retained.ndf != 3 or constrained.ndf != 3 or retained.ndm != 2:
        raise NotImplementedError(
            "rigid_link v1 supports 2D 3-DOF (ux, uy, rz) nodes; the 3D "
            "6-DOF version arrives with the 3D frame (Phase 2 step 4)")
    dx, dy = constrained.coords - retained.coords
    if kind == "beam":
        C = np.array([[1.0, 0.0, -dy],
                      [0.0, 1.0, dx],
                      [0.0, 0.0, 1.0]])
        return MP_Constraint(retained.tag, constrained.tag,
                             (0, 1, 2), (0, 1, 2), C)
    C = np.array([[1.0, 0.0, -dy],
                  [0.0, 1.0, dx]])
    return MP_Constraint(retained.tag, constrained.tag,
                         (0, 1), (0, 1, 2), C)


class TransformationHandler(ConstraintHandler):
    """Handles SP constraints by elimination and MP constraints (and skew
    nodal axes) by the transformation method."""

    def handle(self, domain: Domain, model: AnalysisModel,
               numberer: DOF_Numberer) -> None:
        constrained = collect_sp_values(domain)
        model.build(domain, constrained, numberer.node_order(domain),
                    mps=list(domain.mp_constraints()))
