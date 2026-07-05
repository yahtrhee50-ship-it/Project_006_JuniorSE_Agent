"""
Quad4 — 4-node bilinear isoparametric quadrilateral for 2D continuum
(plane stress / plane strain via the NDMaterial's D-matrix), 2 DOF/node
(ux, uy), full 2x2 Gauss integration, uniform thickness t.

Node order must be CCW (a non-positive Jacobian at any Gauss point raises
at set_domain — the Step 3 mesher's quality gate enforces the same rule
before elements are ever built).

Element loads (QuadBodyLoad per unit volume, QuadEdgeLoad constant
traction per unit area on a straight edge) become consistent nodal loads
accumulated into `_q0`, following the Frame element's pattern: resisting
force = internal(B^T sigma) - q0, so the equivalent nodal loads reach the
residual and the reactions without a separate RHS scatter, and the
tangent stays the exact derivative of the resisting force (q0 is
state-independent).

Stress recovery: sigma at the 2x2 Gauss points (where it is most
accurate), plus bilinear extrapolation to the corner nodes via a fit of
sigma(xi, eta) = c0 + c1*xi + c2*eta + c3*xi*eta through the Gauss values
— the standard Gauss-to-node extrapolation, exact for any bilinear field.

Holds one NDMaterial copy per Gauss point so path-dependent materials
(Phase 5) drop in without element changes.
"""
from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np

from src.fea.elements.element import Element
from src.fea.elements.isoparam import (Q4_CORNERS, b_matrix_2d, gauss_2d,
                                       jacobian, shape_q4)
from src.fea.loads.pattern import QuadBodyLoad, QuadEdgeLoad
from src.fea.materials.nd import NDMaterial

# Edge k runs local node k -> node (k+1) % 4 (CCW).
_EDGES = ((0, 1), (1, 2), (2, 3), (3, 0))


class Quad4(Element):
    """2D 4-node quad; material must be a 3-component (plane) NDMaterial."""

    def __init__(self, tag: int, node_tags: Sequence[int],
                 material: NDMaterial, t: float) -> None:
        super().__init__(tag, node_tags)
        if len(self.node_tags) != 4:
            raise ValueError(f"Quad4 {tag}: needs exactly 4 nodes")
        if material.n_strain != 3:
            raise ValueError(
                f"Quad4 {tag}: needs a plane (3-component) NDMaterial, got "
                f"{material.n_strain} components ('{getattr(material, 'formulation', '?')}')")
        self.t = float(t)
        if self.t <= 0.0:
            raise ValueError(f"Quad4 {tag}: thickness must be > 0")
        self._proto_material = material
        self._coords = np.zeros((4, 2))
        # per Gauss point: shape values N, B matrix, w*detJ*t, material copy
        self._gp: List[Tuple[np.ndarray, np.ndarray, float, NDMaterial]] = []
        self._extrap = np.zeros((4, 4))   # Gauss values -> corner values
        self._q0 = np.zeros(8)            # consistent element loads

    def set_domain(self, domain) -> None:
        super().set_domain(domain)
        for n in self.nodes:
            if n.ndm != 2:
                raise ValueError(f"Quad4 {self.tag}: needs 2D (ndm=2) nodes")
        self._coords = np.array([n.coords for n in self.nodes])
        pts, wts = gauss_2d(2)
        self._gp = []
        for (xi, eta), w in zip(pts, wts):
            N, dN = shape_q4(xi, eta)
            _, detJ, dN_dx = jacobian(self._coords, dN)
            if detJ <= 0.0:
                raise ValueError(
                    f"Quad4 {self.tag}: non-positive Jacobian ({detJ:.3e}) "
                    f"at Gauss point ({xi:.4f}, {eta:.4f}) — element is "
                    f"inverted or its nodes are numbered clockwise")
            self._gp.append((N, b_matrix_2d(dN_dx), w * detJ * self.t,
                             self._proto_material.copy()))
        # Bilinear fit through the Gauss points, evaluated at the corners.
        A_g = np.array([[1.0, xi, eta, xi * eta] for xi, eta in pts])
        A_n = np.array([[1.0, xi, eta, xi * eta] for xi, eta in Q4_CORNERS])
        self._extrap = A_n @ np.linalg.inv(A_g)

    def get_node_dofs(self) -> Sequence[Sequence[int]]:
        return ((0, 1),) * 4

    # -- element loads ------------------------------------------------------

    def zero_loads(self) -> None:
        self._q0 = np.zeros(8)

    def add_load(self, load, factor: float) -> None:
        if isinstance(load, QuadBodyLoad):
            b = factor * np.array([load.bx, load.by])
            for N, _, wdetJt, _ in self._gp:
                for a in range(4):
                    self._q0[2 * a:2 * a + 2] += N[a] * wdetJt * b
        elif isinstance(load, QuadEdgeLoad):
            if not 0 <= load.edge <= 3:
                raise ValueError(
                    f"Quad4 {self.tag}: edge index {load.edge} not in 0..3")
            a, b_ = _EDGES[load.edge]
            ev = self._coords[b_] - self._coords[a]
            L = float(np.linalg.norm(ev))
            if L == 0.0:
                raise ValueError(f"Quad4 {self.tag}: zero-length edge {load.edge}")
            # CCW node order => outward normal = edge tangent rotated -90 deg.
            n_out = np.array([ev[1], -ev[0]]) / L
            trac = factor * (np.array([load.tx, load.ty])
                             - load.pressure * n_out)
            # constant traction on a straight linear edge: half to each node
            F = trac * self.t * L / 2.0
            self._q0[2 * a:2 * a + 2] += F
            self._q0[2 * b_:2 * b_ + 2] += F
        else:
            super().add_load(load, factor)

    # -- state ------------------------------------------------------------

    def _element_disp(self) -> np.ndarray:
        return np.concatenate([n.get_trial_disp()[:2] for n in self.nodes])

    def get_resisting_force(self) -> np.ndarray:
        u = self._element_disp()
        f = -self._q0.copy()
        for _, B, wdetJt, mat in self._gp:
            mat.set_trial_strain(B @ u)
            f += wdetJt * (B.T @ mat.get_stress())
        return f

    def get_tangent_stiff(self) -> np.ndarray:
        u = self._element_disp()
        k = np.zeros((8, 8))
        for _, B, wdetJt, mat in self._gp:
            mat.set_trial_strain(B @ u)
            k += wdetJt * (B.T @ mat.get_tangent() @ B)
        return k

    def commit_state(self) -> None:
        for _, _, _, mat in self._gp:
            mat.commit_state()

    def revert_to_last_commit(self) -> None:
        for _, _, _, mat in self._gp:
            mat.revert_to_last_commit()

    # -- responses ----------------------------------------------------------

    def stress_at_gauss(self) -> np.ndarray:
        """(4, 3) stresses [sxx, syy, txy] at the 2x2 Gauss points (order
        of isoparam.gauss_2d) for the current trial displacements."""
        u = self._element_disp()
        out = np.empty((4, 3))
        for g, (_, B, _, mat) in enumerate(self._gp):
            mat.set_trial_strain(B @ u)
            out[g] = mat.get_stress()
        return out

    def stress_at_nodes(self) -> np.ndarray:
        """(4, 3) stresses extrapolated to the corner nodes (local node
        order) by the bilinear Gauss fit."""
        return self._extrap @ self.stress_at_gauss()

    def get_response(self, name: str):
        if name == "stress_gp":
            return self.stress_at_gauss()
        if name == "stress_nodes":
            return self.stress_at_nodes()
        raise ValueError(f"Quad4 {self.tag}: unknown response '{name}'")
