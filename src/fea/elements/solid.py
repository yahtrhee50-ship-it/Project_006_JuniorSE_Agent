"""
3D solid elements (Phase 2 Step 6): Hex8 + Tet4.

Hex8 — 8-node trilinear isoparametric hexahedron, 3 DOF/node (ux, uy, uz),
full 2x2x2 Gauss integration, solid3d NDMaterial (6-component Voigt).
Node order matches isoparam.H8_CORNERS / mesh.box_grid connectivity: the
bottom face CCW viewed from +zeta, then the top face in the same pattern.
A non-positive Jacobian at any Gauss point raises at set_domain.

Tet4 — 4-node constant-strain tetrahedron (linear shapes, one-point exact
integration). Node convention: local nodes 0-1-2 ordered CCW when viewed
from node 3 (positive 6V determinant); violated order raises. Documented
STIFF in bending — use Hex8 where accuracy matters (the test suite
quantifies the gap on a shared cantilever problem).

Element loads follow the Quad4 `_q0` pattern (resisting force =
internal - q0, tangent untouched): SolidBodyLoad per unit volume,
SolidFaceLoad constant traction / pressure on one face (Hex8 faces are
bilinear surfaces integrated with their own 2x2 Gauss rule and area
Jacobian, so distorted/non-planar faces are handled consistently; Tet4
faces are flat triangles — A/3 to each face node). Face node orderings
are chosen so the parametric tangent cross product points OUTWARD;
`pressure` is positive pushing INWARD (same convention as QuadEdgeLoad).

Stress recovery: sigma at the integration points, plus (Hex8) trilinear
extrapolation of the 8 Gauss values to the corner nodes — exact for any
trilinear field; Tet4 stress is a single constant state.

Holds one NDMaterial copy per integration point so path-dependent
materials (Phase 5) drop in without element changes.
"""
from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np

from src.fea.elements.element import Element
from src.fea.elements.isoparam import (H8_CORNERS, b_matrix_3d, gauss_2d,
                                       gauss_3d, jacobian, shape_h8, shape_q4)
from src.fea.loads.pattern import SolidBodyLoad, SolidFaceLoad
from src.fea.materials.nd import NDMaterial

# Hex8 faces as local node quads, ordered so shape_q4's parametric
# tangents give t_xi x t_eta = OUTWARD normal.
_HEX_FACES = (
    (0, 3, 2, 1),   # 0: zeta = -1 (bottom)
    (4, 5, 6, 7),   # 1: zeta = +1 (top)
    (0, 1, 5, 4),   # 2: eta  = -1
    (1, 2, 6, 5),   # 3: xi   = +1
    (2, 3, 7, 6),   # 4: eta  = +1
    (3, 0, 4, 7),   # 5: xi   = -1
)

# Tet4 face f = triangle opposite local node f, ordered so
# (n1-n0) x (n2-n0) points OUTWARD.
_TET_FACES = (
    (1, 2, 3),   # 0: opposite node 0
    (0, 3, 2),   # 1: opposite node 1
    (0, 1, 3),   # 2: opposite node 2
    (0, 2, 1),   # 3: opposite node 3
)

# Parent-tet shape derivatives (N1 = 1-xi-eta-zeta, N2 = xi, N3 = eta,
# N4 = zeta) — constant, so one Jacobian gives the whole element.
_TET_DN = np.array([
    [-1.0, -1.0, -1.0],
    [ 1.0,  0.0,  0.0],
    [ 0.0,  1.0,  0.0],
    [ 0.0,  0.0,  1.0],
])


def _check_solid_nodes(elem: Element, n_expected: int) -> np.ndarray:
    if len(elem.nodes) != n_expected:
        raise ValueError(
            f"{type(elem).__name__} {elem.tag}: needs exactly {n_expected} nodes")
    for n in elem.nodes:
        if n.ndm != 3 or n.ndf < 3:
            raise ValueError(
                f"{type(elem).__name__} {elem.tag}: needs 3D nodes with at "
                f"least 3 DOFs (node {n.tag} has ndm={n.ndm}, ndf={n.ndf})")
    return np.array([n.coords for n in elem.nodes])


class Hex8(Element):
    """3D 8-node hex; material must be a solid3d (6-component) NDMaterial."""

    def __init__(self, tag: int, node_tags: Sequence[int],
                 material: NDMaterial) -> None:
        super().__init__(tag, node_tags)
        if len(self.node_tags) != 8:
            raise ValueError(f"Hex8 {tag}: needs exactly 8 nodes")
        if material.n_strain != 6:
            raise ValueError(
                f"Hex8 {tag}: needs a solid3d (6-component) NDMaterial, got "
                f"{material.n_strain} components "
                f"('{getattr(material, 'formulation', '?')}')")
        self._proto_material = material
        self._coords = np.zeros((8, 3))
        # per Gauss point: shape values N, B matrix, w*detJ, material copy
        self._gp: List[Tuple[np.ndarray, np.ndarray, float, NDMaterial]] = []
        self._extrap = np.zeros((8, 8))   # Gauss values -> corner values
        self._q0 = np.zeros(24)           # consistent element loads

    def set_domain(self, domain) -> None:
        super().set_domain(domain)
        self._coords = _check_solid_nodes(self, 8)
        pts, wts = gauss_3d(2)
        self._gp = []
        for (xi, eta, ze), w in zip(pts, wts):
            N, dN = shape_h8(xi, eta, ze)
            _, detJ, dN_dx = jacobian(self._coords, dN)
            if detJ <= 0.0:
                raise ValueError(
                    f"Hex8 {self.tag}: non-positive Jacobian ({detJ:.3e}) "
                    f"at Gauss point ({xi:.4f}, {eta:.4f}, {ze:.4f}) — "
                    f"element is inverted or its nodes are misordered")
            self._gp.append((N, b_matrix_3d(dN_dx), w * detJ,
                             self._proto_material.copy()))
        # Trilinear fit through the 8 Gauss points, evaluated at corners.
        def basis(p):
            xi, eta, ze = p
            return [1.0, xi, eta, ze, xi * eta, eta * ze, ze * xi,
                    xi * eta * ze]
        A_g = np.array([basis(p) for p in pts])
        A_n = np.array([basis(p) for p in H8_CORNERS])
        self._extrap = A_n @ np.linalg.inv(A_g)

    def get_node_dofs(self) -> Sequence[Sequence[int]]:
        return ((0, 1, 2),) * 8

    # -- element loads ------------------------------------------------------

    def zero_loads(self) -> None:
        self._q0 = np.zeros(24)

    def add_load(self, load, factor: float) -> None:
        if isinstance(load, SolidBodyLoad):
            b = factor * np.array([load.bx, load.by, load.bz])
            for N, _, wdetJ, _ in self._gp:
                for a in range(8):
                    self._q0[3 * a:3 * a + 3] += N[a] * wdetJ * b
        elif isinstance(load, SolidFaceLoad):
            if not 0 <= load.face <= 5:
                raise ValueError(
                    f"Hex8 {self.tag}: face index {load.face} not in 0..5")
            fnodes = _HEX_FACES[load.face]
            fcoords = self._coords[list(fnodes)]
            t_glob = factor * np.array([load.tx, load.ty, load.tz])
            pts, wts = gauss_2d(2)
            for (a_, b_), w in zip(pts, wts):
                N, dN = shape_q4(a_, b_)
                t1 = fcoords.T @ dN[:, 0]
                t2 = fcoords.T @ dN[:, 1]
                n_vec = np.cross(t1, t2)
                dA = float(np.linalg.norm(n_vec))
                if dA == 0.0:
                    raise ValueError(
                        f"Hex8 {self.tag}: degenerate face {load.face}")
                trac = t_glob - factor * load.pressure * (n_vec / dA)
                for i, ln in enumerate(fnodes):
                    self._q0[3 * ln:3 * ln + 3] += N[i] * w * dA * trac
        else:
            super().add_load(load, factor)

    # -- state --------------------------------------------------------------

    def _element_disp(self) -> np.ndarray:
        return np.concatenate([n.get_trial_disp()[:3] for n in self.nodes])

    def get_resisting_force(self) -> np.ndarray:
        u = self._element_disp()
        f = -self._q0.copy()
        for _, B, wdetJ, mat in self._gp:
            mat.set_trial_strain(B @ u)
            f += wdetJ * (B.T @ mat.get_stress())
        return f

    def get_tangent_stiff(self) -> np.ndarray:
        u = self._element_disp()
        k = np.zeros((24, 24))
        for _, B, wdetJ, mat in self._gp:
            mat.set_trial_strain(B @ u)
            k += wdetJ * (B.T @ mat.get_tangent() @ B)
        return k

    def commit_state(self) -> None:
        for _, _, _, mat in self._gp:
            mat.commit_state()

    def revert_to_last_commit(self) -> None:
        for _, _, _, mat in self._gp:
            mat.revert_to_last_commit()

    # -- responses ----------------------------------------------------------

    def stress_at_gauss(self) -> np.ndarray:
        """(8, 6) stresses [sxx, syy, szz, txy, tyz, tzx] at the 2x2x2
        Gauss points (order of isoparam.gauss_3d) for the current trial
        displacements."""
        u = self._element_disp()
        out = np.empty((8, 6))
        for g, (_, B, _, mat) in enumerate(self._gp):
            mat.set_trial_strain(B @ u)
            out[g] = mat.get_stress()
        return out

    def stress_at_nodes(self) -> np.ndarray:
        """(8, 6) stresses extrapolated to the corner nodes (local node
        order) by the trilinear Gauss fit."""
        return self._extrap @ self.stress_at_gauss()

    def get_response(self, name: str):
        if name == "stress_gp":
            return self.stress_at_gauss()
        if name == "stress_nodes":
            return self.stress_at_nodes()
        raise ValueError(f"Hex8 {self.tag}: unknown response '{name}'")


class Tet4(Element):
    """3D 4-node constant-strain tet; solid3d (6-component) NDMaterial.

    Known to be overly stiff in bending-dominated problems — prefer Hex8
    (or a finer Tet4 mesh) for anything deflection-critical.
    """

    def __init__(self, tag: int, node_tags: Sequence[int],
                 material: NDMaterial) -> None:
        super().__init__(tag, node_tags)
        if len(self.node_tags) != 4:
            raise ValueError(f"Tet4 {tag}: needs exactly 4 nodes")
        if material.n_strain != 6:
            raise ValueError(
                f"Tet4 {tag}: needs a solid3d (6-component) NDMaterial, got "
                f"{material.n_strain} components "
                f"('{getattr(material, 'formulation', '?')}')")
        self._proto_material = material
        self._coords = np.zeros((4, 3))
        self._B = np.zeros((6, 12))
        self._V = 0.0
        self._mat: NDMaterial = material
        self._q0 = np.zeros(12)

    def set_domain(self, domain) -> None:
        super().set_domain(domain)
        self._coords = _check_solid_nodes(self, 4)
        _, detJ, dN_dx = jacobian(self._coords, _TET_DN)
        if detJ <= 0.0:
            raise ValueError(
                f"Tet4 {self.tag}: non-positive volume ({detJ / 6.0:.3e}) — "
                f"local nodes 0-1-2 must be CCW viewed from node 3")
        self._V = detJ / 6.0
        self._B = b_matrix_3d(dN_dx)
        self._mat = self._proto_material.copy()

    def get_node_dofs(self) -> Sequence[Sequence[int]]:
        return ((0, 1, 2),) * 4

    @property
    def volume(self) -> float:
        return self._V

    # -- element loads ------------------------------------------------------

    def zero_loads(self) -> None:
        self._q0 = np.zeros(12)

    def add_load(self, load, factor: float) -> None:
        if isinstance(load, SolidBodyLoad):
            # Linear shapes: consistent load = V/4 * b at each node (exact).
            b = factor * np.array([load.bx, load.by, load.bz])
            for a in range(4):
                self._q0[3 * a:3 * a + 3] += self._V / 4.0 * b
        elif isinstance(load, SolidFaceLoad):
            if not 0 <= load.face <= 3:
                raise ValueError(
                    f"Tet4 {self.tag}: face index {load.face} not in 0..3")
            fnodes = _TET_FACES[load.face]
            p0, p1, p2 = (self._coords[i] for i in fnodes)
            n_vec = np.cross(p1 - p0, p2 - p0)
            A2 = float(np.linalg.norm(n_vec))    # 2 * area
            if A2 == 0.0:
                raise ValueError(
                    f"Tet4 {self.tag}: degenerate face {load.face}")
            trac = factor * (np.array([load.tx, load.ty, load.tz])
                             - load.pressure * (n_vec / A2))
            # Constant traction on a flat linear triangle: A/3 per node.
            F = trac * (A2 / 2.0) / 3.0
            for ln in fnodes:
                self._q0[3 * ln:3 * ln + 3] += F
        else:
            super().add_load(load, factor)

    # -- state --------------------------------------------------------------

    def _element_disp(self) -> np.ndarray:
        return np.concatenate([n.get_trial_disp()[:3] for n in self.nodes])

    def get_resisting_force(self) -> np.ndarray:
        self._mat.set_trial_strain(self._B @ self._element_disp())
        return self._V * (self._B.T @ self._mat.get_stress()) - self._q0

    def get_tangent_stiff(self) -> np.ndarray:
        self._mat.set_trial_strain(self._B @ self._element_disp())
        return self._V * (self._B.T @ self._mat.get_tangent() @ self._B)

    def commit_state(self) -> None:
        self._mat.commit_state()

    def revert_to_last_commit(self) -> None:
        self._mat.revert_to_last_commit()

    # -- responses ----------------------------------------------------------

    def stress_at_gauss(self) -> np.ndarray:
        """(1, 6) constant stress state for the current trial displacements."""
        self._mat.set_trial_strain(self._B @ self._element_disp())
        return self._mat.get_stress().reshape(1, 6)

    def stress_at_nodes(self) -> np.ndarray:
        """(4, 6) — the constant stress replicated at each node."""
        return np.tile(self.stress_at_gauss(), (4, 1))

    def get_response(self, name: str):
        if name == "stress_gp":
            return self.stress_at_gauss()
        if name == "stress_nodes":
            return self.stress_at_nodes()
        raise ValueError(f"Tet4 {self.tag}: unknown response '{name}'")
