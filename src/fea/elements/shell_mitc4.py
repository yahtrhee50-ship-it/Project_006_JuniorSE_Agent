"""
ShellMITC4 — 4-node flat shell, 6 DOF/node (ux, uy, uz, rx, ry, rz):
membrane (Q4 plane stress, full 2x2 integration) + Mindlin-Reissner plate
bending with the MITC4 assumed transverse-shear interpolation (Bathe &
Dvorkin tying at the edge midpoints — the shear-locking fix) + a drilling
DOF stabilized against the membrane's own in-plane rotation.

Local frame (rows of self._R = local axes in global coords): local x along
edge 1-2 projected onto the mean plane, local z = the mean-plane normal
from the cross product of the diagonals (CCW node order => +z), local
y = z x x. Node coordinates are projected onto the mean plane through the
centroid; the out-of-plane offsets are the element's WARP, which only
triggers a warning above `warp_tol` (fraction of the mean diagonal — the
Step 3 mesh quality gate polices the same metric before elements are
built). v1 scope: the analysis itself always uses the flat projection.

Plate sign conventions (local): the rotation vector theta = (rx, ry) moves
a fiber at +z by u = z*ry, v = -z*rx, so the Mindlin section rotations are
phi_x = ry, phi_y = -rx. Curvatures kappa = [phi_x,x, phi_y,y,
phi_x,y + phi_y,x]; moments M = D_b @ kappa with D_b = D_m * t^3/12
(M11 positive = tension on the +z face in local x). Transverse shear
gamma = [w,x + phi_x, w,y + phi_y], V = kappa_s*G*t * gamma (kappa_s =
5/6). Output set at the 2x2 Gauss points mirrors SAP2000:
[F11, F22, F12, M11, M22, M12, V13, V23] (forces/moments per unit width,
local axes).

MITC4 assumed shear: the covariant strains gamma_xi_z / gamma_eta_z are
evaluated at tying points A(0,-1)/C(0,1) and D(-1,0)/B(1,0) respectively
(each with its OWN Jacobian), interpolated linearly in eta / xi, then
pushed to Cartesian with inv(J)^T at the integration point. This passes
the constant-curvature patch test on distorted meshes and keeps the
thin-plate limit lock-free — the element's raison d'etre.

Drilling DOF: rz has no continuum stiffness in a flat shell, so it is
penalized against the membrane rotation omega = (v,x - u,y)/2:
k_drill = drill_alpha * G * t * integral (theta_z_h - omega_h)^2 dA.
A CONSISTENT rigid rotation about the normal (u = -w*y, v = +w*x,
theta_z = w) has zero penalty energy, so the element keeps exactly 6
rigid-body modes — while a theta_z field alone is stiffened at every
node (no spurious drilling zeros in flat assemblies).

Loads (accumulated into local `_q0`, Frame/Quad4 pattern — resisting
force = k_local @ d_local - q0, so equivalent loads reach residual and
reactions without a separate RHS scatter and the tangent stays exact):
ShellPressureLoad (per unit area along the LOCAL +z normal) and
ShellBodyLoad (GLOBAL force per unit volume, e.g. self-weight
bz = -unit_weight); both use the bilinear shape functions (no fixed-end
moments).

v1 is linear elastic: the material must be a plane-stress
ElasticIsotropic (bending D and the transverse-shear G are derived from
its E, nu). Layered/composite shells are Phase 5.
"""
from __future__ import annotations

import warnings
from typing import List, Sequence, Tuple

import numpy as np

from src.fea.elements.element import Element
from src.fea.elements.isoparam import gauss_2d, shape_q4
from src.fea.loads.pattern import ShellBodyLoad, ShellPressureLoad
from src.fea.materials.nd import ElasticIsotropic

# Bathe-Dvorkin tying points on the parent square.
_TIE_A = (0.0, -1.0)   # gamma_xi_z
_TIE_C = (0.0, 1.0)    # gamma_xi_z
_TIE_D = (-1.0, 0.0)   # gamma_eta_z
_TIE_B = (1.0, 0.0)    # gamma_eta_z

_SHEAR_CORRECTION = 5.0 / 6.0


class ShellMITC4(Element):
    """4-node flat shell (membrane + MITC4 Mindlin bending), 6 DOF/node."""

    def __init__(self, tag: int, node_tags: Sequence[int],
                 material: ElasticIsotropic, t: float,
                 drill_alpha: float = 1.0e-3,
                 warp_tol: float = 0.05) -> None:
        super().__init__(tag, node_tags)
        if len(self.node_tags) != 4:
            raise ValueError(f"ShellMITC4 {tag}: needs exactly 4 nodes")
        if (not isinstance(material, ElasticIsotropic)
                or material.formulation != "plane_stress"):
            raise ValueError(
                f"ShellMITC4 {tag}: v1 needs a plane_stress ElasticIsotropic "
                f"material (bending D and shear G derive from its E, nu)")
        self.t = float(t)
        if self.t <= 0.0:
            raise ValueError(f"ShellMITC4 {tag}: thickness must be > 0")
        self.drill_alpha = float(drill_alpha)
        self.warp_tol = float(warp_tol)
        self.material = material
        self._R = np.eye(3)        # rows = local axes in global coords
        self._T = np.zeros((24, 24))
        self._xy = np.zeros((4, 2))  # projected local node coordinates
        self.warp = 0.0
        self._k_local = np.zeros((24, 24))
        # per Gauss point: (N, Bm(3x24), Bb(3x24), Bs(2x24), w*detJ)
        self._gp: List[Tuple[np.ndarray, np.ndarray, np.ndarray,
                             np.ndarray, float]] = []
        self._q0 = np.zeros(24)    # consistent element loads, LOCAL frame

    # -- geometry ------------------------------------------------------------

    def set_domain(self, domain) -> None:
        super().set_domain(domain)
        for n in self.nodes:
            if n.ndm != 3 or n.ndf < 6:
                raise ValueError(
                    f"ShellMITC4 {self.tag}: needs 3D nodes with 6 DOFs")
        X = np.array([n.coords for n in self.nodes])
        c = X.mean(axis=0)
        d13, d24 = X[2] - X[0], X[3] - X[1]
        ez = np.cross(d13, d24)
        nz = np.linalg.norm(ez)
        if nz == 0.0:
            raise ValueError(f"ShellMITC4 {self.tag}: degenerate geometry")
        ez /= nz
        e12 = X[1] - X[0]
        ex = e12 - (e12 @ ez) * ez
        nx = np.linalg.norm(ex)
        if nx == 0.0:
            raise ValueError(
                f"ShellMITC4 {self.tag}: edge 1-2 is parallel to the normal")
        ex /= nx
        ey = np.cross(ez, ex)
        self._R = np.vstack([ex, ey, ez])
        T = np.zeros((24, 24))
        for blk in range(8):
            T[3 * blk:3 * blk + 3, 3 * blk:3 * blk + 3] = self._R
        self._T = T

        local = (X - c) @ self._R.T
        self._xy = local[:, :2]
        diag = 0.5 * (np.linalg.norm(d13) + np.linalg.norm(d24))
        self.warp = float(np.max(np.abs(local[:, 2]))) / diag
        if self.warp > self.warp_tol:
            warnings.warn(
                f"ShellMITC4 {self.tag}: warped geometry (warp {self.warp:.4f}"
                f" > tol {self.warp_tol}) — v1 projects to the mean plane, "
                f"results degrade with warp", stacklevel=2)

        self._build_gauss_data()
        self._build_k_local()

    def _covariant_shear_row(self, xi: float, eta: float
                             ) -> Tuple[np.ndarray, np.ndarray]:
        """Rows mapping the local 24-DOF vector to the covariant transverse
        shears (gamma_xi_z, gamma_eta_z) at one parent point, each with its
        own Jacobian: gamma_xi_z = w,xi + x,xi*phi_x + y,xi*phi_y with
        phi_x = ry, phi_y = -rx."""
        N, dN = shape_q4(xi, eta)
        J = self._xy.T @ dN            # J[i, j] = dx_i/dxi_j
        row_xi = np.zeros(24)
        row_eta = np.zeros(24)
        for a in range(4):
            w, rx, ry = 6 * a + 2, 6 * a + 3, 6 * a + 4
            row_xi[w] = dN[a, 0]
            row_xi[ry] = N[a] * J[0, 0]
            row_xi[rx] = -N[a] * J[1, 0]
            row_eta[w] = dN[a, 1]
            row_eta[ry] = N[a] * J[0, 1]
            row_eta[rx] = -N[a] * J[1, 1]
        return row_xi, row_eta

    def _build_gauss_data(self) -> None:
        row_xi_A, _ = self._covariant_shear_row(*_TIE_A)
        row_xi_C, _ = self._covariant_shear_row(*_TIE_C)
        _, row_eta_D = self._covariant_shear_row(*_TIE_D)
        _, row_eta_B = self._covariant_shear_row(*_TIE_B)

        pts, wts = gauss_2d(2)
        self._gp = []
        for (xi, eta), w in zip(pts, wts):
            N, dN = shape_q4(xi, eta)
            J = self._xy.T @ dN
            detJ = float(np.linalg.det(J))
            if detJ <= 0.0:
                raise ValueError(
                    f"ShellMITC4 {self.tag}: non-positive Jacobian "
                    f"({detJ:.3e}) at Gauss point ({xi:.4f}, {eta:.4f}) — "
                    f"element is inverted or its nodes are numbered clockwise")
            dN_dx = dN @ np.linalg.inv(J)

            Bm = np.zeros((3, 24))
            Bb = np.zeros((3, 24))
            for a in range(4):
                u, v = 6 * a, 6 * a + 1
                rx, ry = 6 * a + 3, 6 * a + 4
                dx, dy = dN_dx[a]
                Bm[0, u] = dx
                Bm[1, v] = dy
                Bm[2, u] = dy
                Bm[2, v] = dx
                # kappa = [phi_x,x, phi_y,y, phi_x,y + phi_y,x],
                # phi_x = ry, phi_y = -rx
                Bb[0, ry] = dx
                Bb[1, rx] = -dy
                Bb[2, ry] = dy
                Bb[2, rx] = -dx

            # MITC4 assumed covariant shear, then covariant -> Cartesian
            B_cov = np.vstack([
                0.5 * (1.0 - eta) * row_xi_A + 0.5 * (1.0 + eta) * row_xi_C,
                0.5 * (1.0 - xi) * row_eta_D + 0.5 * (1.0 + xi) * row_eta_B,
            ])
            Bs = np.linalg.inv(J).T @ B_cov

            self._gp.append((N, Bm, Bb, Bs, w * detJ))

    def _build_k_local(self) -> None:
        mat = self.material
        Dm = mat.get_tangent()
        Db = Dm * self.t ** 3 / 12.0
        G = mat.E / (2.0 * (1.0 + mat.nu))
        Cs = _SHEAR_CORRECTION * G * self.t * np.eye(2)
        k = np.zeros((24, 24))
        for N, Bm, Bb, Bs, wdetJ in self._gp:
            k += wdetJ * self.t * (Bm.T @ Dm @ Bm)
            k += wdetJ * (Bb.T @ Db @ Bb)
            k += wdetJ * (Bs.T @ Cs @ Bs)
        # drilling: penalize theta_z against the membrane rotation
        # omega = (v,x - u,y)/2 at every Gauss point
        for N, Bm, _, _, wdetJ in self._gp:
            row = np.zeros(24)
            # recover dN_dx from Bm layout: Bm[0, u_a] = dNa/dx,
            # Bm[1, v_a] = dNa/dy
            for a in range(4):
                u, v, rz = 6 * a, 6 * a + 1, 6 * a + 5
                dx = Bm[0, u]
                dy = Bm[1, v]
                row[rz] = N[a]
                row[v] = -0.5 * dx
                row[u] = 0.5 * dy
            k += wdetJ * (self.drill_alpha * G * self.t) * np.outer(row, row)
        self._k_local = k

    def get_node_dofs(self) -> Sequence[Sequence[int]]:
        return ((0, 1, 2, 3, 4, 5),) * 4

    # -- element loads --------------------------------------------------------

    def zero_loads(self) -> None:
        self._q0 = np.zeros(24)

    def add_load(self, load, factor: float) -> None:
        if isinstance(load, ShellPressureLoad):
            p = factor * load.p
            for N, _, _, _, wdetJ in self._gp:
                for a in range(4):
                    self._q0[6 * a + 2] += N[a] * wdetJ * p
        elif isinstance(load, ShellBodyLoad):
            b_local = factor * self.t * (
                self._R @ np.array([load.bx, load.by, load.bz]))
            for N, _, _, _, wdetJ in self._gp:
                for a in range(4):
                    self._q0[6 * a:6 * a + 3] += N[a] * wdetJ * b_local
        else:
            super().add_load(load, factor)

    # -- state ----------------------------------------------------------------

    def _disp_local(self) -> np.ndarray:
        d = np.concatenate([n.get_trial_disp()[:6] for n in self.nodes])
        return self._T @ d

    def get_resisting_force(self) -> np.ndarray:
        return self._T.T @ (self._k_local @ self._disp_local() - self._q0)

    def get_tangent_stiff(self) -> np.ndarray:
        return self._T.T @ self._k_local @ self._T

    # -- responses --------------------------------------------------------------

    def forces_at_gauss(self) -> np.ndarray:
        """(4, 8) shell resultants [F11, F22, F12, M11, M22, M12, V13, V23]
        per unit width in LOCAL axes at the 2x2 Gauss points (order of
        isoparam.gauss_2d) for the current trial displacements."""
        u = self._disp_local()
        mat = self.material
        Dm = mat.get_tangent()
        Db = Dm * self.t ** 3 / 12.0
        G = mat.E / (2.0 * (1.0 + mat.nu))
        Cs = _SHEAR_CORRECTION * G * self.t * np.eye(2)
        out = np.empty((4, 8))
        for g, (N, Bm, Bb, Bs, _) in enumerate(self._gp):
            out[g, 0:3] = self.t * (Dm @ (Bm @ u))
            out[g, 3:6] = Db @ (Bb @ u)
            out[g, 6:8] = Cs @ (Bs @ u)
        return out

    def get_response(self, name: str):
        if name == "forces_gp":
            return self.forces_at_gauss()
        raise ValueError(f"ShellMITC4 {self.tag}: unknown response '{name}'")
