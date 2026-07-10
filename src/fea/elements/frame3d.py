"""
Frame3D — 3D linear elastic frame (beam-column) element: 6 DOF/node
(ux, uy, uz, rx, ry, rz), combining axial (EA/L), St-Venant torsion (GJ/L),
and Euler-Bernoulli bending about both principal axes (EIz in the local x-y
plane, EIy in the local x-z plane) in local coordinates, then rotated to
global via a 3x3 direction-cosine matrix.

Local DOF order: [ux1, uy1, uz1, rx1, ry1, rz1, ux2, uy2, uz2, rx2, ry2, rz2].

Orientation follows the OpenSees geomTransf convention: local x runs node
i -> node j; `vecxz` is any vector lying in the local x-z plane (not
parallel to the member axis); local y = vecxz x ex (normalized), local
z = ex x ey. Bending in the local x-y plane (v, theta_z) uses the standard
Euler-Bernoulli 4x4 unchanged; bending in the local x-z plane (w, theta_y)
uses the same 4x4 with the theta rows/columns sign-flipped, because a
right-handed theta_y equals -dw/dx (S = diag(1,-1,1,-1), k_xz = S B S).

Element loads (FrameUniformLoad3D / FramePointLoad3D, GLOBAL components)
are projected onto the local axes and converted to consistent nodal loads
via the same Hermitian shape-function integrals as the 2D Frame, with the
theta_y sign flip applied to the x-z plane terms.

Member end moment releases (release_i / release_j) release BOTH bending
moments (local My and Mz) at that end by static condensation — a released
end is a 3D pin for bending but keeps torsional continuity. Rigid end
offsets (offset_i / offset_j, GLOBAL 3-vectors from the node to the
flexible member end) use the rigid-link kinematics u_end = u + theta x o
per node; length/orientation come from the offset end coordinates and
element loads span only the flexible length. Both generalize the validated
2D Frame implementations (kept untouched as the regression baseline).
"""
from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np

from src.fea.elements.element import Element
from src.fea.elements.frame import (_bernoulli_4x4, _hermite_ints,
                                    _hermite_shapes)
from src.fea.loads.pattern import FramePointLoad3D, FrameUniformLoad3D
from src.fea.sections.section import ElasticSection

# Local DOF index groups (per the module-docstring ordering).
_AXIAL = (0, 6)
_TORSION = (3, 9)
_BEND_XY = (1, 5, 7, 11)    # (v1, th_z1, v2, th_z2) — standard signs
_BEND_XZ = (2, 4, 8, 10)    # (w1, th_y1, w2, th_y2) — theta signs flipped
_S_FLIP = np.diag([1.0, -1.0, 1.0, -1.0])


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


class Frame3D(Element):
    """2-node, 6-DOF/node (ux, uy, uz, rx, ry, rz) linear elastic 3D frame."""

    def __init__(self, tag: int, node_tags: Sequence[int],
                 section: ElasticSection, vecxz: Sequence[float],
                 release_i: bool = False, release_j: bool = False,
                 offset_i: Sequence[float] = (0.0, 0.0, 0.0),
                 offset_j: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        super().__init__(tag, node_tags)
        if len(self.node_tags) != 2:
            raise ValueError(f"Frame3D {tag}: needs exactly 2 nodes")
        self.section = section
        for name, val in (("Iy", section.Iy), ("J", section.J),
                          ("G", section.G)):
            if val <= 0.0:
                raise ValueError(
                    f"Frame3D {tag}: section needs positive {name} "
                    f"(got {val}) — 3D bending/torsion requires Iy, J, G")
        self._vecxz = np.asarray(vecxz, dtype=float)
        if self._vecxz.shape != (3,) or np.linalg.norm(self._vecxz) == 0.0:
            raise ValueError(f"Frame3D {tag}: vecxz must be a nonzero 3-vector")
        self._released = tuple(
            idx for idxs, rel in (((4, 5), release_i), ((10, 11), release_j))
            if rel for idx in idxs)
        self._offset_i = np.asarray(offset_i, dtype=float)
        self._offset_j = np.asarray(offset_j, dtype=float)
        if self._offset_i.shape != (3,) or self._offset_j.shape != (3,):
            raise ValueError(
                f"Frame3D {tag}: offsets must be 3D global (dx, dy, dz) vectors")
        self.L = 0.0
        self._R = np.eye(3)      # rows = local axes in global coords
        self._T = np.zeros((12, 12))
        self._W = np.eye(12)     # rigid-link node -> flexible-end kinematics
        self._q0 = np.zeros(12)  # local equivalent nodal loads

    def set_domain(self, domain) -> None:
        super().set_domain(domain)
        ni, nj = self.nodes
        if ni.ndm != 3 or nj.ndm != 3:
            raise ValueError(f"Frame3D {self.tag}: needs 3D (ndm=3) nodes")
        end_i = ni.coords + self._offset_i
        end_j = nj.coords + self._offset_j
        dx = end_j - end_i
        self.L = float(np.linalg.norm(dx))
        if self.L == 0.0:
            raise ValueError(f"Frame3D {self.tag}: zero flexible length")
        ex = dx / self.L
        ey = np.cross(self._vecxz, ex)
        ny = np.linalg.norm(ey)
        if ny < 1e-8 * np.linalg.norm(self._vecxz):
            raise ValueError(
                f"Frame3D {self.tag}: vecxz is parallel to the member axis")
        ey = ey / ny
        ez = np.cross(ex, ey)
        self._R = np.vstack([ex, ey, ez])
        T = np.zeros((12, 12))
        for blk in range(4):
            T[3*blk:3*blk+3, 3*blk:3*blk+3] = self._R
        self._T = T
        # Rigid link: u_end = u_node + theta x offset = u - skew(o) @ theta.
        W = np.eye(12)
        W[0:3, 3:6] = -_skew(self._offset_i)
        W[6:9, 9:12] = -_skew(self._offset_j)
        self._W = W

    def get_node_dofs(self) -> Sequence[Sequence[int]]:
        return ((0, 1, 2, 3, 4, 5), (0, 1, 2, 3, 4, 5))

    # -- local stiffness ------------------------------------------------------

    def _k_local(self) -> np.ndarray:
        k = np.zeros((12, 12))
        EA_L = self.section.EA / self.L
        k[np.ix_(_AXIAL, _AXIAL)] += EA_L * np.array([[1.0, -1.0],
                                                      [-1.0, 1.0]])
        GJ_L = self.section.GJ / self.L
        k[np.ix_(_TORSION, _TORSION)] += GJ_L * np.array([[1.0, -1.0],
                                                          [-1.0, 1.0]])
        k[np.ix_(_BEND_XY, _BEND_XY)] += _bernoulli_4x4(self.section.EIz, self.L)
        b_y = _bernoulli_4x4(self.section.EIy, self.L)
        k[np.ix_(_BEND_XZ, _BEND_XZ)] += _S_FLIP @ b_y @ _S_FLIP
        return k

    def _condensed(self) -> Tuple[np.ndarray, np.ndarray]:
        """Local stiffness and equivalent-load vector with released bending
        DOFs statically condensed out (released end moments exactly zero)."""
        k = self._k_local()
        q = self._q0
        if not self._released:
            return k, q
        e = list(self._released)
        r = [i for i in range(12) if i not in self._released]
        kee_inv = np.linalg.inv(k[np.ix_(e, e)])
        k_cond = np.zeros((12, 12))
        k_cond[np.ix_(r, r)] = k[np.ix_(r, r)] - k[np.ix_(r, e)] @ kee_inv @ k[np.ix_(e, r)]
        q_cond = np.zeros(12)
        q_cond[r] = q[r] - k[np.ix_(r, e)] @ kee_inv @ q[e]
        return k_cond, q_cond

    def _disp_local(self) -> np.ndarray:
        ni, nj = self.nodes
        d_global = np.concatenate([ni.get_trial_disp()[:6],
                                   nj.get_trial_disp()[:6]])
        return self._T @ (self._W @ d_global)

    # -- element loads ----------------------------------------------------------

    def zero_loads(self) -> None:
        self._q0 = np.zeros(12)

    def add_load(self, load, factor: float) -> None:
        """Project GLOBAL load components onto local axes and accumulate
        consistent nodal loads (see module docstring)."""
        L = self.L
        if isinstance(load, FrameUniformLoad3D):
            w = factor * (self._R @ np.array([load.wx, load.wy, load.wz]))
            self._q0[0] += w[0] * L / 2.0
            self._q0[6] += w[0] * L / 2.0
            i1, i2, i3, i4 = _hermite_ints(0.0, 1.0)
            self._q0[1] += w[1] * L * i1
            self._q0[5] += w[1] * L**2 * i2
            self._q0[7] += w[1] * L * i3
            self._q0[11] += w[1] * L**2 * i4
            self._q0[2] += w[2] * L * i1
            self._q0[4] -= w[2] * L**2 * i2
            self._q0[8] += w[2] * L * i3
            self._q0[10] -= w[2] * L**2 * i4
        elif isinstance(load, FramePointLoad3D):
            a = load.distance_from_i
            if a < -1e-9 or a > L + 1e-9:
                raise ValueError(
                    f"Frame3D {self.tag}: point load distance {a} outside [0, {L}]")
            xi = min(max(a / L, 0.0), 1.0)
            P = factor * (self._R @ np.array([load.Px, load.Py, load.Pz]))
            self._q0[0] += P[0] * (1.0 - xi)
            self._q0[6] += P[0] * xi
            N1, N2, N3, N4 = _hermite_shapes(xi, L)
            self._q0[1] += P[1] * N1
            self._q0[5] += P[1] * N2
            self._q0[7] += P[1] * N3
            self._q0[11] += P[1] * N4
            self._q0[2] += P[2] * N1
            self._q0[4] -= P[2] * N2
            self._q0[8] += P[2] * N3
            self._q0[10] -= P[2] * N4
        else:
            super().add_load(load, factor)

    # -- state ------------------------------------------------------------------

    def get_local_forces(self) -> np.ndarray:
        """Local basic forces [N1, Vy1, Vz1, T1, My1, Mz1,
        N2, Vy2, Vz2, T2, My2, Mz2] (member convention: local x axial,
        moments right-handed about the local axes). Released end bending
        moments are exactly zero."""
        v_local = self._disp_local()
        k_cond, q_cond = self._condensed()
        return k_cond @ v_local - q_cond

    def get_resisting_force(self) -> np.ndarray:
        return self._W.T @ (self._T.T @ self.get_local_forces())

    def get_tangent_stiff(self) -> np.ndarray:
        TW = self._T @ self._W
        k_cond, _ = self._condensed()
        return TW.T @ k_cond @ TW

    # -- responses ----------------------------------------------------------

    def get_response(self, name: str):
        f = self.get_local_forces()
        names = ("N1", "Vy1", "Vz1", "T1", "My1", "Mz1",
                 "N2", "Vy2", "Vz2", "T2", "My2", "Mz2")
        try:
            return f[names.index(name)]
        except ValueError:
            raise ValueError(
                f"Frame3D {self.tag}: unknown response '{name}'") from None
