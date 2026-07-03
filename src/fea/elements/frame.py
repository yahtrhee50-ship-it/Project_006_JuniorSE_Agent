"""
Frame — 2D linear elastic frame (beam-column) element: 3 DOF/node
(ux, uy, rz), combining axial (EA/L) and Euler-Bernoulli bending stiffness
in local coordinates, then rotated to global via direction cosines.

Local DOF order: [u1, v1, th1, u2, v2, th2]. Local x runs node i -> node j;
local y is local x rotated +90 deg (right-handed, matches global z), so
theta_local == rz_global with no sign flip and the standard Euler-Bernoulli
4x4 (v, theta) block applies unchanged.

Element loads (FrameUniformLoad / FramePointLoad, given in GLOBAL
components) are projected onto local axial/transverse directions and
converted to consistent nodal loads via the Hermitian shape functions
(matching src/calcs/beam_stiffness.py's _hermite_ints / point-load FEF,
verified against Hibbeler Ch 15). Because the element is linear and these
loads are constant (state-independent), get_resisting_force is affine in
displacement and get_tangent_stiff is its exact derivative.

Phase-1 scope is the 2D in-plane case (Hibbeler Ch 16 oracle); the 3D
6-DOF upgrade (biaxial bending + torsion) is deferred, see
docs/fea_engine_plan.md's Phase 1 frame.py note.
"""
from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np

from src.fea.elements.element import Element
from src.fea.loads.pattern import FramePointLoad, FrameUniformLoad
from src.fea.sections.section import ElasticSection


def _bernoulli_4x4(EI: float, L: float) -> np.ndarray:
    """4x4 Euler-Bernoulli local bending stiffness for [v1, th1, v2, th2]."""
    c = EI / L**3
    return c * np.array([
        [ 12.0,    6.0*L,   -12.0,    6.0*L   ],
        [  6.0*L,  4.0*L**2, -6.0*L,   2.0*L**2],
        [-12.0,   -6.0*L,    12.0,   -6.0*L   ],
        [  6.0*L,  2.0*L**2, -6.0*L,   4.0*L**2],
    ], dtype=float)


def _hermite_ints(alpha: float, beta: float) -> Tuple[float, float, float, float]:
    """Definite integrals of the Hermite shape functions over [alpha, beta]
    subset of [0, 1] (dimensionless local coordinate)."""
    def s1(x): return x - x**3 + x**4 / 2
    def s2(x): return x**2 / 2 - 2*x**3 / 3 + x**4 / 4
    def s3(x): return x**3 - x**4 / 2
    def s4(x): return -x**3 / 3 + x**4 / 4
    return s1(beta)-s1(alpha), s2(beta)-s2(alpha), s3(beta)-s3(alpha), s4(beta)-s4(alpha)


def _hermite_shapes(xi: float, L: float) -> Tuple[float, float, float, float]:
    N1 = 1 - 3*xi**2 + 2*xi**3
    N2 = L * (xi - 2*xi**2 + xi**3)
    N3 = 3*xi**2 - 2*xi**3
    N4 = L * (-xi**2 + xi**3)
    return N1, N2, N3, N4


class Frame(Element):
    """2-node, 3-DOF/node (ux, uy, rz) linear elastic 2D frame element."""

    def __init__(self, tag: int, node_tags: Sequence[int],
                 section: ElasticSection) -> None:
        super().__init__(tag, node_tags)
        if len(self.node_tags) != 2:
            raise ValueError(f"Frame {tag}: needs exactly 2 nodes")
        self.section = section
        self.L = 0.0
        self._cx = 0.0
        self._cy = 0.0
        self._T = np.zeros((6, 6))
        self._q0 = np.zeros(6)   # local equivalent nodal loads

    def set_domain(self, domain) -> None:
        super().set_domain(domain)
        ni, nj = self.nodes
        if ni.ndm != 2 or nj.ndm != 2:
            raise ValueError(f"Frame {self.tag}: needs 2D (ndm=2) nodes")
        dx = nj.coords - ni.coords
        self.L = float(np.linalg.norm(dx))
        if self.L == 0.0:
            raise ValueError(f"Frame {self.tag}: zero length")
        self._cx, self._cy = dx / self.L
        r = np.array([
            [ self._cx, self._cy, 0.0],
            [-self._cy, self._cx, 0.0],
            [ 0.0,      0.0,      1.0],
        ])
        T = np.zeros((6, 6))
        T[:3, :3] = r
        T[3:, 3:] = r
        self._T = T

    def get_node_dofs(self) -> Sequence[Sequence[int]]:
        return ((0, 1, 2), (0, 1, 2))

    # -- local stiffness ------------------------------------------------------

    def _k_local(self) -> np.ndarray:
        EA_L = self.section.EA / self.L
        k = np.zeros((6, 6))
        k[0, 0] = k[3, 3] = EA_L
        k[0, 3] = k[3, 0] = -EA_L
        b = _bernoulli_4x4(self.section.EIz, self.L)
        idx = (1, 2, 4, 5)
        for a, ia in enumerate(idx):
            for c, ic in enumerate(idx):
                k[ia, ic] += b[a, c]
        return k

    def _disp_local(self) -> np.ndarray:
        ni, nj = self.nodes
        d_global = np.concatenate([ni.get_trial_disp()[:3], nj.get_trial_disp()[:3]])
        return self._T @ d_global

    # -- element loads ----------------------------------------------------------

    def zero_loads(self) -> None:
        self._q0 = np.zeros(6)

    def add_load(self, load, factor: float) -> None:
        """Project GLOBAL load components onto local axial/transverse axes
        and accumulate consistent nodal loads (see module docstring)."""
        L = self.L
        cx, cy = self._cx, self._cy
        if isinstance(load, FrameUniformLoad):
            wx_local = factor * (cx * load.wx + cy * load.wy)
            wy_local = factor * (-cy * load.wx + cx * load.wy)
            self._q0[0] += wx_local * L / 2.0
            self._q0[3] += wx_local * L / 2.0
            i1, i2, i3, i4 = _hermite_ints(0.0, 1.0)
            self._q0[1] += wy_local * L * i1
            self._q0[2] += wy_local * L**2 * i2
            self._q0[4] += wy_local * L * i3
            self._q0[5] += wy_local * L**2 * i4
        elif isinstance(load, FramePointLoad):
            a = load.distance_from_i
            if a < -1e-9 or a > L + 1e-9:
                raise ValueError(
                    f"Frame {self.tag}: point load distance {a} outside [0, {L}]")
            xi = min(max(a / L, 0.0), 1.0)
            Px_local = factor * (cx * load.Px + cy * load.Py)
            Py_local = factor * (-cy * load.Px + cx * load.Py)
            self._q0[0] += Px_local * (1.0 - xi)
            self._q0[3] += Px_local * xi
            N1, N2, N3, N4 = _hermite_shapes(xi, L)
            self._q0[1] += Py_local * N1
            self._q0[2] += Py_local * N2
            self._q0[4] += Py_local * N3
            self._q0[5] += Py_local * N4
        else:
            super().add_load(load, factor)

    # -- state ------------------------------------------------------------------

    def get_local_forces(self) -> np.ndarray:
        """Local basic forces [N1, V1, M1, N2, V2, M2] (member convention:
        local x axial, local y transverse, moments CCW+ matching rz)."""
        v_local = self._disp_local()
        return self._k_local() @ v_local - self._q0

    def get_resisting_force(self) -> np.ndarray:
        return self._T.T @ self.get_local_forces()

    def get_tangent_stiff(self) -> np.ndarray:
        return self._T.T @ self._k_local() @ self._T

    # -- responses ----------------------------------------------------------

    def get_response(self, name: str):
        f = self.get_local_forces()
        mapping = {"N1": f[0], "V1": f[1], "M1": f[2],
                   "N2": f[3], "V2": f[4], "M2": f[5]}
        if name in mapping:
            return mapping[name]
        raise ValueError(f"Frame {self.tag}: unknown response '{name}'")
