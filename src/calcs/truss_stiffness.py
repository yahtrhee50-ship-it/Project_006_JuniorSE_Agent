"""
Plane truss analysis using the direct stiffness (matrix stiffness) method.

PlaneTruss — 2-D truss: axial members only, 2 DOFs per node (ux, uy).

Sign conventions
    Coordinates : x positive rightward, y positive upward (inches)
    Axial force : positive = tension, negative = compression
    Nodal loads : Fx positive rightward, Fy positive upward (kips)
    Units       : kips, in, ksi
"""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class TrussNode:
    id: str
    x: float            # in
    y: float            # in
    ux_fixed: bool = False
    uy_fixed: bool = False


@dataclass
class TrussMember:
    id: str
    i_node: str         # start node id
    j_node: str         # end node id
    A: float            # in²
    E: float            # ksi
    delta_L0: float = 0.0    # in; fabrication misfit — + = member fabricated too long (wants to elongate)
    delta_T_F: float = 0.0   # deg F temperature change
    alpha_per_F: float = 0.0 # coefficient of thermal expansion, 1/deg F


@dataclass
class NodalLoad:
    node_id: str
    Fx: float = 0.0     # kip (+rightward)
    Fy: float = 0.0     # kip (+upward)


@dataclass
class TrussResults:
    displacements: Dict[str, Tuple[float, float]]   # node_id -> (ux_in, uy_in)
    reactions:     Dict[str, Tuple[float, float]]   # node_id -> (Rx_kip, Ry_kip)
    member_forces: Dict[str, float]                 # member_id -> F_kip
    K_global:      np.ndarray                       # full assembled K (no BCs applied)
    node_order:    List[str]                        # DOF ordering used in K_global


class PlaneTruss:
    """Builder for a 2-D plane truss. Call solve() for results."""

    def __init__(self) -> None:
        self._nodes:   Dict[str, TrussNode]   = {}
        self._members: Dict[str, TrussMember] = {}
        self._loads:   Dict[str, NodalLoad]   = {}

    # ------------------------------------------------------------------
    # Builder methods
    # ------------------------------------------------------------------

    def add_node(self, id: str, x: float, y: float,
                 ux_fixed: bool = False, uy_fixed: bool = False) -> "PlaneTruss":
        self._nodes[id] = TrussNode(id, x, y, ux_fixed, uy_fixed)
        return self

    def add_member(self, id: str, i: str, j: str,
                   A: float, E: float, delta_L0: float = 0.0,
                   delta_T_F: float = 0.0, alpha_per_F: float = 0.0) -> "PlaneTruss":
        self._members[id] = TrussMember(id, i, j, A, E, delta_L0, delta_T_F, alpha_per_F)
        return self

    def add_load(self, node_id: str, Fx: float = 0.0,
                 Fy: float = 0.0) -> "PlaneTruss":
        self._loads[node_id] = NodalLoad(node_id, Fx, Fy)
        return self

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _member_k_global(self, m: TrussMember
                         ) -> Tuple[np.ndarray, float, float, float]:
        """Return 4×4 global stiffness contribution for member m, plus L, lx, ly."""
        ni = self._nodes[m.i_node]
        nj = self._nodes[m.j_node]
        dx = nj.x - ni.x
        dy = nj.y - ni.y
        L  = float(np.hypot(dx, dy))
        if L == 0.0:
            raise ValueError(f"Member {m.id}: zero length (nodes {m.i_node} and {m.j_node} coincide)")
        lx = dx / L
        ly = dy / L
        k  = m.A * m.E / L
        c, s = lx, ly
        km = k * np.array([
            [ c*c,  c*s, -c*c, -c*s],
            [ c*s,  s*s, -c*s, -s*s],
            [-c*c, -c*s,  c*c,  c*s],
            [-c*s, -s*s,  c*s,  s*s],
        ])
        return km, L, lx, ly

    @staticmethod
    def _member_F0(m: TrussMember, L: float) -> float:
        """Fixed-end axial force (kip) equivalent to the member's free elongation
        (fabrication misfit + thermal expansion). + = member wants to lengthen."""
        return m.E * m.A * (m.delta_L0 / L + m.alpha_per_F * m.delta_T_F)

    def _assemble(self) -> Tuple[np.ndarray, Dict[str, int]]:
        """Assemble full global K; return (K, node_index)."""
        node_ids   = list(self._nodes)
        node_index = {id: i for i, id in enumerate(node_ids)}
        ndof = 2 * len(node_ids)
        K    = np.zeros((ndof, ndof))

        for m in self._members.values():
            km, *_ = self._member_k_global(m)
            ii = node_index[m.i_node]
            jj = node_index[m.j_node]
            dofs = [2*ii, 2*ii+1, 2*jj, 2*jj+1]
            for r in range(4):
                for c in range(4):
                    K[dofs[r], dofs[c]] += km[r, c]

        return K, node_index

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    def assemble_K(self) -> Tuple[np.ndarray, List[str]]:
        """Return (K_global, node_order) without solving."""
        K, node_index = self._assemble()
        node_order = list(self._nodes)
        return K, node_order

    def solve(self) -> TrussResults:
        """Assemble K, apply BCs, solve for displacements, reactions, member forces."""
        node_ids   = list(self._nodes)
        node_index = {id: i for i, id in enumerate(node_ids)}
        ndof = 2 * len(node_ids)

        K, _ = self._assemble()

        # Categorise DOFs
        free_dofs  = []
        fixed_dofs = []
        for id, node in self._nodes.items():
            base = 2 * node_index[id]
            (fixed_dofs if node.ux_fixed else free_dofs).append(base)
            (fixed_dofs if node.uy_fixed else free_dofs).append(base + 1)

        if not free_dofs:
            raise ValueError("All DOFs are fixed — no free DOFs to solve.")

        # Force vector
        F = np.zeros(ndof)
        for load in self._loads.values():
            idx = node_index[load.node_id]
            F[2*idx]   += load.Fx
            F[2*idx+1] += load.Fy

        # Equivalent nodal loads from member fabrication misfit / thermal strain
        for m in self._members.values():
            if m.delta_L0 == 0.0 and (m.delta_T_F == 0.0 or m.alpha_per_F == 0.0):
                continue
            _, L, lx, ly = self._member_k_global(m)
            F0 = self._member_F0(m, L)
            ii = node_index[m.i_node]
            jj = node_index[m.j_node]
            F[2*ii]   += -F0 * lx
            F[2*ii+1] += -F0 * ly
            F[2*jj]   += F0 * lx
            F[2*jj+1] += F0 * ly

        # Solve partitioned system
        K_ff = K[np.ix_(free_dofs, free_dofs)]
        F_f  = F[free_dofs]
        try:
            d_f = np.linalg.solve(K_ff, F_f)
        except np.linalg.LinAlgError as exc:
            raise ValueError(f"Singular stiffness matrix — truss may be a mechanism: {exc}") from exc

        d = np.zeros(ndof)
        for i, dof in enumerate(free_dofs):
            d[dof] = d_f[i]

        # Reactions (full K·d − F)
        R = K @ d - F

        # Member axial forces
        member_forces: Dict[str, float] = {}
        for m in self._members.values():
            km, L, lx, ly = self._member_k_global(m)
            ii = node_index[m.i_node]
            jj = node_index[m.j_node]
            d_i = d[2*ii:2*ii+2]
            d_j = d[2*jj:2*jj+2]
            delta = lx*(d_j[0]-d_i[0]) + ly*(d_j[1]-d_i[1])
            F0 = self._member_F0(m, L)
            member_forces[m.id] = m.A * m.E / L * delta - F0

        displacements = {id: (d[2*node_index[id]], d[2*node_index[id]+1])
                         for id in node_ids}
        reactions     = {id: (R[2*node_index[id]], R[2*node_index[id]+1])
                         for id in node_ids}

        return TrussResults(
            displacements=displacements,
            reactions=reactions,
            member_forces=member_forces,
            K_global=K,
            node_order=node_ids,
        )
