"""
AnalysisModel — the bridge between the Domain and the global equation system.

Every node's GLOBAL displacement components resolve to a linear form

    u_node = g_node + T_node @ u_hat[idx_node]

over the free equations u_hat, where g carries prescribed (SP) values and
T carries: identity (plain free DOFs), nodal-axes rotations (skew supports),
and MP-constraint coupling (transformation method, K_hat = T^T K T assembled
element by element). Elements are untouched: they read global trial
displacements and scatter global stiffness/forces; this class owns the
mapping. Free-equation numbering is deterministic (numberer node order;
slaved DOFs get no equation).
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from src.fea.domain import Domain


class _NodeResolution:
    """u_global (ndf) = g + T @ u_hat[idx]."""

    __slots__ = ("idx", "T", "g", "plain_eq")

    def __init__(self, idx: np.ndarray, T: np.ndarray, g: np.ndarray,
                 plain_eq: Optional[np.ndarray]) -> None:
        self.idx = idx          # unique free-equation numbers this node uses
        self.T = T              # ndf x len(idx)
        self.g = g              # ndf constants (prescribed contributions)
        self.plain_eq = plain_eq  # legacy per-DOF eq array (plain nodes only)


class AnalysisModel:
    def __init__(self) -> None:
        self._res: Dict[int, _NodeResolution] = {}
        self._constrained: Dict[Tuple[int, int], float] = {}
        self._u = np.zeros(0)   # accumulated free-equation solution
        self.num_eq = 0

    # -- numbering / resolution ---------------------------------------------

    def build(self, domain: Domain,
              constrained: Dict[Tuple[int, int], float],
              node_order: List[int],
              mps: Sequence = ()) -> None:
        """Assign equation numbers and build every node's resolution.

        constrained : {(node_tag, local_dof): prescribed_value} from the
                      ConstraintHandler (0.0 = fixed support). For a node
                      with skew axes, local_dof indexes the ROTATED frame.
        node_order  : deterministic ordering from the DOF_Numberer.
        mps         : MP_Constraints (transformation method).
        """
        self._res.clear()
        self._constrained = dict(constrained)

        # -- hygiene checks on the MP set ---------------------------------
        # An MP may couple its constrained node to SEVERAL retained nodes
        # (mp.terms = (retained_tag, retained_dofs, matrix) triples, e.g. a
        # mesh hanging node tied to both ends of the coarse edge).
        slaved: Dict[Tuple[int, int], Tuple] = {}   # (tag, dof) -> (mp, row)
        for mp in mps:
            for row, dof in enumerate(mp.constrained_dofs):
                key = (mp.constrained_tag, dof)
                if key in slaved:
                    raise ValueError(
                        f"DOF {dof} of node {mp.constrained_tag} is slaved "
                        f"by more than one MP constraint")
                if key in constrained:
                    raise ValueError(
                        f"DOF {dof} of node {mp.constrained_tag} is both "
                        f"slaved (MP) and SP-constrained")
                slaved[key] = (mp, row)
        for mp in mps:
            for rtag, _, _ in mp.terms:
                if any((rtag, d) in slaved
                       for d in range(domain.get_node(rtag).ndf)):
                    raise ValueError(
                        f"Node {rtag} is retained by one MP constraint and "
                        f"slaved by another — chains are not supported")
            if domain.get_node(mp.constrained_tag).axes is not None:
                raise ValueError(
                    f"Slaved node {mp.constrained_tag} has skew nodal axes "
                    f"— not supported (put the skew on the retained node)")

        # -- pass 1: number the equations (slaved DOFs get none) -----------
        # For a skew node the equation slots live in the ROTATED frame.
        slot_eq: Dict[Tuple[int, int], int] = {}
        next_free = 0
        for tag in node_order:
            node = domain.get_node(tag)
            for dof in range(node.ndf):
                if (tag, dof) in slaved or (tag, dof) in constrained:
                    continue
                slot_eq[(tag, dof)] = next_free
                next_free += 1
        self.num_eq = next_free
        self._u = np.zeros(next_free)

        # -- pass 2: resolve non-slaved nodes ------------------------------
        for tag in node_order:
            node = domain.get_node(tag)
            if any((tag, d) in slaved for d in range(node.ndf)):
                continue
            self._res[tag] = self._resolve_plain_or_skew(
                node, slot_eq, constrained)

        # -- pass 3: resolve slaved nodes through their retained nodes -----
        for tag in node_order:
            node = domain.get_node(tag)
            rows_terms: List[Dict[int, float]] = [dict() for _ in range(node.ndf)]
            g = np.zeros(node.ndf)
            is_slaved_node = False
            for dof in range(node.ndf):
                key = (tag, dof)
                if key not in slaved:
                    continue
                is_slaved_node = True
                mp, row = slaved[key]
                for rtag, rdofs, mat in mp.terms:
                    r_res = self._res[rtag]
                    for col, rdof in enumerate(rdofs):
                        c = mat[row, col]
                        if c == 0.0:
                            continue
                        # retained node's GLOBAL component rdof:
                        # g + T[rdof,:] @ u
                        g[dof] += c * r_res.g[rdof]
                        for k, eq in enumerate(r_res.idx):
                            coeff = c * r_res.T[rdof, k]
                            if coeff != 0.0:
                                rows_terms[dof][int(eq)] = (
                                    rows_terms[dof].get(int(eq), 0.0) + coeff)
            if not is_slaved_node:
                continue
            # non-slaved DOFs of this node resolve on their own slots
            for dof in range(node.ndf):
                if (tag, dof) in slaved:
                    continue
                if (tag, dof) in constrained:
                    g[dof] = constrained[(tag, dof)]
                else:
                    rows_terms[dof][slot_eq[(tag, dof)]] = 1.0
            self._res[tag] = self._pack(rows_terms, g, node.ndf)

    def _resolve_plain_or_skew(self, node, slot_eq, constrained
                               ) -> _NodeResolution:
        tag, ndf = node.tag, node.ndf
        if node.axes is None:
            eq = np.empty(ndf, dtype=int)
            idx: List[int] = []
            T_rows: List[Tuple[int, int]] = []   # (dof, col into idx)
            g = np.zeros(ndf)
            next_con = -1
            for dof in range(ndf):
                if (tag, dof) in constrained:
                    g[dof] = constrained[(tag, dof)]
                    eq[dof] = next_con
                    next_con -= 1
                else:
                    eq[dof] = slot_eq[(tag, dof)]
                    T_rows.append((dof, len(idx)))
                    idx.append(slot_eq[(tag, dof)])
            T = np.zeros((ndf, len(idx)))
            for dof, col in T_rows:
                T[dof, col] = 1.0
            return _NodeResolution(np.asarray(idx, dtype=int), T, g, eq)
        # skew node: u_global = R @ u_local; local slots are free or SP'd
        R = node.axes
        rows_terms: List[Dict[int, float]] = [dict() for _ in range(ndf)]
        g = np.zeros(ndf)
        for j in range(ndf):
            if (tag, j) in constrained:
                val = constrained[(tag, j)]
                for i in range(ndf):
                    g[i] += R[i, j] * val
            else:
                eq_j = slot_eq[(tag, j)]
                for i in range(ndf):
                    if R[i, j] != 0.0:
                        rows_terms[i][eq_j] = (
                            rows_terms[i].get(eq_j, 0.0) + R[i, j])
        return self._pack(rows_terms, g, ndf)

    @staticmethod
    def _pack(rows_terms: List[Dict[int, float]], g: np.ndarray,
              ndf: int) -> _NodeResolution:
        idx = sorted({eq for row in rows_terms for eq in row})
        pos = {eq: k for k, eq in enumerate(idx)}
        T = np.zeros((ndf, len(idx)))
        for dof, row in enumerate(rows_terms):
            for eq, coeff in row.items():
                T[dof, pos[eq]] = coeff
        return _NodeResolution(np.asarray(idx, dtype=int), T, g, None)

    # -- legacy access (plain nodes) ----------------------------------------

    def eq_numbers(self, node_tag: int) -> np.ndarray:
        """Per-DOF equation numbers (free >= 0, constrained < 0). Only
        defined for plain nodes (no skew axes, not slaved)."""
        res = self._res[node_tag]
        if res.plain_eq is None:
            raise ValueError(
                f"Node {node_tag} has skew axes or is MP-slaved — its DOFs "
                f"have no one-to-one equation numbers")
        return res.plain_eq.copy()

    def id_for(self, element) -> np.ndarray:
        """Element location array (plain-node elements only)."""
        ids: List[int] = []
        for node_tag, dofs in zip(element.node_tags, element.get_node_dofs()):
            eq = self.eq_numbers(node_tag)
            ids.extend(int(eq[d]) for d in dofs)
        return np.asarray(ids, dtype=int)

    # -- element resolution ---------------------------------------------------

    def _element_transform(self, element):
        """(idx_e, T_e) with rows = element global DOFs, cols = the union of
        free equations touched: u_ele = g_e + T_e @ u_hat[idx_e]."""
        node_res = [self._res[t] for t in element.node_tags]
        idx_e = sorted({int(eq) for res in node_res for eq in res.idx})
        pos = {eq: k for k, eq in enumerate(idx_e)}
        rows = sum(len(dofs) for dofs in element.get_node_dofs())
        T_e = np.zeros((rows, len(idx_e)))
        r = 0
        for res, dofs in zip(node_res, element.get_node_dofs()):
            cols = [pos[int(eq)] for eq in res.idx]
            for d in dofs:
                T_e[r, cols] = res.T[d, :]
                r += 1
        return np.asarray(idx_e, dtype=int), T_e

    def _is_plain(self, element) -> bool:
        return all(self._res[t].plain_eq is not None
                   for t in element.node_tags)

    # -- scatter (assembly) ---------------------------------------------------

    def assemble_tangent(self, domain: Domain, soe) -> None:
        for elem in domain.elements():
            ke = elem.get_tangent_stiff()
            if self._is_plain(elem):
                soe.add_matrix(ke, self.id_for(elem))
            else:
                idx_e, T_e = self._element_transform(elem)
                soe.add_matrix(T_e.T @ ke @ T_e, idx_e)

    def assemble_resisting_force(self, domain: Domain, soe) -> None:
        """Subtract element resisting forces from the RHS (unbalance)."""
        for elem in domain.elements():
            fe = elem.get_resisting_force()
            if self._is_plain(elem):
                soe.add_vector(-fe, self.id_for(elem))
            else:
                idx_e, T_e = self._element_transform(elem)
                soe.add_vector(-(T_e.T @ fe), idx_e)

    def add_nodal_load_to_rhs(self, soe, node_tag: int,
                              values: np.ndarray) -> None:
        """Scatter a GLOBAL nodal load vector into the reduced RHS:
        b += T_node^T @ values (loads on slaved/skew DOFs transfer to the
        retained/rotated equations with the transformation coefficients)."""
        res = self._res[node_tag]
        if res.idx.size:
            soe.add_vector(res.T.T @ np.asarray(values, dtype=float), res.idx)

    # -- constrained (prescribed) DOFs ----------------------------------------

    def apply_prescribed_disp(self, domain: Domain) -> None:
        """Impose the resolved displacement state (prescribed constants +
        current free-equation solution) on node trial displacements."""
        self._push_trials(domain)

    # -- gather ---------------------------------------------------------------

    def update(self, domain: Domain, delta_u: np.ndarray) -> None:
        """Accumulate a free-equation increment and push the resolved state
        into node trial displacements (slaved/skew DOFs included)."""
        self._u += delta_u
        self._push_trials(domain)

    def _push_trials(self, domain: Domain) -> None:
        for tag, res in self._res.items():
            disp = res.g.copy()
            if res.idx.size:
                disp += res.T @ self._u[res.idx]
            domain.get_node(tag).set_trial_disp(disp)

    # -- reactions --------------------------------------------------------------

    def compute_reactions(self, domain: Domain) -> None:
        """Store on each node its residual = internal - external force
        (GLOBAL components).

        At constrained DOFs this is the support reaction; at free DOFs it is
        the equilibrium residual (≈ 0 for a converged/linear solution). At
        MP-coupled pairs the slave and retained residuals carry the
        constraint interaction forces (equal and opposite through T^T). For
        a skew node, rotate with node.axes.T to read the reaction in the
        inclined support frame.
        """
        residual: Dict[int, np.ndarray] = {
            node.tag: np.zeros(node.ndf) for node in domain.nodes()}

        for elem in domain.elements():
            f = elem.get_resisting_force()
            k = 0
            for node_tag, dofs in zip(elem.node_tags, elem.get_node_dofs()):
                for d in dofs:
                    residual[node_tag][d] += f[k]
                    k += 1

        for pattern in domain.load_patterns():
            factor = pattern.time_series.get_factor(domain.current_time)
            for load in pattern.nodal_loads:
                residual[load.node_tag] -= factor * load.values

        for node in domain.nodes():
            node.reaction = residual[node.tag]
