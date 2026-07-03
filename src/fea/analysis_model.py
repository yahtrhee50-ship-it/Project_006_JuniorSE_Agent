"""
AnalysisModel — the bridge between the Domain and the global equation system.

Owns the DOF -> equation-number map (free DOFs >= 0, constrained DOFs < 0),
the scatter of element matrices/vectors into the SystemOfEqn, and the gather
of the solution back into node trial displacements. This mapping is the thing
the whole engine leans on, so it is explicit and deterministic.
"""
from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np

from src.fea.domain import Domain


class AnalysisModel:
    def __init__(self) -> None:
        # node tag -> array of equation numbers, one per node DOF.
        # Free DOF: 0..num_eq-1. Constrained DOF k: -(k+1).
        self._eq: Dict[int, np.ndarray] = {}
        # (node_tag, local_dof) -> prescribed value, in constraint index order
        self._constrained: Dict[Tuple[int, int], float] = {}
        self.num_eq = 0

    # -- numbering ---------------------------------------------------------

    def build(self, domain: Domain,
              constrained: Dict[Tuple[int, int], float],
              node_order: List[int]) -> None:
        """Assign equation numbers to every node DOF.

        constrained : {(node_tag, local_dof): prescribed_value} from the
                      ConstraintHandler (0.0 = fixed support).
        node_order  : deterministic ordering from the DOF_Numberer.
        """
        self._eq.clear()
        self._constrained = dict(constrained)
        next_free = 0
        next_con = 0
        for tag in node_order:
            node = domain.get_node(tag)
            eq = np.empty(node.ndf, dtype=int)
            for dof in range(node.ndf):
                if (tag, dof) in constrained:
                    next_con += 1
                    eq[dof] = -next_con
                else:
                    eq[dof] = next_free
                    next_free += 1
            self._eq[tag] = eq
        self.num_eq = next_free

    def eq_numbers(self, node_tag: int) -> np.ndarray:
        return self._eq[node_tag].copy()

    def id_for(self, element) -> np.ndarray:
        """Element location array: global equation numbers for each element
        DOF, in the element's local DOF order."""
        ids: List[int] = []
        for node_tag, dofs in zip(element.node_tags, element.get_node_dofs()):
            eq = self._eq[node_tag]
            ids.extend(int(eq[d]) for d in dofs)
        return np.asarray(ids, dtype=int)

    # -- scatter (assembly) --------------------------------------------------

    def assemble_tangent(self, domain: Domain, soe) -> None:
        for elem in domain.elements():
            soe.add_matrix(elem.get_tangent_stiff(), self.id_for(elem))

    def assemble_resisting_force(self, domain: Domain, soe) -> None:
        """Subtract element resisting forces from the RHS (unbalance)."""
        for elem in domain.elements():
            soe.add_vector(-elem.get_resisting_force(), self.id_for(elem))

    # -- constrained (prescribed) DOFs ----------------------------------------

    def apply_prescribed_disp(self, domain: Domain) -> None:
        """Impose prescribed values (support settlements; 0 for fixed
        supports) directly on node trial displacements. Element resisting
        forces then carry the coupling terms to the free DOFs automatically."""
        for (tag, dof), value in self._constrained.items():
            domain.get_node(tag).set_trial_disp_component(dof, value)

    # -- gather ---------------------------------------------------------------

    def update(self, domain: Domain, delta_u: np.ndarray) -> None:
        """Scatter a solution increment for the free DOFs into node trial
        displacements."""
        for tag, eq in self._eq.items():
            node = domain.get_node(tag)
            for dof in range(node.ndf):
                if eq[dof] >= 0:
                    node.incr_trial_disp_component(dof, float(delta_u[eq[dof]]))

    # -- reactions --------------------------------------------------------------

    def compute_reactions(self, domain: Domain) -> None:
        """Store on each node its residual = internal - external force.

        At constrained DOFs this is the support reaction; at free DOFs it is
        the equilibrium residual (≈ 0 for a converged/linear solution).
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
