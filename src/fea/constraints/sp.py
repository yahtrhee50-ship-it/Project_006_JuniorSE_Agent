"""
Single-point constraints and the ConstraintHandler.

SP_Constraint fixes or prescribes one DOF of one node (value 0.0 = fixed
support; nonzero = support settlement / prescribed displacement, applied at
full value — not scaled by the load factor — in Phase 0).

PlainHandler implements SP elimination: constrained DOFs get no equation
number; their prescribed values are imposed on node trial displacements so
element resisting forces carry the coupling to the free DOFs.

Multi-point constraints (equalDOF, rigid links) are handled by the
TransformationHandler in constraints/mp.py (Phase 2).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Tuple

from src.fea.analysis_model import AnalysisModel
from src.fea.domain import Domain
from src.fea.numberer import DOF_Numberer


@dataclass
class SP_Constraint:
    node_tag: int
    dof: int            # local DOF index at the node (0-based)
    value: float = 0.0  # prescribed displacement (0.0 = fixed)


class ConstraintHandler(ABC):
    @abstractmethod
    def handle(self, domain: Domain, model: AnalysisModel,
               numberer: DOF_Numberer) -> None:
        """Build the equation numbering in `model`, honoring constraints."""


def collect_sp_values(domain: Domain) -> Dict[Tuple[int, int], float]:
    """Gather SP constraints into {(node_tag, dof): value}, rejecting
    conflicting prescriptions of the same DOF."""
    constrained: Dict[Tuple[int, int], float] = {}
    for sp in domain.sp_constraints():
        key = (sp.node_tag, sp.dof)
        if key in constrained and constrained[key] != sp.value:
            raise ValueError(
                f"Conflicting SP constraints on node {sp.node_tag} "
                f"dof {sp.dof}")
        constrained[key] = sp.value
    return constrained


class PlainHandler(ConstraintHandler):
    """Handles SP constraints only, by elimination."""

    def handle(self, domain: Domain, model: AnalysisModel,
               numberer: DOF_Numberer) -> None:
        if domain.has_mp_constraints():
            raise ValueError(
                "Model has MP constraints — use TransformationHandler "
                "(src.fea.constraints.mp), not PlainHandler")
        model.build(domain, collect_sp_values(domain),
                    numberer.node_order(domain))
