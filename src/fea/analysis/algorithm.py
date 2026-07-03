"""
SolutionAlgorithm — one of the four core contracts: drives the iteration
that brings the residual to zero within a step.

Linear does exactly one tangent solve (exact for linear systems; Newton-
Raphson with convergence tests arrives in Phase 4).
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from src.fea.analysis_model import AnalysisModel
from src.fea.domain import Domain
from src.fea.analysis.integrator import Integrator
from src.fea.system.soe import LinearSOE


class SolutionAlgorithm(ABC):
    @abstractmethod
    def solve_current_step(self, domain: Domain, model: AnalysisModel,
                           integrator: Integrator, soe: LinearSOE) -> None:
        """Raise on failure; return normally on success."""


class Linear(SolutionAlgorithm):
    def solve_current_step(self, domain: Domain, model: AnalysisModel,
                           integrator: Integrator, soe: LinearSOE) -> None:
        integrator.form_tangent(domain, model, soe)
        integrator.form_unbalance(domain, model, soe)
        delta_u = soe.solve()
        model.update(domain, delta_u)
