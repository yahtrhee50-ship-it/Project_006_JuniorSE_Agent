"""
Integrator — one of the four core contracts: defines what the tangent and
the residual (unbalance) *are* for the current analysis step.

LoadControl advances a load factor lambda by d_lambda each step and stores it
as the Domain pseudo-time, which the load patterns' time series convert into
load factors (LinearTimeSeries -> proportional loading).
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from src.fea.analysis_model import AnalysisModel
from src.fea.domain import Domain
from src.fea.system.soe import LinearSOE


class Integrator(ABC):
    @abstractmethod
    def new_step(self, domain: Domain, model: AnalysisModel) -> None: ...

    @abstractmethod
    def form_tangent(self, domain: Domain, model: AnalysisModel,
                     soe: LinearSOE) -> None: ...

    @abstractmethod
    def form_unbalance(self, domain: Domain, model: AnalysisModel,
                       soe: LinearSOE) -> None: ...


class LoadControl(Integrator):
    def __init__(self, d_lambda: float = 1.0) -> None:
        self.d_lambda = float(d_lambda)

    def new_step(self, domain: Domain, model: AnalysisModel) -> None:
        domain.current_time += self.d_lambda
        # Impose prescribed/fixed DOF values on trial displacements up front:
        # element resisting forces then carry their effect into the residual.
        model.apply_prescribed_disp(domain)

    def form_tangent(self, domain: Domain, model: AnalysisModel,
                     soe: LinearSOE) -> None:
        soe.zero_A()
        model.assemble_tangent(domain, soe)

    def form_unbalance(self, domain: Domain, model: AnalysisModel,
                       soe: LinearSOE) -> None:
        """b = lambda * P_ref - F_resisting(trial u)."""
        soe.zero_b()
        for elem in domain.elements():
            elem.zero_loads()
        for pattern in domain.load_patterns():
            pattern.apply(domain, model, soe, domain.current_time)
        model.assemble_resisting_force(domain, soe)
