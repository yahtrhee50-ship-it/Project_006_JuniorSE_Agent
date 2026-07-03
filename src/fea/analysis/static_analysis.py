"""
StaticAnalysis — the aggregation that runs a static analysis: it owns one of
each interchangeable component and orchestrates
    handle -> size -> [new_step -> solve -> commit -> reactions -> record]*
"""
from __future__ import annotations

from typing import Sequence

from src.fea.analysis_model import AnalysisModel
from src.fea.constraints.sp import ConstraintHandler, PlainHandler
from src.fea.domain import Domain
from src.fea.numberer import DOF_Numberer, PlainNumberer
from src.fea.system.soe import LinearSOE
from src.fea.analysis.algorithm import SolutionAlgorithm, Linear
from src.fea.analysis.integrator import Integrator, LoadControl


class StaticAnalysis:
    def __init__(self,
                 domain: Domain,
                 constraint_handler: ConstraintHandler | None = None,
                 numberer: DOF_Numberer | None = None,
                 soe: LinearSOE | None = None,
                 integrator: Integrator | None = None,
                 algorithm: SolutionAlgorithm | None = None,
                 recorders: Sequence = ()) -> None:
        self.domain = domain
        self.constraint_handler = constraint_handler or PlainHandler()
        self.numberer = numberer or PlainNumberer()
        self.soe = soe or LinearSOE()
        self.integrator = integrator or LoadControl(1.0)
        self.algorithm = algorithm or Linear()
        self.recorders = list(recorders)
        self.model = AnalysisModel()

    def analyze(self, num_steps: int = 1) -> AnalysisModel:
        """Run `num_steps` load steps; returns the AnalysisModel (numbering +
        reaction access). Raises on solve failure."""
        self.constraint_handler.handle(self.domain, self.model, self.numberer)
        if self.model.num_eq == 0:
            raise ValueError("All DOFs are constrained — nothing to solve.")
        self.soe.set_size(self.model.num_eq)

        for _ in range(num_steps):
            self.integrator.new_step(self.domain, self.model)
            self.algorithm.solve_current_step(
                self.domain, self.model, self.integrator, self.soe)
            self.domain.commit()
            self.model.compute_reactions(self.domain)
            for recorder in self.recorders:
                recorder.record(self.domain, self.domain.current_time)
        return self.model
