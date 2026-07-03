"""
Headless structural finite-element analysis engine (Phase 0+).

Architecture follows the OpenSees object model: a Domain describes what the
structure *is* (nodes, elements, materials, sections, constraints, load
patterns); an Analysis is assembled from interchangeable components
(ConstraintHandler, DOF_Numberer, SystemOfEqn+Solver, Integrator,
SolutionAlgorithm, ConvergenceTest) that decide *how* it is solved.

Units: unit-system-agnostic — supply inputs in any one consistent unit set
(e.g. kip/in/ksi or N/m/Pa) and results come back in that same set.

See docs/fea_engine_plan.md and docs/fea_engine_research_brief.md.
"""
from src.fea.domain import Domain, Node
from src.fea.api import Model

__all__ = ["Domain", "Node", "Model"]
