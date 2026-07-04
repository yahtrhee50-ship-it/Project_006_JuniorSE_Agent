"""
Load cases and combinations (plan §10.1, Phase 1: results-level).

A *load case* = one solved analysis with a chosen subset of the Domain's load
patterns active. Its full response (node displacements, reactions, element
end forces) is snapshotted into an immutable CaseResults.

A *combination* scales and sums CaseResults linearly — valid ONLY because
Phase 1 analysis is linear (superposition). Nonlinear combinations (Phase 4+)
must re-run the analysis with factored loads instead; `combine` is not the
tool for that.

An *envelope* takes componentwise min/max over any set of CaseResults (cases
or combinations), the way design checks consume LRFD combos.

Factor generation is deliberately code-agnostic here: pair this with
`src.calcs.asce7` (ASCE 7-22 LRFD factors) to build the factor dicts.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, Mapping, Tuple

import numpy as np

from src.fea.domain import Domain


@dataclass(frozen=True)
class CaseResults:
    """Frozen response snapshot of one solved load case (or combination).

    node_disp / reactions : {node_tag: array(ndf)}
    ele_force             : {ele_tag: array of GLOBAL end forces, element DOF order}
    """
    name: str
    node_disp: Dict[int, np.ndarray] = field(default_factory=dict)
    reactions: Dict[int, np.ndarray] = field(default_factory=dict)
    ele_force: Dict[int, np.ndarray] = field(default_factory=dict)


def snapshot(domain: Domain, name: str) -> CaseResults:
    """Capture the Domain's committed response as a CaseResults."""
    return CaseResults(
        name=name,
        node_disp={n.tag: n.get_committed_disp() for n in domain.nodes()},
        reactions={n.tag: n.reaction.copy() for n in domain.nodes()},
        ele_force={e.tag: np.asarray(e.get_resisting_force(), dtype=float).copy()
                   for e in domain.elements()},
    )


def combine(name: str, cases: Mapping[str, CaseResults],
            factors: Mapping[str, float]) -> CaseResults:
    """Linear combination sum(factor * case) — linear analysis only.

    Args:
        name: label for the result (e.g. "LC2: 1.2D + 1.6L").
        cases: solved cases keyed by case name.
        factors: {case_name: factor}; every named case must exist.
    """
    missing = [c for c in factors if c not in cases]
    if missing:
        raise KeyError(f"combine('{name}'): unknown case(s) {missing}; "
                       f"have {sorted(cases)}")
    if not factors:
        raise ValueError(f"combine('{name}'): empty factor map")

    first = cases[next(iter(factors))]
    node_disp = {t: np.zeros_like(v) for t, v in first.node_disp.items()}
    reactions = {t: np.zeros_like(v) for t, v in first.reactions.items()}
    ele_force = {t: np.zeros_like(v) for t, v in first.ele_force.items()}
    for cname, f in factors.items():
        case = cases[cname]
        for t in node_disp:
            node_disp[t] = node_disp[t] + f * case.node_disp[t]
        for t in reactions:
            reactions[t] = reactions[t] + f * case.reactions[t]
        for t in ele_force:
            ele_force[t] = ele_force[t] + f * case.ele_force[t]
    return CaseResults(name, node_disp, reactions, ele_force)


@dataclass(frozen=True)
class EnvelopeResults:
    """Componentwise extremes over a set of CaseResults; *_src holds the name
    of the case/combo governing each component."""
    node_disp_min: Dict[int, np.ndarray]
    node_disp_max: Dict[int, np.ndarray]
    reactions_min: Dict[int, np.ndarray]
    reactions_max: Dict[int, np.ndarray]
    ele_force_min: Dict[int, np.ndarray]
    ele_force_max: Dict[int, np.ndarray]
    node_disp_min_src: Dict[int, list]
    node_disp_max_src: Dict[int, list]
    reactions_min_src: Dict[int, list]
    reactions_max_src: Dict[int, list]
    ele_force_min_src: Dict[int, list]
    ele_force_max_src: Dict[int, list]


def envelope(results: Iterable[CaseResults]) -> EnvelopeResults:
    """Componentwise min/max over cases/combos, tracking the governing name."""
    results = list(results)
    if not results:
        raise ValueError("envelope(): no results given")

    def _env(getter) -> Tuple[dict, dict, dict, dict]:
        lo, hi, lo_src, hi_src = {}, {}, {}, {}
        for tag in getter(results[0]):
            stack = np.stack([getter(r)[tag] for r in results])
            imin = np.argmin(stack, axis=0)
            imax = np.argmax(stack, axis=0)
            lo[tag] = stack[imin, np.arange(stack.shape[1])]
            hi[tag] = stack[imax, np.arange(stack.shape[1])]
            lo_src[tag] = [results[int(i)].name for i in imin]
            hi_src[tag] = [results[int(i)].name for i in imax]
        return lo, hi, lo_src, hi_src

    d_lo, d_hi, d_lo_s, d_hi_s = _env(lambda r: r.node_disp)
    r_lo, r_hi, r_lo_s, r_hi_s = _env(lambda r: r.reactions)
    f_lo, f_hi, f_lo_s, f_hi_s = _env(lambda r: r.ele_force)
    return EnvelopeResults(d_lo, d_hi, r_lo, r_hi, f_lo, f_hi,
                           d_lo_s, d_hi_s, r_lo_s, r_hi_s, f_lo_s, f_hi_s)
