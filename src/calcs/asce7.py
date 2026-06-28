"""
ASCE 7-22 LRFD load combinations (strength design).

Combinations 1-5 are the basic combinations of §2.3.1; combinations 6-7 are the
basic seismic combinations of §2.3.6 (E supplied directly as a load effect — the
0.2*S_DS dead-load adjustment and Ev/Eh expansion are the caller's responsibility
and are out of scope this phase).

Loads are passed as magnitudes in any single consistent unit (e.g. kip/ft for a
distributed beam load, or kips/ksf for a point/area effect); the factored result is
returned in that same unit. Keys (all optional, missing -> 0.0):
    D   dead          L   live           Lr  roof live
    S   snow          R   rain           W   wind (1.0W, strength level)
    E   seismic       (E supplied as a strength-level effect)

Scope notes:
  - Live-load reduction (§4.7) is NOT applied here — pass already-reduced L if desired.
  - W and E magnitudes are treated as additive (gravity-direction) effects; load
    reversal / uplift sign handling is the caller's responsibility.
"""
from __future__ import annotations
from dataclasses import dataclass

_KEYS = ("D", "L", "Lr", "S", "R", "W", "E")


@dataclass
class Combination:
    name:     str     # short id, e.g. "LC2"
    equation: str     # symbolic form, e.g. "1.2D + 1.6L + 0.5(Lr or S or R)"
    factored: float   # factored load effect in the input unit
    code_ref: str     # ASCE 7-22 section reference


def _norm(loads: dict) -> dict:
    """Return a dict with all seven keys present (missing -> 0.0)."""
    out = {k: 0.0 for k in _KEYS}
    for k, v in (loads or {}).items():
        if k not in out:
            raise ValueError(
                f"Unknown load key {k!r}. Valid keys: {', '.join(_KEYS)}."
            )
        if v is None:
            continue
        if v < 0:
            raise ValueError(f"Load {k!r} must be >= 0, got {v}.")
        out[k] = float(v)
    return out


def lrfd_combinations(loads: dict) -> list[Combination]:
    """Return the seven ASCE 7-22 LRFD strength combinations for the given loads.

    Args:
        loads: dict of load magnitudes (consistent unit); keys among D,L,Lr,S,R,W,E.

    Returns:
        List of seven Combination objects, in code order (LC1..LC7).
    """
    q = _norm(loads)
    D, L, Lr, S, R, W, E = (q[k] for k in _KEYS)
    roof = max(Lr, S, R)   # governing roof live / snow / rain term, §2.3.1

    return [
        Combination(
            "LC1", "1.4D",
            1.4 * D, "ASCE 7-22 §2.3.1(1)"),
        Combination(
            "LC2", "1.2D + 1.6L + 0.5(Lr or S or R)",
            1.2 * D + 1.6 * L + 0.5 * roof, "ASCE 7-22 §2.3.1(2)"),
        Combination(
            "LC3", "1.2D + 1.6(Lr or S or R) + (L or 0.5W)",
            1.2 * D + 1.6 * roof + max(L, 0.5 * W), "ASCE 7-22 §2.3.1(3)"),
        Combination(
            "LC4", "1.2D + 1.0W + L + 0.5(Lr or S or R)",
            1.2 * D + 1.0 * W + L + 0.5 * roof, "ASCE 7-22 §2.3.1(4)"),
        Combination(
            "LC5", "0.9D + 1.0W",
            0.9 * D + 1.0 * W, "ASCE 7-22 §2.3.1(5)"),
        Combination(
            "LC6", "1.2D + 1.0E + L + 0.2S",
            1.2 * D + 1.0 * E + L + 0.2 * S, "ASCE 7-22 §2.3.6(6)"),
        Combination(
            "LC7", "0.9D + 1.0E",
            0.9 * D + 1.0 * E, "ASCE 7-22 §2.3.6(7)"),
    ]


def factored_envelope(loads: dict) -> tuple[float, Combination]:
    """Return the maximum factored load effect and the combination that governs.

    Args:
        loads: dict of load magnitudes (consistent unit).

    Returns:
        (max_factored, governing_Combination)
    """
    combos = lrfd_combinations(loads)
    governing = max(combos, key=lambda c: c.factored)
    return governing.factored, governing
