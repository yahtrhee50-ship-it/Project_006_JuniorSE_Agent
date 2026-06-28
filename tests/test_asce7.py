"""Unit tests for ASCE 7-22 LRFD load combinations (src/calcs/asce7.py)."""
import math
import pytest

from src.calcs import asce7


def gov(loads):
    return asce7.factored_envelope(loads)


def test_lc2_governs_typical_gravity():
    # 1.2D + 1.6L = 1.2*2 + 1.6*1 = 4.0 governs
    w, c = gov({"D": 2, "L": 1})
    assert c.name == "LC2"
    assert math.isclose(w, 4.0)


def test_dead_heavy_1p4D_governs():
    # 1.4D = 14.0 beats 1.2D+1.6L = 13.6
    w, c = gov({"D": 10, "L": 1})
    assert c.name == "LC1"
    assert math.isclose(w, 14.0)


def test_roof_term_lc3_governs():
    # 1.2D + 1.6Lr = 1.2 + 8.0 = 9.2 governs
    w, c = gov({"D": 1, "Lr": 5})
    assert c.name == "LC3"
    assert math.isclose(w, 9.2)


def test_roof_uses_max_of_lr_s_r():
    # snow term enters max(Lr,S,R): LC3 = 1.2 + 1.6*4 = 7.6
    w, c = gov({"D": 1, "S": 4})
    assert math.isclose(w, 7.6)


def test_all_seven_combinations_present():
    combos = asce7.lrfd_combinations({"D": 1})
    assert [c.name for c in combos] == ["LC1", "LC2", "LC3", "LC4", "LC5", "LC6", "LC7"]
    assert all(c.code_ref.startswith("ASCE 7-22") for c in combos)


def test_missing_keys_default_zero():
    w, c = gov({"D": 5})
    assert math.isclose(w, 7.0)  # 1.4*5


def test_unknown_key_rejected():
    with pytest.raises(ValueError):
        asce7.lrfd_combinations({"X": 1})


def test_negative_load_rejected():
    with pytest.raises(ValueError):
        asce7.lrfd_combinations({"D": -1})
