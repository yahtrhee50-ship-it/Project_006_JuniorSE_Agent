"""
ASTM A615 / A706 standard deformed bar properties.
Nominal dimensions per ASTM A615/A706 Table 1.
"""
from __future__ import annotations
import re

_TABLE: dict[str, dict] = {
    "#3":  {"db_in": 0.375,  "Ab_in2": 0.11,  "wt_plf": 0.376},
    "#4":  {"db_in": 0.500,  "Ab_in2": 0.20,  "wt_plf": 0.668},
    "#5":  {"db_in": 0.625,  "Ab_in2": 0.31,  "wt_plf": 1.043},
    "#6":  {"db_in": 0.750,  "Ab_in2": 0.44,  "wt_plf": 1.502},
    "#7":  {"db_in": 0.875,  "Ab_in2": 0.60,  "wt_plf": 2.044},
    "#8":  {"db_in": 1.000,  "Ab_in2": 0.79,  "wt_plf": 2.670},
    "#9":  {"db_in": 1.128,  "Ab_in2": 1.00,  "wt_plf": 3.400},
    "#10": {"db_in": 1.270,  "Ab_in2": 1.27,  "wt_plf": 4.303},
    "#11": {"db_in": 1.410,  "Ab_in2": 1.56,  "wt_plf": 5.313},
    "#14": {"db_in": 1.693,  "Ab_in2": 2.25,  "wt_plf": 7.650},
    "#18": {"db_in": 2.257,  "Ab_in2": 4.00,  "wt_plf": 13.600},
}

DESIGNATIONS: list[str] = list(_TABLE.keys())

# Bars eligible for headed (T-head) anchors per §25.4.4: #3 through #11
THEAD_ELIGIBLE: list[str] = ["#3","#4","#5","#6","#7","#8","#9","#10","#11"]


def get_bar(designation: str) -> dict:
    """Return a copy of properties dict for the given bar designation.

    Keys: db_in (diameter, in), Ab_in2 (area, in²), wt_plf (weight, lb/ft).

    Accepts '#8', '8', '#10', '10', etc.
    """
    key = designation.strip()
    if not key.startswith("#"):
        key = "#" + key
    if key not in _TABLE:
        raise ValueError(
            f"Unknown bar designation {designation!r}. "
            f"Valid: {', '.join(DESIGNATIONS)}"
        )
    return dict(_TABLE[key])


def parse_bar_string(s: str) -> tuple[int, str]:
    """Parse a bar-count string like '4#8' or '3 #10' into (count, '#8').

    Accepts: '4#8', '3#10', '4 #8', '4-#8', '4 - #8'.
    """
    m = re.fullmatch(r"\s*(\d+)\s*[-\s]*(#\s*\d+)\s*", s)
    if not m:
        raise ValueError(
            f"Cannot parse bar string {s!r}. "
            "Expected format '4#8', '3#10', '4 #8', etc."
        )
    n    = int(m.group(1))
    bar  = "#" + m.group(2).replace("#", "").replace(" ", "")
    if bar not in _TABLE:
        raise ValueError(f"Bar {bar!r} parsed from {s!r} is not in the rebar table.")
    return n, bar
