"""
AISC section database query module.
All values are imperial units: in, in², in³, in⁴, kip/ft.
Source: AISC Shapes Database v16.0
"""
import json
from pathlib import Path
from typing import Optional

_DB_PATH = Path(__file__).parent.parent.parent / "data" / "AISC" / "sections_v16.json"

# Loaded once on first access, keyed by uppercase label (e.g. "W16X40")
_db: dict[str, dict] = {}


def _load() -> None:
    if _db:
        return
    with open(_DB_PATH, encoding="utf-8") as f:
        records = json.load(f)
    for r in records:
        label = r.get("AISC_Manual_Label")
        if label:
            _db[label.upper()] = r


def get_section(label: str) -> dict:
    """Return section property dict for an AISC designation.

    Args:
        label: AISC designation, e.g. 'W16X40' (case-insensitive)

    Returns:
        Dict with keys: Type, AISC_Manual_Label, W, A, d, tw, bf, tf, kdes,
        Ix, Zx, Sx, rx, Iy, Zy, Sy, ry, J, Cw, bf/2tf, h/tw

    Raises:
        ValueError: if the designation is not found in the database
    """
    _load()
    key = label.strip().upper()
    if key not in _db:
        raise ValueError(
            f"Section '{label}' not found in AISC v16.0 database. "
            f"Check designation (e.g. 'W16X40', 'HSS6X6X0.500')."
        )
    return _db[key]


def list_sections(section_type: Optional[str] = None) -> list[str]:
    """Return sorted list of AISC_Manual_Label values.

    Args:
        section_type: filter by type code, e.g. 'W', 'HSS', 'C', 'L'.
                      If None, returns all sections.
    """
    _load()
    if section_type:
        t = section_type.upper()
        return sorted(
            label for label, r in _db.items() if r.get("Type", "").upper() == t
        )
    return sorted(_db.keys())


def list_types() -> list[str]:
    """Return sorted list of unique section type codes in the database."""
    _load()
    return sorted({r.get("Type", "") for r in _db.values() if r.get("Type")})


def find_lightest_W(min_Zx_in3: float) -> dict:
    """Return lightest W-shape with plastic section modulus Zx >= min_Zx_in3.

    Useful for initial beam sizing: call with required Zx, get lightest
    adequate section.

    Args:
        min_Zx_in3: minimum required Zx (in³)

    Returns:
        Section property dict for the lightest qualifying W-shape.

    Raises:
        ValueError: if no W-shape in the database meets the requirement.
    """
    _load()
    candidates = [
        r for r in _db.values()
        if r.get("Type") == "W"
        and isinstance(r.get("Zx"), (int, float))
        and r["Zx"] >= min_Zx_in3
    ]
    if not candidates:
        raise ValueError(
            f"No W-shape in AISC v16.0 database has Zx >= {min_Zx_in3:.1f} in³."
        )
    return min(candidates, key=lambda r: r["W"])
