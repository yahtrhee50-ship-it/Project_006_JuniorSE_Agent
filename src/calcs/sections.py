"""
AISC section database query module.
All values are imperial units: in, in², in³, in⁴, kip/ft.
Sources:
  Modern:     AISC Shapes Database v16.0   (2,299 sections)
  Historical: AISC Shapes Database v16.0H  (18,877 sections, includes pre-1950 shapes)
"""
import json
from pathlib import Path
from typing import Optional

_DATA_DIR = Path(__file__).parent.parent.parent / "data" / "AISC"
_DB_PATHS = {
    "modern":     _DATA_DIR / "sections_v16.json",
    "historical": _DATA_DIR / "sections_v16h.json",
}

# Each database loaded once on first access, keyed by uppercase label
_dbs: dict[str, dict[str, dict]] = {"modern": {}, "historical": {}}


def _load(db: str) -> None:
    if _dbs[db]:
        return
    with open(_DB_PATHS[db], encoding="utf-8") as f:
        records = json.load(f)
    for r in records:
        label = r.get("AISC_Manual_Label")
        if label:
            _dbs[db][label.upper()] = r


def get_section(label: str, historical: bool = False) -> dict:
    """Return section property dict for an AISC designation.

    Args:
        label:      AISC designation, e.g. 'W16X40' (case-insensitive)
        historical: if True, search the v16.0H historical database instead

    Returns:
        Dict with keys: Type, AISC_Manual_Label, W, A, d, tw, bf, tf, kdes,
        OD, ID, tnom, tdes, Ix, Zx, Sx, rx, Iy, Zy, Sy, ry, J, Cw,
        bf/2tf, h/tw, D/t  (None for fields not applicable to the section type)

    Raises:
        ValueError: if the designation is not found
    """
    db = "historical" if historical else "modern"
    _load(db)
    key = label.strip().upper()
    if key not in _dbs[db]:
        which = "v16.0H historical" if historical else "v16.0 modern"
        raise ValueError(
            f"Section '{label}' not found in AISC {which} database. "
            f"Check designation (e.g. 'W16X40', 'PIPE6STD', 'B18.5X64' for historical)."
        )
    return _dbs[db][key]


def list_sections(section_type: Optional[str] = None, historical: bool = False) -> list[str]:
    """Return sorted list of AISC_Manual_Label values.

    Args:
        section_type: filter by type code, e.g. 'W', 'HSS', 'C', 'L', 'PIPE'.
                      If None, returns all sections.
        historical:   if True, search the v16.0H historical database
    """
    db = "historical" if historical else "modern"
    _load(db)
    if section_type:
        t = section_type.upper()
        return sorted(
            label for label, r in _dbs[db].items()
            if (r.get("Type") or "").upper() == t
        )
    return sorted(_dbs[db].keys())


def list_types(historical: bool = False) -> list[str]:
    """Return sorted list of unique section type codes in the database."""
    db = "historical" if historical else "modern"
    _load(db)
    return sorted({r.get("Type", "") for r in _dbs[db].values() if r.get("Type")})


def find_lightest_W(min_Zx_in3: float, historical: bool = False) -> dict:
    """Return lightest W-shape with plastic section modulus Zx >= min_Zx_in3.

    Args:
        min_Zx_in3: minimum required Zx (in³)
        historical:  if True, search the v16.0H historical database

    Returns:
        Section property dict for the lightest qualifying W-shape.

    Raises:
        ValueError: if no qualifying W-shape exists in the database.
    """
    db = "historical" if historical else "modern"
    _load(db)
    candidates = [
        r for r in _dbs[db].values()
        if r.get("Type") == "W"
        and isinstance(r.get("Zx"), (int, float))
        and r["Zx"] >= min_Zx_in3
    ]
    if not candidates:
        raise ValueError(
            f"No W-shape in AISC {'v16.0H' if historical else 'v16.0'} "
            f"database has Zx >= {min_Zx_in3:.1f} in³."
        )
    return min(candidates, key=lambda r: r["W"])
