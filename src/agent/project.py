"""
Lean project memory / audit-trail helper for the Junior SE agent.

A "project" is a folder under Projects/<name>/ scaffolded from Projects/_TEMPLATE/:
    project.json            always-load tier (name, risk category, codes, key values)
    input/                  user-supplied input files
    members/inventory.json  running member inventory (beams / columns / girders)
    calcs/                  dated calc outputs (audit trail) — never auto-loaded
    sessions/               compact session summaries

This module only does file plumbing; all engineering math lives in src/calcs/.
"""
from __future__ import annotations
import json
import re
import shutil
from datetime import date
from pathlib import Path

_PROJECTS_DIR = Path(__file__).parent.parent.parent / "Projects"
_TEMPLATE_DIR = _PROJECTS_DIR / "_TEMPLATE"
_DEFAULT_PROJECT = "DEFAULT"
_VALID_BUCKETS = ("beams", "columns", "girders")

_SLUG_RE = re.compile(r"[^A-Za-z0-9._-]+")


def _slug(text: str) -> str:
    """Filesystem-safe token (letters, digits, dot, underscore, hyphen)."""
    s = _SLUG_RE.sub("_", str(text).strip()).strip("_")
    return s or "unnamed"


def resolve_project_dir(name: str | None = None) -> Path:
    """Return the project directory, scaffolding it from _TEMPLATE on first use.

    Args:
        name: project name; defaults to 'DEFAULT' if None/empty.

    Returns:
        Path to Projects/<name>/ (created if it did not exist).
    """
    proj = _slug(name) if name else _DEFAULT_PROJECT
    if proj == "_TEMPLATE":
        raise ValueError("'_TEMPLATE' is reserved; choose another project name.")
    dest = _PROJECTS_DIR / proj
    if not dest.exists():
        shutil.copytree(_TEMPLATE_DIR, dest)
        # stamp the project name into project.json
        pj = dest / "project.json"
        data = json.loads(pj.read_text(encoding="utf-8"))
        data["project"] = proj
        pj.write_text(json.dumps(data, indent=2), encoding="utf-8")
    return dest


def load_project_json(project_dir: Path) -> dict:
    """Load and return the always-load project.json for a project directory."""
    return json.loads((Path(project_dir) / "project.json").read_text(encoding="utf-8"))


def write_calc(project_dir: Path, member: str, markdown: str, csv: str | None = None) -> Path:
    """Write a dated calc output to calcs/ (and optional sibling .csv). Never overwrites.

    Args:
        project_dir: project directory (from resolve_project_dir).
        member:      member id used in the filename, e.g. 'B1' or 'W16X40'.
        markdown:    full calc markdown to archive.
        csv:         optional CSV/TSV diagram block written to a sibling .csv file.

    Returns:
        Path to the written .md file.
    """
    calcs = Path(project_dir) / "calcs"
    calcs.mkdir(exist_ok=True)
    stem = f"{date.today().isoformat()}_{_slug(member)}"

    md_path = calcs / f"{stem}.md"
    counter = 2
    while md_path.exists():
        md_path = calcs / f"{stem}_{counter}.md"
        counter += 1

    md_path.write_text(markdown, encoding="utf-8")
    if csv is not None:
        md_path.with_suffix(".csv").write_text(csv, encoding="utf-8")
    return md_path


def append_member(project_dir: Path, member_id: str, record: dict, bucket: str = "beams") -> None:
    """Insert/update a member record in members/inventory.json.

    Args:
        project_dir: project directory.
        member_id:   key for the member (e.g. 'B1').
        record:      dict of member properties to store.
        bucket:      one of 'beams', 'columns', 'girders'.
    """
    if bucket not in _VALID_BUCKETS:
        raise ValueError(f"bucket must be one of {_VALID_BUCKETS}, got {bucket!r}.")
    inv_path = Path(project_dir) / "members" / "inventory.json"
    inv = json.loads(inv_path.read_text(encoding="utf-8")) if inv_path.exists() else {}
    inv.setdefault(bucket, {})[member_id] = record
    inv_path.parent.mkdir(exist_ok=True)
    inv_path.write_text(json.dumps(inv, indent=2), encoding="utf-8")
