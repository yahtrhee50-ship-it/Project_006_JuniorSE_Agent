"""
statusline.py — Claude Code statusLine helper for Project_006_JuniorSE_Agent

Reads the Claude Code JSON payload from stdin and prints a context-window
progress bar with ANSI color coding:
  - Green  (< 50% used)
  - Yellow (50-80% used)
  - Red    (> 80% used)

Context usage is computed from the session transcript (transcript_path in the
payload), summing the token counts on the most recent assistant turn. This is
the field Claude Code actually provides; there is no context_window percentage
in the statusline payload.

Usage (in ~/.claude/settings.json):
  "statusLine": {
    "type": "command",
    "command": "C:\\Python314\\python.exe D:\\AI_TEST\\Agent_Developer\\Project_006_JuniorSE_Agent\\statusline.py"
  }
"""

import sys
import json
import os

# Force UTF-8 output so the block-bar characters render on Windows
# (default console encoding is cp1252, which cannot encode them).
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

# ANSI color codes
GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
DIM    = "\033[2m"
RESET  = "\033[0m"

BAR_WIDTH = 10
FILL_CHAR  = "█"  # full block
EMPTY_CHAR = "░"  # light shade

# Default context window (tokens). Claude models are 200k unless the 1M beta
# is active; we detect the larger window from exceeds_200k_tokens when needed.
DEFAULT_CONTEXT = 200_000


def make_bar(used_pct: float, width: int = BAR_WIDTH) -> str:
    filled = max(0, min(width, round(used_pct / 100.0 * width)))
    return FILL_CHAR * filled + EMPTY_CHAR * (width - filled)


def pick_color(used_pct: float) -> str:
    if used_pct < 50:
        return GREEN
    if used_pct < 80:
        return YELLOW
    return RED


def context_used_tokens(transcript_path: str):
    """Return the token count in context from the latest assistant usage entry.

    Sums input_tokens + cache_creation_input_tokens + cache_read_input_tokens,
    which together represent everything currently in the context window.
    Returns None if it cannot be determined.
    """
    if not transcript_path or not os.path.exists(transcript_path):
        return None
    latest = None
    try:
        with open(transcript_path, "r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line or '"usage"' not in line:
                    continue
                try:
                    rec = json.loads(line)
                except Exception:
                    continue
                usage = (rec.get("message") or {}).get("usage") or rec.get("usage")
                if not isinstance(usage, dict):
                    continue
                total = (
                    (usage.get("input_tokens") or 0)
                    + (usage.get("cache_creation_input_tokens") or 0)
                    + (usage.get("cache_read_input_tokens") or 0)
                )
                if total > 0:
                    latest = total
    except Exception:
        return None
    return latest


def render(used_pct, model_name):
    label = f"{used_pct:.0f}%" if used_pct is not None else "--%"
    pct_for_bar = used_pct if used_pct is not None else 0.0
    bar = make_bar(pct_for_bar)
    color = pick_color(pct_for_bar) if used_pct is not None else GREEN
    name = f" {DIM}{model_name}{RESET}" if model_name else ""
    sys.stdout.write(f"{color}{bar} {label}{RESET}{name}")
    sys.stdout.flush()


def main() -> None:
    raw = sys.stdin.read() if not sys.stdin.isatty() else ""
    try:
        data = json.loads(raw) if raw.strip() else {}
    except Exception:
        data = {}

    model_name = (data.get("model") or {}).get("display_name") or ""

    # Preferred: an explicit percentage if a future payload ever provides one.
    used_pct = (data.get("context_window") or {}).get("used_percentage")

    if used_pct is None:
        used = context_used_tokens(data.get("transcript_path"))
        if used is not None:
            limit = 1_000_000 if data.get("exceeds_200k_tokens") else DEFAULT_CONTEXT
            used_pct = min(100.0, used / limit * 100.0)

    render(used_pct, model_name)


if __name__ == "__main__":
    main()
