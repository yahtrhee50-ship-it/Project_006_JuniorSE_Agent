"""
statusline.py — Claude Code statusLine helper for Project_006_JuniorSE_Agent

Reads the Claude Code JSON payload from stdin and prints a context-window
progress bar with ANSI color coding:
  - Green  (< 20% used)
  - Yellow (20–70% used)
  - Red    (> 70% used)

Usage (in ~/.claude/settings.json):
  "statusLine": {
    "type": "command",
    "command": "C:\\Python314\\python.exe D:\\AI_TEST\\Agent_Developer\\Project_006_JuniorSE_Agent\\statusline.py"
  }
"""

import sys
import json

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
RESET  = "\033[0m"

BAR_WIDTH = 10
FILL_CHAR  = "█"  # █
EMPTY_CHAR = "░"  # ░


def make_bar(used_pct: float, width: int = BAR_WIDTH) -> str:
    filled = max(0, min(width, round(used_pct / 100.0 * width)))
    return FILL_CHAR * filled + EMPTY_CHAR * (width - filled)


def pick_color(used_pct: float) -> str:
    if used_pct < 20:
        return GREEN
    elif used_pct <= 70:
        return YELLOW
    else:
        return RED


def main() -> None:
    try:
        data = json.load(sys.stdin)
    except Exception:
        # If JSON cannot be parsed, emit nothing rather than crash.
        return

    ctx = data.get("context_window") or {}
    used_pct = ctx.get("used_percentage")

    if used_pct is None:
        # No API call made yet — show a neutral empty bar.
        print(f"{GREEN}{EMPTY_CHAR * BAR_WIDTH}  --%{RESET}", end="")
        return

    bar   = make_bar(used_pct)
    color = pick_color(used_pct)
    print(f"{color}{bar} {used_pct:.0f}%{RESET}", end="")


if __name__ == "__main__":
    main()
