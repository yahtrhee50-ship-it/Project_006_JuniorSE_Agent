import sys
from pathlib import Path

# Repo root on sys.path so `from src.fea import ...` works (same convention
# as the existing tests/).
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
