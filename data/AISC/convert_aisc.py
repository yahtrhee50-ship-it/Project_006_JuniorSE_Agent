"""Convert AISC shapes database Excel files to compact JSON for token-efficient lookup."""
import json
import openpyxl

KEEP = [
    "Type", "AISC_Manual_Label",
    "W", "A", "d", "tw", "bf", "tf", "kdes",
    "Ix", "Zx", "Sx", "rx",
    "Iy", "Zy", "Sy", "ry",
    "J", "Cw",
    "bf/2tf", "h/tw",
]

def convert(xlsx_path, sheet_name, out_path):
    wb = openpyxl.load_workbook(xlsx_path, read_only=True, data_only=True)
    ws = wb[sheet_name]
    rows = ws.iter_rows(values_only=True)
    headers = list(next(rows))
    idx = {}
    for i, h in enumerate(headers):
        if h in KEEP and h not in idx:  # first occurrence only (imperial columns)
            idx[h] = i

    sections = []
    for row in rows:
        if not row[0]:
            continue
        record = {}
        for field in KEEP:
            if field not in idx:
                continue
            val = row[idx[field]]
            if isinstance(val, str) and val.strip() in ("", "—", "-"):
                val = None
            record[field] = val
        sections.append(record)

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(sections, f, separators=(",", ":"))
    print(f"Wrote {len(sections)} sections to {out_path}")

base = "d:/AI_TEST/Agent_Developer/Project_006_JuniorSE_Agent/data/AISC"

convert(
    f"{base}/aisc-shapes-database-v160-2.xlsx",
    "Database v16.0",
    f"{base}/sections_v16.json",
)

convert(
    f"{base}/aisc-shapes-database-v160h.xlsx",
    "Database v16.0H",
    f"{base}/sections_v16h.json",
)
