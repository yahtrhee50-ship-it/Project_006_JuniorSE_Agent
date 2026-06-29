# Junior Structural Engineer Agent — Claude Instructions

## Project Overview
This project builds a "Junior Structural Engineer" AI agent that operates under the supervision of a licensed senior engineer (the user). It extends Project_005 (SAP2000 AI Model Builder) by calling its REST API as a tool, rather than rewriting SAP2000 COM integration from scratch.

## Agent Rules

- Keep all work inside this project folder unless explicitly approved otherwise.
- Do not create, modify, move, or delete files outside this project folder without approval.
- Do not say a task or project is complete until the relevant files and logic have been verified.
- Do not mark any test, validation, or verification item as passed unless it was actually run or directly confirmed by the user.
- Do not claim integration tests passed unless a real integration test was actually run.
- Do not claim unit tests passed unless a real unit test was actually run.
- If only a syntax check was run, describe it as a syntax check — not a full test suite.
- If the user manually verifies behavior in a real browser or app, record that as manual verification.
- Tell the user what files changed.
- If making assumptions, state them clearly.

## Context Window Management

Monitor context usage throughout each session and manage it proactively:

- **Target pause zone: 30–40% context used.** When context reaches this range AND the current
  work is at a clean transition point (task complete, no background process running, no
  multi-step sequence in flight), stop, write a session log, and tell the user to run `/clear`.
- **Hard ceiling: 45%.** If this is reached before a natural stopping point, finish the
  current atomic action (e.g., complete the commit in progress, finish the tool call), then
  immediately stop, write a session log, and tell the user to run `/clear`.
- **Do not interrupt:** a multi-step calc sequence, an open git commit, a mid-stream MCP tool
  call, or any operation where stopping would leave the project in a broken or inconsistent
  state. Complete the atomic unit first, then pause.
- **Session log:** write progress to `docs/sessions/YYYY-MM-DD_session.md` (today's date)
  before asking the user to clear. Include: what was completed, what is in progress, and the
  next step to resume after `/clear`.
- **Notify clearly:** say "Context is approaching the limit — I've logged progress to
  `docs/sessions/YYYY-MM-DD_session.md`. Please run `/clear` to start a fresh context, then
  tell me to resume."

## Git & GitHub

**Always use Git and GitHub for this project.**

After completing any task that changes files:
1. Stage the changed files by name (`git add <file> ...`)
2. Write a clean, descriptive commit message (what changed and why)
3. Commit locally
4. Push to GitHub (`git push origin master`)

**Tool paths:**
- Run `where git` before any git operations to confirm the path
- If `git` is not in PATH, use `D:\AI_TEST\GIT\Git\cmd\git.exe`
- GitHub CLI (`gh`): `C:\Program Files\GitHub CLI\gh.exe`
- Remote branch is `master` (not `main`)

## Environment

- OS: Windows 11, PowerShell primary shell
- Python: `C:\Python314\python.exe`
- SAP2000 API: Hosted at Project_005 (`D:\AI_TEST\Agent_Developer\Project_005_SAP2000api_v3`)
- Git: `D:\AI_TEST\GIT\Git\cmd\git.exe`
- GitHub CLI (`gh`): `C:\Program Files\GitHub CLI\gh.exe`

## Architecture

**This agent calls Project_005's REST API as a tool** — do not rewrite SAP2000 COM integration.

Key Project_005 endpoints to reuse:
- `POST /api/sap2000/build-from-json` — build a SAP2000 model from a StructuralModel dict
- `POST /api/sap2000/connect` — connect to/launch SAP2000
- `GET /api/sap2000/status` — check SAP2000 connection
- `POST /api/preview` — compute 3D preview from model dict (stateless)

## Design Intent (from initial interview)

The junior SE agent should be able to:
- Interpret drawings and identify load paths
- Calculate all load types (dead, live, wind, seismic, snow, soil, hydrostatic, equipment)
- Develop tributary areas and ASCE 7 load combinations
- Analyze basic structural elements (beams, columns, slabs, walls, foundations, connections)
- Use and sanity-check SAP2000 results
- Apply ASCE 7, ACI 318, AISC 360, NDS, and applicable masonry/existing-building standards
- Document assumptions, code provisions, DCRs, and limit states
- Clearly distinguish confirmed info / engineering assumptions / items needing investigation
- Prepare calculations, memos, sketches, and RFI responses
- Recognize its own limits and escalate to the senior engineer (user)

## Project Structure (to be built)

```
src/
├── agent/
│   ├── junior_se.py         Main agent entry point
│   ├── tools/               Tool definitions (SAP2000 API, load calcs, code lookup, etc.)
│   └── prompts/             System prompts and persona definition
├── calcs/                   Hand calculation modules
├── codes/                   Code reference data (ASCE 7, AISC 360, ACI 318, etc.)
└── reports/                 Report/document generation

docs/
├── INTERVIEW.md             Design interview transcript (resume here)
└── ...
```

## Interview Status
See `docs/INTERVIEW.md` — interview is in progress, resume at Q3 (interface decision).
