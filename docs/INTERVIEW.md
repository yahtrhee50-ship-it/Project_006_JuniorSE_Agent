# Junior SE Agent — Design Interview Transcript

**Status: COMPLETE**
**Date started: 2026-06-25**
**Date closed: 2026-06-26**
**Senior engineer (user): yahtrhee50-ship-it**

---

## Q1 — What should the agent do?

**Answer:** Full junior structural engineer scope, including:

- Interpret architectural, civil, and existing-condition drawings
- Identify gravity and lateral load paths
- Calculate dead, live, wind, seismic, snow, soil, hydrostatic, and equipment loads
- Develop tributary areas and prepare load combinations (ASCE 7)
- Analyze basic beams, columns, slabs, walls, foundations, connections, and simple framing
- Use hand calculations, spreadsheets, and software (SAP2000, ETABS, RISA, RAM)
- Independently sanity-check whether software results are reasonable
- Apply ASCE 7, ACI 318, AISC 360, NDS, and applicable masonry/existing-building standards
- Document assumptions, reference code provisions, calculate demand-to-capacity ratios (DCRs), identify controlling limit states
- Clearly distinguish: confirmed information / engineering assumptions / items requiring further investigation
- Prepare calculations, markups, sketches, structural plans, details, schedules, general notes, technical memos, inspection reports, and RFI responses
- Check dimensions, elevations, member sizes, reinforcement, connections, and drawing consistency
- Coordinate with architects, civil, geotech, MEP, contractors, fabricators, and owners
- Review submittals and shop drawings against design intent
- Document field observations, identify deviations or deterioration, flag safety/stability issues
- Ask focused technical questions, manage assigned tasks and deadlines
- Recognize the limits of its knowledge and authority — escalate to the senior engineer (user)

---

## Q2 — Who supervises it?

**Answer:** The **user** acts as the senior/licensed engineer. The agent completes well-defined assignments, documents its work, and defers major design decisions to the user.

---

## Q3 — Interface / Architecture

**Answer: AI coordinator + MCP tools (Claude Code as the interface)**

- Claude acts as the orchestrator/brain — decides which tools to call and in what order
- All structural calculations live in deterministic Python tool modules — Claude never does code math itself
- Tools are exposed via an MCP server connected to Claude Code
- User interacts directly through the Claude Code interface (no separate UI needed)
- Subagents optional later for parallelizing independent tasks (e.g., wind + seismic simultaneously)

**Stack:**
```
User (senior SE)
    └── Claude Code  ←── [MCP server with custom SE tools]
            └── subagent (optional, for parallel tasks)
```

**Why not pure subagent:** A subagent alone only provides general file/shell tools, not SE domain tools.
**Why not standalone app:** Claude Code is already the interface; MCP adds custom tools on top with no new UI.

---

## Q4 — MVP: What is the first task?

**Answer: Simple beam check — end-to-end workflow**

1. Input: span, loading (DL + LL), beam size (or agent selects one)
2. Calculate reactions, shear demand, moment demand
3. Check AISC 360 flexure and shear capacity (deterministic tool)
4. Report DCR, controlling limit state, code references
5. Output: formatted calc with CSV diagram data

This exercises the full pattern: interpret input → run deterministic calc → check against code → document result.

---

## Q5 — File formats

**Input:**
- Plain text / chat (primary — user types task in Claude Code)
- Excel load schedule dropped into project `input\` folder

**Output:**
- Markdown calc summary (renders in Claude Code, copyable)
- CSV/TSV block inside the Markdown for copy-paste into Excel to generate moment, shear, and deflection diagrams

**Why not PDF input:** Vision tokens are expensive; avoid unless no alternative exists.

---

## Q6 — Output format

**Answer: Markdown + embedded CSV**

Calc summary in Markdown, followed by a CSV code block:

```
x (ft), M (kip-ft), V (kips), δ (in)
0,      0.0,        45.0,      0.000
5,      187.5,      20.0,     -0.121
10,     300.0,      -5.0,     -0.214
```

User copies CSV block, pastes into Excel, inserts chart. No reformatting needed.

---

## Q7 — Memory across sessions

**Answer: File-based project folders, tiered memory, dense JSON**

**Structure:**
```
Project_006_JuniorSE_Agent\
└── Projects\
    └── [ProjectName]\
        ├── project.json       ← always-load tier
        ├── input\             ← user input files (Excel, task files)
        ├── members\           ← running member inventory
        ├── calcs\             ← dated calc outputs (audit trail)
        └── sessions\          ← compact session summaries (on-demand tier)
```

**Memory tiers:**
| Tier | Content | When loaded | Size |
|---|---|---|---|
| Always | project.json (name, RC, codes, key values) | Every call | ~100-200 tokens |
| On demand | members, session summaries | Only when relevant | varies |
| Archive | Full calc outputs in calcs\ | Never auto-loaded | unlimited |

**Optimizations:**
- Dense JSON format (not prose) — ~5x fewer tokens
- Task-scoped loading — only pull memory relevant to current task type
- Prompt caching — always-load tier cached at ~90% cost reduction
- Summarization — agent writes compact session summary at end of each conversation

---

## Additional Design Decisions

| Topic | Decision |
|---|---|
| Units | Imperial only (kips, ft, in, psf, ksf) |
| Code tools | One deterministic Python module per standard (AISC 360, ASCE 7, ACI 318, NDS) |
| Escalation | Agent presents all results; user reviews and approves. Escalation protocol refined over time. |
| Audit trail | Every calc output: date, inputs, code sections referenced, tool version, user review status |
| Ambiguous input | Agent always asks clarifying questions before proceeding |
| SAP2000 | Not used for simple calcs — deferred until needed |
| Scope limits | No stamping, no final design decisions, no issuing drawings — all go through user review |
| Self-weight | Always included automatically in DL unless user explicitly overrides |
| Analysis method | Matrix stiffness method — deterministic, auditable, verified against textbook solutions |
| Input validation | All tool inputs validated against engineering bounds before calc runs |
| AISC database | Two Excel files provided (v16.0 modern + historical) — converted to compact JSON |
| Textbook verification | User to provide textbook reference; first calc verified against known solution |

---

## Agreed Technical Decisions

| Decision | Choice | Reason |
|---|---|---|
| SAP2000 integration | Reuse Project_005 API when needed | No need to rewrite COM layer |
| Supervision model | Human-in-the-loop (user = senior SE) | Agent proposes, user approves |
| Project location | Project_006_JuniorSE_Agent | New standalone project |
| Interface | MCP server + Claude Code | Leverages existing tooling, no new UI |
| Calc engine | Deterministic Python tools | Reproducibility required for engineering |
| Memory | File-based project folders | Persistent, auditable, token-efficient |
| Output | Markdown + CSV | Renders in Claude Code, pastes into Excel |
