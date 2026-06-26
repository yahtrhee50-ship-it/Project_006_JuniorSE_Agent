# Junior SE Agent — Design Interview Transcript

**Status: IN PROGRESS — resume at Q3**
**Date started: 2026-06-25**
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

## Q3 — Where does it live / what is the interface? ← RESUME HERE

**Discussion so far:**

**Agreed architecture: Option C — Hybrid**
- New standalone agent (Project_006)
- Calls existing Project_005 SAP2000 REST API endpoints as tools (no need to rewrite COM integration)
- Builds its own toolkit for: load calcs, code lookups, document generation, hand calc verification

**Interface options discussed:**
- CLI script invoked with a task description (e.g., `python junior_se.py "check girder sizing"`)
- Claude Code subagent (spawned from within Claude Code sessions)
- New chat UI tab in the existing web app

**Decision: NOT YET MADE — resume here**

The user was shown what a CLI invocation looks like and asked if that changes their thinking. They tabled the discussion at this point to set up Project_006.

---

## Remaining Questions

- **Q3 (cont):** What interface? CLI, Claude Code subagent, or chat UI?
- **Q4:** What are the MVP capabilities? What's the first task you'd hand it?
- **Q5:** What file formats will it work with? (PDFs, DWG, Excel, hand calc sheets?)
- **Q6:** How should it document its work? (Markdown, PDF, structured JSON, Excel?)
- **Q7:** Should it have memory across sessions? (remember past projects and decisions?)

---

## Agreed Technical Decisions

| Decision | Choice | Reason |
|---|---|---|
| SAP2000 integration | Reuse Project_005 API | No need to rewrite COM layer |
| Supervision model | Human-in-the-loop (user = senior SE) | Agent proposes, user approves |
| Project location | Project_006_JuniorSE_Agent | New standalone project |
| Template source | `_TEMPLATE` folder | Standard project scaffolding |
