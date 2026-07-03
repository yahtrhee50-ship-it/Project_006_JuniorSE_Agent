---
title: Headless Structural Finite Element Analysis Engine — Final Plan
project: Project_006_JuniorSE_Agent
approach: Python-first (NumPy/SciPy), OpenSees object model
scope: Full general FE engine (incl. plates/shells), delivered phase by phase
first_deliverable: Phase 0 + Phase 1 (verified linear engine)
source_brief: docs/fea_engine_research_brief.md
status: approved
date: 2026-07-03
---

# Final Plan — Headless Structural Finite Element Analysis Engine

## Context

The draft `structural-fea-engine-plan.md` (a research brief, now to be committed into the
repo) argues for building the **analysis core of a program like SAP2000 — "SAP2000 without
the GUI"**: a headless engine that takes a structural model (nodes, elements, materials,
sections, loads, supports) and computes its response, growing from a rock-solid linear core
out to full nonlinear (geometric + material), dynamics, staged construction, and plate/shell
elements. The confirmed target scope is the **full general FE engine, including plates/shells**.

Why now / why here: this repo (`Project_006_JuniorSE_Agent`) already contains two hand-rolled
direct-stiffness solvers — `src/calcs/truss_stiffness.py` (2D plane truss, benchmarked against
Hibbeler Ch 14) and `src/calcs/beam_stiffness.py` (`GeneralBeam`, benchmarked against Hibbeler
Ch 15). They are **procedural, per-analysis-type, and not extensible** — adding frames, shells,
or any nonlinearity means a rewrite each time. The most recent session note already flagged the
next step as "build a Ch 16 frame solver following the same pattern," which is exactly the dead
end the draft warns against. The strategic decision from the draft is: **stop growing one-off
solvers and instead build one extensible framework** using the OpenSees object model (Domain /
Element / Material / Section / Constraint / LoadPattern, analyzed by a swappable aggregation of
ConstraintHandler / DOF_Numberer / SystemOfEqn+Solver / Integrator / SolutionAlgorithm /
ConvergenceTest). Every future capability then becomes a plug-in, not a rewrite.

Intended outcome of this plan: a new `src/fea/` package implementing that framework, delivered
**phase by phase** (each phase independently useful and fully verified before the next begins),
with the existing solvers + their passing Hibbeler benchmarks reused as regression oracles. This
plan details Phase 0–1 concretely (the immediate executable work) and lays out Phases 2–7 as the
roadmap. **This is a large, multi-increment effort** — the deliverable of *this* plan's first
execution is Phase 0 + Phase 1 (a verified linear engine), not the entire SAP2000 feature set.

Approach = the draft's **Path A** (Python-first with NumPy/SciPy, interfaces clean enough that a
later C++/Julia port is mechanical). This matches the repo (pure-Python, `numpy`, `pytest`) and
the AI-assisted workflow.

---

## Guiding principles (from the draft — non-negotiable)

1. **Separate model from analysis.** The Domain (what the structure *is*) never knows how it's
   solved; an Analysis is *assembled* from interchangeable component objects. (draft §2)
2. **Four contracts to get right early:** `Element` (returns resisting force + tangent stiffness,
   owns `commitState`/`revertToLastCommit`), `Material` (strain→stress+tangent, committable),
   `Integrator` (`formTangent`/`formResidual`), `SolutionAlgorithm` (drives the iteration). (§2)
3. **Consistent tangents everywhere.** Every element/material tangent must be the true derivative
   of its returned force/stress — verified by an automated finite-difference test. This single
   test prevents the #1 class of nonlinear-convergence bugs. (§5 note, §13.1, §15)
4. **Wrap, don't rewrite, numerical backends.** Linear solve + eigen-solve sit behind thin
   interfaces from day one (SciPy sparse now; CHOLMOD/MUMPS/PARDISO later). (§7, §12)
5. **Build the V&V suite alongside the code, not after.** Never advance a phase until its
   verification passes. (§13, §14)
6. **Deterministic numbering.** Stable, defined node/DOF ordering — no set-iteration order
   leaking into equation numbers (a real source of run-to-run drift). (§11.4)

---

## Repo integration decisions

- **New package `src/fea/`** holds the engine; it is independent of the existing procedural
  `src/calcs/` solvers, which remain untouched and become **regression oracles**, not building
  blocks (wrapping their procedural internals into the object model would be awkward — reimplement
  cleanly, validate against them).
- **Reuse as oracles / reference math (do not import — copy the verified formulas):**
  - `src/calcs/beam_stiffness.py`: `_bernoulli_4x4` (4×4 Euler-Bernoulli k), `_hermite_ints`
    (distributed-load equivalent nodal loads), the point-load fixed-end-force expressions, the
    hinge = extra-independent-rotation-DOF technique (`_assign_dofs`), and prescribed-displacement
    handling (`solve`, `_prescribed_v`).
  - `src/calcs/truss_stiffness.py`: `_member_k_global` (axial element in global coords), the
    skew-support transformation matrix `_transform_matrix`, and prestrain equivalent loads
    (`_member_F0`) for thermal/fabrication cases.
  - `tests/test_general_beam.py`, `tests/test_truss_stiffness.py`, `tests/benchmark_ch15_p*.py`
    + `Reference/chapter_14_models.json` and
    `Reference/hibbeler_chapter_15_node_member_model/…md` — the existing V&V harness and published
    Hibbeler answers, reused directly to validate the new Truss and Frame elements.
  - **Note:** Ch 16 (frame) benchmark data is **not committed** in this repo. Phase-1 frame
    validation uses closed-form benchmarks + cross-check against `GeneralBeam`; committing Ch 16
    data (or regenerating it) is a Phase-1 sub-task if textbook frame benchmarks are wanted.
- **MCP surface:** add one tool `solve_model` in `src/mcp_server.py` mirroring the existing
  `solve_truss` / `solve_beam` shape — it accepts a Tier-1 model (JSON/script) and returns
  displacements, reactions, and element forces. Existing tools stay as-is.
- **Dependencies:** add `scipy` to `requirements.txt` (sparse direct solve + `eigsh` for later
  modal/buckling); `numpy`/`pytest` already present. The engine defaults to a numpy dense solver
  so it runs with no new dependency; the scipy sparse path is an optional lazy import.
  `openseespy` is an **optional** dev extra for cross-validation (§13.5) — network-gated install,
  tests that use it must `skip` cleanly when it is absent.
- **Units:** the engine is **unit-system-agnostic (consistent units in → consistent units out)**,
  unlike the imperial-only `calcs/` modules — this is required for general FE and for the SI
  Hibbeler benchmarks. Document the "pick one consistent set" rule in the API.

---

## Phase 0 — Skeleton & core abstractions

Goal: it runs, assembles a global system, and solves one trivial model through the full
component stack. (draft §14 Phase 0, §2, §3)

Create `src/fea/` with:

```
src/fea/
  __init__.py
  domain.py            # Domain (tagged storage of all model objects) + Node (geom, ndf DOFs, mass, response)
  numberer.py          # DOF_Numberer (Plain first; RCM in Phase 1); builds per-node equation numbers
  analysis_model.py    # AnalysisModel: Domain<->equations bridge; owns numbering; free DOF >=0, constrained <0
  elements/
    __init__.py
    element.py         # Element ABC: get_tangent_stiff(), get_resisting_force(), get_dof_map(),
                       #   commit_state(), revert_to_last_commit()
  materials/
    __init__.py
    uniaxial.py        # UniaxialMaterial ABC (set_trial_strain->stress+tangent, commit/revert) + Elastic
  sections/
    __init__.py
    section.py         # Section ABC + ElasticSection (EA, EI, GJ, GA)
  constraints/
    __init__.py
    sp.py              # SP_Constraint (fix/prescribe a single DOF) + ConstraintHandler ABC + PlainHandler (SP elimination)
  loads/
    __init__.py
    pattern.py         # LoadPattern + TimeSeries (Constant/Linear); NodalLoad; (element loads in Phase 1)
  system/
    __init__.py
    soe.py             # SystemOfEqn + Solver interface; DenseSolver (numpy) + SparseSolver (scipy.sparse.linalg.spsolve)
  analysis/
    __init__.py
    integrator.py      # Integrator ABC + LoadControl (static): form_tangent(), form_unbalance()
    algorithm.py       # SolutionAlgorithm ABC + Linear
    convergence.py     # ConvergenceTest ABC (norms) — trivial pass for Linear
    static_analysis.py # StaticAnalysis: aggregates handler+numberer+soe+integrator+algorithm; analyze(steps)
  recorders/
    __init__.py
    recorder.py        # Recorder ABC + NodeRecorder / ElementRecorder (to memory; disk later)
  api.py               # Tier-1 imperative scripting API (OpenSeesPy-style): node/element/section/
                       #   material/fix/pattern/load + analysis builder calls, all populating one Domain
```

Key implementation points:
- **Assembly / DOF mapping is the thing to make bulletproof** (draft §3): each element exposes an
  `ID`/location array of global equation numbers; `AnalysisModel` scatters `K_e` into the global
  matrix and gathers `u` back. Represent it explicitly and deterministically.
- Node carries a fixed `ndf` (number of DOFs); the framework is dimension-agnostic (2D nodes ndf=2/3,
  3D frame nodes ndf=6). Don't hard-code 2D.
- Only `LinearStatic` need exist; `LoadControl` + `Linear` algorithm + trivial convergence.

Phase 0 deliverable: `python -c` smoke script builds a 2-node model through `api.py`, runs
`StaticAnalysis`, and returns a displacement. Unit tests: Domain add/get by tag; deterministic
numbering; assembly scatter/gather round-trips.

---

## Phase 1 — Linear statics MVP (the first genuinely useful engine)

Goal: a usable linear structural analysis program that matches closed-form and the existing
solvers to machine precision. (draft §14 Phase 1, §4.1–4.2, §5.3/5.5, §10.1)

1. **Elements** (`src/fea/elements/`):
   - `truss.py` — linear elastic Truss (2/3D, axial only). Validate assembly + transformation.
     Oracle: `PlaneTruss` + `Reference/chapter_14_models.json` published answers.
   - `frame.py` — linear elastic **3D beam-column, 6 DOF/node** (axial + biaxial bending + torsion),
     Euler-Bernoulli via Hermitian cubics; local<->global transformation. 2D problems are the
     constrained case (fix out-of-plane DOFs). Support member end **moment releases** (hinge) and
     **rigid end offsets**. Oracle for in-plane bending: `GeneralBeam` + Hibbeler Ch 15 benchmarks;
     reuse `_bernoulli_4x4` / `_hermite_ints` math.
2. **Constraints:** SP constraints (restraints + prescribed displacement/settlement) via
   `PlainHandler`. (Reuse the prescribed-displacement approach from `GeneralBeam.solve`.)
3. **Loads:** the **LoadPattern/case/combination data model** (§10.1) — nodal forces/moments;
   member distributed (UDL, trapezoidal), point, and thermal/prestrain loads → equivalent nodal
   loads (reuse `_hermite_ints`, point FEF, `_member_F0`); self-weight; support settlement. Load
   cases pair a pattern with an analysis; combinations combine results linearly / by envelope.
4. **Section property tool** (§5.5, elastic part): utility computing A, I₂/I₃, J, shear areas,
   centroid, principal axes for built-up/elastic sections — feeds `ElasticSection`. (Nonlinear
   moment-curvature / P-M deferred to Phase 5 with fiber sections.)
5. **Tier-2 archetypes** (`src/fea/archetypes.py`, §2): first generators that expand a terse
   description into a Tier-1 meshed model — `SimplySupportedBeam`, `Cantilever`, `PortalFrame`,
   `ContinuousBeam`. Each ships its own tiny structured mesher (a member → a line of frame
   elements). These are what the coding-AI front-end translates prose into.
6. **DOF ordering:** add **RCM** to `numberer.py` (cheap bandwidth reduction).
7. **MCP:** add `solve_model` to `src/mcp_server.py` (Tier-1 JSON model in → results text out),
   mirroring `solve_truss`.

Phase 1 deliverable: matches closed-form cantilever `PL³/3EI`, simple-beam `5wL⁴/384EI`, a
hand-computed truss, and portal-frame results; and reproduces the existing `PlaneTruss` /
`GeneralBeam` + Hibbeler Ch 14/15 answers to machine precision. **This alone is a usable linear
FE program.**

---

## Phases 2–7 — roadmap (build order; detail deferred to each phase's kickoff)

Each maps directly to draft §14; expand into its own concrete plan when reached. Do not start a
phase until the prior phase's verification suite passes.

- **Phase 2 — Constraints, shells, solids, solver upgrade.** MP constraints (rigid diaphragm,
  equalDOF) via a **transformation handler**; MITC4 shell (membrane+bending); hex/tet solids;
  a **structured/mapped conforming mesher** with quality checks (aspect ratio, **positive
  Jacobian**) + edge-constraint stitching; **patch tests**; swap in CHOLMOD/MUMPS via SciPy/
  bindings for large models; section cuts + generalized displacements. (§4.3–4.4, §6, §7, §10.5, §11)
- **Phase 3 — Modal & linear dynamics.** Lumped/consistent mass; generalized eigensolver
  (`scipy…eigsh`/ARPACK, shift-invert, auto-shift); participation factors/effective mass; Rayleigh
  damping; modal time-history + response-spectrum (SRSS/CQC); influence lines & moving-load
  enveloping via **factorize-once/multi-RHS**; optional steady-state/frequency-domain. (§9.1–9.4,
  §9.7, §10.3)
- **Phase 4 — Geometric nonlinearity.** Geometric stiffness / **P-Delta**; **corotational**
  transformation for large displacement; LoadControl + **Newton-Raphson** + convergence tests +
  **line search**; eigenvalue buckling `(K+λK_g)φ=0`. (§8.5, §8.2, §9.5)
- **Phase 5 — Material nonlinearity.** Uniaxial nonlinear materials; **fiber sections**;
  **force-based** (and displacement-based) nonlinear frame elements; **J2 plasticity NDMaterial
  with consistent tangent** (radial return); Mander confined/unconfined concrete + steel–concrete
  **bond-slip**; layered nonlinear shell; nonlinear links (gap/hook/multilinear/damper/isolator);
  **displacement control + pushover**; nonlinear moment-curvature / P-M tool; energy accounting;
  **fracture-energy/crack-band regularization** for mesh-objective softening. (§4.5, §5.1–5.6,
  §8.3, §9.8, §11.4)
- **Phase 6 — Advanced solution & path-following.** **Arc-length** (Riks/Crisfield); automatic
  step-size cutting + event-to-event; quasi-Newton/BFGS; nonlinear **direct-integration**
  time-history (Newmark, HHT); nonlinear foundation springs + compression-only/uplift supports.
  (§8.3, §8.6, §9.3, §10.4)
- **Phase 7 — Staged construction & productionization.** Element activation/deactivation with
  state carry-forward; sequential-construction integrator; time-dependent concrete (creep/
  shrinkage/aging); **tendons/prestress** with full loss chain + target-force iteration; catenary
  cables; the **deterioration overlay** (corrosion section loss, ductility/bond degradation,
  cover-fiber spalling, confinement loss) on fiber sections; automatic code-based seismic/wind/
  wave/vehicle load generation; unstructured meshing by wrapping Gmsh/Netgen/TetGen; performance/
  parallel tuning; stable scripting API + model I/O. (§4.6, §5.4, §5.6, §9.6, §10.2, §11.5)
- **(Later, separable) Design-check modules** — AISC/ACI/Eurocode post-processors on results;
  outside the analysis core. (The repo's existing `aisc360.py`/`aci318.py` slot in here.)

---

## Verification & validation (build alongside, per §13; gate every phase)

Wire all of this into `tests/fea/` and run under `pytest` (existing convention).

- **Tier 1 — math unit tests:** shape functions partition of unity; correct Jacobian; element `K`
  symmetric; unconstrained element has exactly the right number of rigid-body (zero-energy) modes
  (check eigenvalues); **every material/element tangent equals a finite-difference of its own
  force output** (the highest-value single test — add it in Phase 0 as a reusable helper).
- **Tier 2 — patch tests** (Phase 2+): constant-strain field reproduced exactly by shells/solids.
- **Tier 3 — closed-form benchmarks:** cantilever `PL³/3EI`, simple beam `5wL⁴/384EI`, Euler
  buckling `π²EI/L²` (Phase 4), SDOF natural frequency + transient (Phase 3).
- **Tier 4 — regression against existing solvers (this repo's advantage):** run identical models
  through `PlaneTruss` / `GeneralBeam` and assert agreement; reuse the passing Hibbeler Ch 14/15
  benchmark inputs and published answers already in `tests/` + `Reference/`.
- **Tier 5 — cross-validation against OpenSees** (optional dev extra): identical models in
  OpenSeesPy; compare displacements/forces/frequencies. Tests `pytest.importorskip("openseespy")`
  so CI stays green without it.
- **Tier 6 — convergence studies** (Phase 2/5): mesh refinement converges at the theoretical rate;
  softening stays mesh-objective after regularization.

Pitfall watchlist to encode as tests/asserts (draft §15): inconsistent tangents, locking/
hourglassing, mesh non-conformity + softening mesh-dependence, wrong rigid-body-mode count,
penalty-constraint ill-conditioning, large-rotation (non-vector) handling, state committed only
on convergence, mass/unit consistency (frequencies off by clean factors).

---

## How to verify this plan's first deliverable (end-to-end, Phase 0 → Phase 1)

1. **Add deps:** append `scipy` to `requirements.txt`; install into the project interpreter
   (optional — the engine runs on numpy alone; scipy enables the sparse path).
2. **Run the suite:** `python -m pytest tests/fea -q` — all Tier-1 math tests, closed-form
   benchmarks, and Tier-4 regression-vs-`GeneralBeam`/`PlaneTruss` tests green.
3. **Smoke-drive the Tier-1 API** (a script under `tests/fea/` or a scratch file): build a
   10-ft steel cantilever with a tip load via `fea.api`, run `StaticAnalysis`, and assert tip
   deflection == `PL³/3EI` to ~1e-9.
4. **Cross-check a real case:** reproduce one Hibbeler Ch 15 continuous-beam benchmark through the
   new frame element and assert it matches the committed published reactions/moments (and the
   `GeneralBeam` result).
5. **Exercise the MCP tool:** call `solve_model` with a small JSON portal frame and confirm the
   returned displacements/reactions satisfy statics (ΣF, ΣM ≈ 0) — the draft's "sanity-check
   reactions against statics before trusting any result" gate (§15).
6. **(Optional) OpenSees cross-check:** if `openseespy` is available, run the same portal frame
   and confirm agreement; otherwise the test skips.

Commit the draft brief into the repo (e.g., `docs/fea_engine_research_brief.md`) as the standing
reference for Phases 2–7, and follow the repo's git convention (stage by name, descriptive
message, push to `master`) per `CLAUDE.md`.
