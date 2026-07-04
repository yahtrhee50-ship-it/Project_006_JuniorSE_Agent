---
title: FEA Engine Phase 2 — Constraints, Shells, Solids, Mesher, Solver Upgrade
project: Project_006_JuniorSE_Agent
parent_plan: docs/fea_engine_plan.md (Phase 2 roadmap entry)
source_brief: docs/fea_engine_research_brief.md (§4.3–4.4, §6, §7, §10.5, §11)
status: approved (engineer, 2026-07-04)
date: 2026-07-04
---

# FEA Engine Phase 2 — Concrete Plan

## Context

Phase 1 is complete and gated (commit `f364b9e`): linear Truss + 2D Frame (releases,
offsets), SP constraints, element loads, load cases/combos, RCM numberer, archetypes,
section property tool, `solve_model` MCP tool — 256 tests green, validated against all
Hibbeler Ch 14/15/16 benchmarks and the MacNeal-Harder beam suite.

Phase 2's deliverable per the master plan: **real building models (frames + floors +
walls) solve correctly.** That requires multi-point constraints, a shell element, solid
elements, a conforming mesher with quality gates, patch tests, a sparse-capable solve
path, and section-cut post-processing.

Two items deferred out of Phase 1 fold in naturally here:

- **Native skew nodal axes** (Ch 14 problems 14-15/16 were validated via a rotated-model
  workaround) — arrives with the transformation handler (Step 1).
- **3D frame upgrade (6 DOF/node)** — the master plan's Phase 1 called for a 3D frame;
  we deliberately shipped 2D/3-DOF because that's all Ch 16 needed. "Frames + floors"
  in one model makes 3D frames a Phase 2 co-requisite (Step 4). MacNeal-Harder
  out-of-plane + twist cases (already recorded as deferred in
  `Reference/macneal_harder_models.json`) become its acceptance tests.

Also honored here: **NAFEMS path C** (plane-stress continuum → NAFEMS LE1 elliptic
membrane), which the engineer has designated as required for SAP2000 adoption. The
plane-stress quad is the natural stepping stone to both the MITC4 membrane part and the
solids, so it is scheduled as its own step rather than left implicit.
(NAFEMS path A — dynamics — is Phase 3, not here.)

Units: the engine stays unit-agnostic. New closed-form tests are written in imperial
per the standing directive; published benchmark targets (NAFEMS, textbook SI) stay in
their source units.

---

## Build order

Each step lands with its tests green before the next begins. Steps 2→3→5 share the
isoparametric machinery, so that ordering is load-bearing; Step 1 is independent and
goes first because Steps 3/6 consume it (edge stitching, diaphragms).

### Step 1 — MP constraints via a transformation handler (+ native skew axes)

**Files:** `constraints/mp.py` (new), `analysis_model.py` (extend), `api.py` (extend).

- `MP_Constraint(retained_tag, constrained_tag, matrix, retained_dofs, constrained_dofs)`
  — general linear relation `u_c = C · u_r` (OpenSees shape). Convenience constructors:
  - `equal_dof(r, c, dofs)` — identity coupling.
  - `rigid_link(r, c, kind="beam"|"bar")` — 2D and 3D rigid links (translation +
    rotation×lever-arm coupling; the Frame `offset` math generalizes here).
  - `rigid_diaphragm(retained, constrained_nodes, plane)` — floor nodes slaved to a
    master's in-plane translations + rotation about the plane normal.
- **Enforcement = transformation method** (the brief's production-quality choice; skip
  the penalty pass — the elimination architecture is already in place for SPs):
  extend `AnalysisModel.build` so a slaved DOF maps not to one equation but to a list
  of `(equation, coeff)` terms. `id_for` scatter generalizes to scatter-with-
  coefficients: `K̂ = TᵀKT` assembled element-by-element, `u_c` recovered after solve.
  Free/SP DOFs keep the existing fast path (single eq number, coeff 1).
- **Skew nodal axes:** `Node.local_axes(angle)` / `Model.node(..., axes=...)` — a
  per-node rotation folded into the same per-DOF transformation terms. SPs at a skew
  node then constrain the *rotated* DOFs (inclined roller = plain `fix` of the rotated
  normal).
- Constraint hygiene checks (fail loudly): a DOF may not be slaved twice, slaved and
  SP-constrained, or slaved to another slaved DOF (no chains in v1); retained node must
  exist and be un-slaved.

**Verify (tests/fea/test_mp_constraints.py):**
- equalDOF splice of two collinear truss halves == single-mesh answer (machine precision).
- Rigid link vs Frame `offset_i/j` result — same model, same answer.
- Rigid diaphragm on a 2-column portal: both column tops share ux; story shear splits
  by stiffness; ΣF statics gate.
- Native skew roller reproduces Hibbeler 14-15/16 published answers **without** the
  rotated-model workaround (and matches the Phase 1 rotated-model values).
- FD tangent check still passes on a constrained model (transformation preserves
  consistency by construction — prove it anyway).

### Step 2 — Isoparametric machinery + plane-stress/plane-strain Quad4 (NAFEMS LE1)

**Files:** `elements/isoparam.py` (new — shared machinery, built once per the brief §4
"shared element machinery"), `materials/nd.py` (new — NDMaterial ABC + ElasticIsotropic
with plane_stress / plane_strain / axisym-later / solid3d D-matrices),
`elements/quad4.py` (new).

- `isoparam.py`: shape functions N(ξ,η) + derivatives for Q4 (and H8/Tet4 in Step 5),
  Jacobian, B-matrix builders, Gauss rules (1/2/3-point lines, 2×2, 2×2×2). Unit-tested
  directly: partition of unity, ∂N sums to zero, Jacobian of a unit square == I.
- `Quad4`: 4-node bilinear isoparametric quad, ndf=2 nodes, full 2×2 integration,
  thickness t. Element loads: body force + edge traction → consistent nodal loads.
  Stress recovery at Gauss points + extrapolation to nodes.

**Verify (tests/fea/test_quad4.py):**
- **Patch test (Tier-2 gate, first appearance):** arbitrary 5-element distorted patch
  under linear displacement field reproduces constant stress exactly.
- Rigid-body modes: single unconstrained Q4 has exactly 3 zero eigenvalues.
- FD tangent; K symmetric.
- Cantilever of Q4s vs beam theory with shear correction (document the expected
  bending-locking-ish softness of Q4 at coarse mesh; convergence study to the target —
  Tier-6 first appearance).
- **NAFEMS LE1 elliptic membrane** (target σyy = 92.7 MPa at point D): needs the
  mapped curved-edge mesher from Step 3 — the test lands there if mesher order wins,
  or here with a hand-built mesh. Machine-readable targets to
  `Reference/nafems_le1.json` with provenance notes (public secondary sources; the
  engineer may supply the paywalled NAFEMS P18 PDF into `Reference/NAFEMS/` later).

### Step 3 — Structured conforming mesher + quality gates + edge stitching

**Files:** `mesh.py` (new), consumed by archetypes and tests.

- Generators (all deterministic numbering, no set-iteration order): line (n segments),
  structured rect grid (quads), **transfinite/mapped quad patch** on four (possibly
  curved) boundary edges — this is what LE1's elliptic geometry needs — and box hex
  grid (Step 5 consumes).
- **Quality gate before analysis** (brief §11.3, non-negotiable): positive Jacobian at
  every integration point, aspect ratio, corner angles, quad warp (3D). API refuses to
  emit a failing mesh (raise with the offending element + metric).
- **Edge-constraint stitching** for unavoidable non-conforming interfaces: hanging-node
  DOFs slaved to linear interpolation of the coarse edge's end nodes — emitted as
  Step 1 `MP_Constraint`s. Documented as an approximation to keep away from regions of
  interest (brief §11.2).
- `Model.mesh_*` Tier-1 hooks + a slab/wall Tier-2 archetype (structured quad grid with
  supports and area load).

**Verify (tests/fea/test_mesh.py):**
- Conformity: every shared edge is node-identical; deterministic re-run gives identical
  numbering.
- Quality gate rejects an inverted/degenerate quad (negative Jacobian) and a sliver.
- Fine patch stitched to coarse patch via edge constraints: patch test passes across
  the tied interface for constant stress; graded-mesh cantilever stays within tolerance
  of the conforming answer.
- LE1 mesh-convergence sequence (if LE1 landed in Step 2 with a hand mesh, it upgrades
  here to the mapped mesher).

### Step 4 — 3D frame (6 DOF/node) upgrade

**Files:** `elements/frame.py` (extend or `frame3d.py` — decide at implementation by
diff size; keep the 2D element untouched either way as the regression baseline).

- Local 12×12: axial EA/L, torsion GJ/L, bending EIz (Hermite, existing `_bernoulli_4x4`)
  + bending EIy; standard 3D rotation from direction cosines + a `vecxz` orientation
  vector (OpenSees geomTransf convention). `ElasticSection` already carries Iy/J/G —
  they finally get used.
- Releases/offsets carry over (condensation + rigid-link transform generalize to 6 DOF).
- Element loads: uniform/point in global XYZ projected to local axes (Phase 1 pattern).

**Verify (tests/fea/test_frame3d.py + extend MacNeal-Harder):**
- 2D regression: any in-plane model run through the 3D element (out-of-plane DOFs
  fixed) matches the 2D element to machine precision — including a Ch 16 benchmark
  re-run.
- Closed form: cantilever tip torsion TL/GJ, out-of-plane bending PL³/3EIy, grid
  (grillage) problem with combined torsion+bending.
- **MacNeal-Harder out-of-plane + twist cases** (already in
  `Reference/macneal_harder_models.json` as deferred) now pass.
- FD tangent; rigid-body modes = 6.

### Step 5 — MITC4 shell (membrane + Mindlin bending, no shear locking)

**Files:** `elements/shell_mitc4.py` (new), reusing `isoparam.py` + `materials/nd.py`.

The hardest element in the phase — scheduled after quads (membrane part = Step 2) and
3D nodes (Step 4) exist.

- 4-node flat shell, ndf=6 nodes: **membrane** (Q4 plane stress) + **Mindlin-Reissner
  plate bending with MITC4 assumed transverse-shear interpolation** (the locking fix —
  tying points per Bathe/Dvorkin) + **drilling DOF** stabilized with a small artificial
  stiffness (scaled to the element, documented; Allman upgrade later if needed).
- Warped-geometry handling v1: project to the mean plane, warn above a warp tolerance
  (rely on the Step 3 quality gate).
- Loads: uniform surface pressure + self-weight → consistent nodal loads. Output:
  membrane forces + bending/twisting moments + transverse shears at Gauss points
  (SAP2000's F11/F22/F12/M11/M22/M12/V13/V23 set).

**Verify (tests/fea/test_shell_mitc4.py):**
- Patch tests: membrane AND bending constant-stress/curvature patches (Tier-2 gate).
- Rigid-body modes: 6 zero-energy modes exactly — no hourglass extras, no spurious
  drilling zeros (the classic MITC4 pathology checklist, brief §4.3).
- Thin-limit: SS square plate uniform load vs Kirchhoff 0.00406·qa⁴/D within band as
  t/a → thin (locking would blow this up at fine slenderness); thick 6 m / t=0.15 m
  case vs the **SAP2000-validated Mindlin number already in hand** (7.336 mm,
  `plate_ss_check.sdb` — ratio band ~[0.97, 1.03]).
- Twisted-ribbon / out-of-plane MacNeal-Harder shell cases where applicable.
- Shear-locking regression: aspect-ratio-distorted mesh still converges (MITC4's
  raison d'être).

### Step 6 — Solid elements: Hex8 + Tet4

**Files:** `elements/solid.py` (new — Hex8 full 2×2×2 integration; Tet4 constant
strain), 3D D-matrix already in `materials/nd.py`.

**Verify (tests/fea/test_solids.py):**
- 3D patch tests (Hex8 distorted patch, Tet4 assembly), rigid-body modes = 6,
  FD tangent, K symmetric.
- Hex8 cantilever beam vs beam theory (+ convergence study); uniaxial bar of hexes ==
  PL/AE exactly; Tet4 documented-stiff behavior vs Hex8 on the same problem.
- Mesher box-grid integration (Step 3 hex generator).

### Step 7 — Solver upgrade: sparse assembly + factorize-once + optional CHOLMOD

**Files:** `system/soe.py` (extend).

Shell/solid models make the current dense-default assembly the bottleneck long before
solve time does.

- **Sparse assembly path:** COO triplet accumulation → CSR (scipy), behind the existing
  `SystemOfEqn` interface; dense stays the no-scipy fallback. Auto-select by num_eq
  threshold, overridable.
- **Factorize-once / multi-RHS API** (`factorize()` + `solve_rhs(b)`) via
  `scipy.sparse.linalg.splu` / `cholesky` — needed *now* for `analyze_cases` with many
  load cases, and is the machinery Phase 3's influence lines are built on (brief §10.3).
- **Optional CHOLMOD** (scikit-sparse) behind the same interface as a lazy import,
  skipped cleanly when absent — same pattern as the existing optional scipy.
- Determinism note: RCM ordering already in place; verify sparse path uses it.

**Verify (tests/fea/test_solver_upgrade.py):** dense vs sparse vs (if present) CHOLMOD
identical results on a shell model; multi-RHS == repeated solves; a ~10–20k-DOF
structured slab solves in seconds (smoke perf bound, generous).

### Step 8 — Section cuts + generalized displacements

**Files:** `postprocess.py` (new: section cuts, generalized displacements), recorders
extended.

- **Section cut:** given a node group + a side-selection of elements, sum element
  resisting-force contributions on the cut nodes → resultant F/M about a user point
  (brief §10.5). Works for frame, shell, and solid meshes.
- **Generalized displacement:** named linear combination of nodal DOFs (e.g. drift,
  relative deflection) as a recorder quantity — also the future control DOF for
  Phase 5 pushover.
- Exposed in `json_solve.py` / `solve_model` output (opt-in request keys).

**Verify (tests/fea/test_postprocess.py):**
- Beam midspan cut == M(x)=wx(L−x)/2 closed form; portal base cut == story shear.
- Shell deck midspan cut: slab+beams moment total == static wL²/8 tributary total —
  the same balance the SAP2000 AreaForceShell check wants (nice cross-project echo).
- Generalized displacement == hand combination of solved DOFs.

---

## Phase gate — "real building model" integration test

`tests/fea/test_phase2_gate.py`, mirroring the SAP2000 thick-shell validation already
performed live (session 3, 2026-07-04):

One model, imperial: rectangular concrete deck (MITC4 shells) on steel W-girders and
beams (3D frames, offsets ignored v1) on columns (3D frames, fixed bases), rigid
diaphragm option exercised, dead + live cases + LRFD combos via `analyze_cases`.

Pass criteria:
1. ΣF/ΣM statics gate exact per case (the §15 rule, already built into `json_solve`).
2. Column reactions match double-symmetry groups exactly.
3. Deck center deflection within a stated band of the SAP2000 thick-shell result for
   the equivalent SI model (the 18/18-validated `deck_bgc_check` numbers).
4. Section cut across midspan balances the static total moment.
5. Whole suite (target ≈ 340–380 tests) green, plus the 11 standalone Ch 15 scripts.

---

## Explicitly OUT of Phase 2 (do not scope-creep)

- Mass/eigen/dynamics (Phase 3 — NAFEMS path A lands there).
- Any nonlinearity: P-Delta, corotational, Newton (Phase 4); materials/fiber (Phase 5).
- Unstructured meshing (Gmsh wrapping — Phase 7); axisymmetric elements; Quad8/Hex20
  higher-order family (add later if convergence studies demand).
- Lagrange-multiplier and penalty constraint handlers (transformation only).
- Shell composite/layered behavior; frame shear deformation (Timoshenko) — noted for
  Phase 5/3 respectively.

## Dependencies

None new required. `scipy` already optional-but-present; `scikit-sparse` (CHOLMOD) is
optional dev-extra, tests skip when absent (same convention as `openseespy`).

## Verification discipline (unchanged)

FD consistent-tangent check (`src/fea/testing.py`) on every new element; patch tests
gate Steps 2/5/6; statics gate on every integration model; no phase-gate claim until
the full suite has actually been run. Commit + push per step (stage by name,
descriptive message) — one step, one commit minimum.
