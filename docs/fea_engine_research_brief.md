# Building a Headless Structural Finite Element Analysis Engine
### A research brief and engineering plan for an AI-assisted implementation of a "SAP2000 without the GUI"

> **Purpose of this document.** This is a domain briefing *and* an architecture/implementation plan. It is written so that a capable coding AI (and its human director) can understand what a structural finite-element engine actually is, how the proven ones are organized internally, and how to build one incrementally without getting lost. Read Sections 1–3 for fundamentals and architecture, Sections 4–11 for the subsystems, and Sections 12–16 for the technology stack, verification, and the phased build plan.

---

## 0. Executive summary and honest scope

You want the *analysis core* of a program like SAP2000: the part that takes a structural model (nodes, members, materials, loads, supports) and computes its response — displacements, member forces, stresses, mode shapes, time-history response — including **nonlinear** behavior. You explicitly do **not** want the CAD/GUI layer.

Two things must be said up front so the plan is realistic:

1. **This core is buildable and well-understood.** The mathematics and software architecture are mature and thoroughly documented in the open literature. A single focused developer working with a strong coding AI can produce a genuinely useful nonlinear engine covering a large, valuable subset of SAP2000's analysis capabilities.

2. **"All of SAP2000" is a decades-long, team-scale effort.** SAP2000's analysis engine (branded *SAPFire*) represents ~40 years of accumulated element formulations, solver tuning, and validation. The winning strategy is **not** to clone everything at once. It is to build a clean, extensible *framework* and grow the element/material/analysis library outward from a rock-solid linear core. This document is organized to make exactly that possible.

The single most important strategic decision is this: **do not invent a new architecture.** The open-source engine **OpenSees** (Open System for Earthquake Engineering Simulation, UC Berkeley) already solved the hard design problem — how to structure a nonlinear FE engine so it stays extensible. We will use its object model as the reference blueprint throughout. Reimplementing a clean, modern version of that design is a realistic and excellent goal.

---

## 1. What the engine must do — decomposing "what makes SAP2000 great"

Before architecture, enumerate capabilities as concrete, buildable units. SAP2000's analysis engine offers, roughly in order of increasing difficulty:

**Linear statics (the foundation)**
- Direct stiffness method: assemble and solve `K u = F`.
- 1D frame/truss elements, 2D shell/plate elements, 3D solid elements, spring/link elements.
- Boundary conditions (restraints), prescribed displacements, multi-point constraints (rigid diaphragms, equal-DOF, rigid links).
- Load types: nodal loads, member distributed loads, thermal loads, prestrain, self-weight, support settlement.
- Load cases and load combinations.

**Dynamics — modal and response**
- Eigenvalue analysis: natural frequencies and mode shapes (`K φ = λ M φ`). SAP2000 offers both classic eigen-analysis (with automatic shifting for ill-conditioned systems) and **Ritz-vector** analysis (load-dependent vectors that converge faster for a given loading).
- Modal superposition time-history and response-spectrum analysis (SRSS / CQC modal combination).
- Modal participation factors and effective modal mass.

**Geometric nonlinearity**
- P-Delta (second-order effects from axial force acting through lateral displacement) via a geometric stiffness matrix.
- Large-displacement / large-rotation analysis (corotational or updated-Lagrangian formulations).
- Catenary cable analysis, snap-through, and buckling (linear/eigenvalue buckling and nonlinear buckling).

**Material nonlinearity**
- Plasticity in frames via **concentrated plastic hinges** (moment-rotation or interaction P-M2-M3 hinges) and via **distributed fiber hinges / fiber sections** (cross-section discretized into fibers, each with a uniaxial stress-strain law).
- Layered nonlinear shells (e.g., von Mises / J2 plasticity through the thickness).
- Nonlinear links: gap/hook, multilinear plastic, friction-pendulum isolators, viscous/viscoelastic dampers.

**Nonlinear solution procedures**
- Nonlinear static analysis (incremental-iterative Newton-type solution).
- **Pushover analysis** (displacement- or load-controlled nonlinear static, the backbone of performance-based seismic design).
- Nonlinear direct-integration time-history (Newmark / HHT integration with nonlinear state at each step).
- **Staged (sequential) construction**: activate/deactivate groups of elements while carrying state forward — critical for bridges, tall buildings, tensioned structures.

**Prestress, cables, and time-dependent behavior**
- Post-tensioned/pretensioned **tendons** with the full loss chain (friction along the profile, anchorage set, elastic shortening, and long-term creep/shrinkage/relaxation).
- **Catenary cables** with large-displacement tension stiffening.
- **Target-force / target-tension** iteration: solve for the prestrain or jacking force that yields a specified *final* member force in the redistributed structure.
- **Time-dependent concrete**: creep, shrinkage, and age-dependent stiffness — essential when staged construction spans time.

**Moving loads and influence-based analysis**
- **Influence lines** (spine/frame models) and **influence surfaces** (refined shell models).
- **Moving-load / vehicle-live-load** analysis that envelopes extreme response as loads traverse the structure, with standard design-vehicle libraries (e.g., AASHTO).

**Specialized analysis modes and derived outputs**
- **Soil-structure interaction**: foundation springs, Winkler supports, compression-only/uplift supports, support plasticity.
- **Section cuts**: integrate stresses/forces across an arbitrary plane to recover resultant forces.
- **Generalized displacements**: user-defined DOF combinations (e.g., inter-story drift), usable as pushover control DOFs.
- **Floor-vibration and frequency-domain** response (steady-state, power-spectral-density).
- **Energy accounting** (strain, kinetic, damping, hysteretic) over nonlinear histories.
- **Cross-section property and moment-curvature / P-M interaction** computation for arbitrary built-up sections.

**Condition assessment and deterioration modeling**
- Represent *degraded/aging* structures: reinforcement **section loss** (uniform and pitting corrosion), reduced steel strength and **ductility**, **bond deterioration** (steel–concrete bond-slip), cover **cracking and spalling**, and **loss of confinement** as stirrups corrode.
- Evolve these over **time** (corrosion propagation) with the staged/time machinery, or evaluate discrete damage scenarios (e.g., condition at 25/50/75 years).
- Expressed as *parameterizations* of fibers, materials, sections, and bond interfaces — not a separate physics engine (see Section 5.6).

**Loading and meshing (analysis-adjacent, but in scope for a headless engine)**
- Automatic code-based generation of **seismic, wind, wave, and vehicle** loads.
- Automatic **meshing** of member/area/solid objects into a *conforming* (node-matched) finite-element mesh, with quality control and edge-constraint stitching only where meshes must mismatch — see Section 11. This matters most for nonlinear models, where mesh quality and node-to-node compatibility directly govern result consistency.

**Out of the analysis core (separate modules — deferred or excluded)**
- **Design checks** (steel/concrete/aluminum/cold-formed per AISC/ACI/Eurocode) — a large post-processing domain that consumes analysis results.
- **Parametric bridge modeling** (layout lines, spans, bearings, tendon templates) — a modeling front-end built on top of the engine.
- **Public API / interoperability** (COM/.NET/Python bindings, IFC/DXF/Excel exchange) — for a headless engine your own scripting interface already fills this role (see Section 12).

**Design implication:** almost every item above is a *plug-in* to a small, stable framework. If the framework is right, adding "layered nonlinear shell" or "friction-pendulum link" later is a self-contained task, not a rewrite. That is the entire reason for the architecture in Section 2.

---

## 2. Reference architecture — the object model to copy

The proven design (from Frank McKenna's 1997 Berkeley PhD thesis and realized in OpenSees) cleanly separates **the model** (what the structure *is*) from **the analysis** (how you *solve* it). This separation is the crown jewel; preserve it.

```
                        ┌──────────────────────────────────────────┐
                        │              ModelBuilder                 │
                        │  (reads input / API calls; constructs and │
                        │   registers all model objects)            │
                        └───────────────────┬──────────────────────┘
                                            │ populates
                                            ▼
   ┌───────────────────────────────  DOMAIN  ────────────────────────────────┐
   │  The container for the finite-element model. Holds, by unique tag:       │
   │    • Nodes            (geometry + DOFs + mass + current response)         │
   │    • Elements         (know their nodes; compute K, resisting force, M)  │
   │    • Materials        (uniaxial / nD constitutive laws; own their state) │
   │    • Sections         (aggregate materials over a cross-section)         │
   │    • SP_Constraints   (single-point: restraints, prescribed disp.)       │
   │    • MP_Constraints   (multi-point: rigid links, diaphragms, equalDOF)   │
   │    • LoadPatterns     (time-varying containers of nodal/element loads)   │
   │    • TimeSeries       (scale factor vs. time/pseudo-time for a pattern)  │
   └──────────────────────────────────┬──────────────────────────────────────┘
                                       │ analyzed by
                                       ▼
   ┌────────────────────────────────  ANALYSIS  ─────────────────────────────┐
   │  An Analysis is ASSEMBLED from interchangeable component objects:        │
   │    • ConstraintHandler  – enforces SP/MP constraints on the equations    │
   │    • DOF_Numberer       – maps free DOFs → equation numbers (ordering)   │
   │    • AnalysisModel      – the bookkeeping bridge Domain ↔ equations       │
   │    • SystemOfEqn+Solver – how Ax=b is stored and factorized/solved       │
   │    • Integrator         – defines what equations to form + how to update │
   │                            (LoadControl, DisplacementControl, ArcLength, │
   │                             Newmark, HHT, …)                             │
   │    • SolutionAlgorithm  – the iteration scheme (Linear, Newton, Modified │
   │                            Newton, BFGS, Broyden, NewtonLineSearch, …)   │
   │    • ConvergenceTest     – when to declare an iteration converged        │
   └──────────────────────────────────────────────────────────────────────────┘
                                       │ writes to
                                       ▼
                        ┌──────────────────────────────────────────┐
                        │               Recorders                   │
                        │  (observe Domain each step; stream node/  │
                        │   element response to memory/disk)        │
                        └──────────────────────────────────────────┘
```

**Why this works.** A "linear static" analysis and a "nonlinear seismic time-history" analysis differ only in *which component objects are plugged into the Analysis*. The same elements, materials, and Domain serve both. To add a new solution capability you write one new Integrator or Algorithm; to add a new physical behavior you write one new Element or Material. Nothing else changes. This is the extensibility you are paying for.

**Key polymorphic interfaces to nail down early (these are the contracts the whole system rests on):**

- `Element`: given the current nodal displacements (held on its Nodes), return its **resisting force vector** and its **tangent stiffness matrix** (and mass/damping matrices). It also owns `commitState()` / `revertToLastCommit()` so nonlinear/path-dependent state is advanced only when a step converges.
- `Material` (uniaxial): given a trial strain, return trial **stress** and **tangent modulus**; support `commitState()`. `NDMaterial` is the multiaxial analogue returning a stress vector and a tangent matrix.
- `Integrator`: `formTangent()` and `formResidual()` — it decides the left- and right-hand sides. Static integrators (LoadControl, DisplacementControl, ArcLength) and transient integrators (Newmark, HHT) are just different implementations.
- `SolutionAlgorithm`: drives the iteration loop, repeatedly asking the Integrator to form the system, the Solver to solve it, and updating the Domain until the ConvergenceTest passes.

Get these four interfaces right and the rest is filling in the library.

**The input layer — description-driven, not CAD.** You will *describe* models, not draw them, so plan two tiers of model definition, both of which populate the same `Domain` through the `ModelBuilder`:
- **Tier 1 — a programmatic/scripting API** (OpenSeesPy-style imperative commands): explicit `node`, `element`, `section`, `material`, `constraint`, `load`, and analysis calls. This is the ground truth every model ultimately compiles to, and it doubles as your test-authoring language.
- **Tier 2 — a parametric library of structural archetypes**: high-level generators such as `SimplySupportedRCBeam(span, section, rebar_layout, cover, supports, load)`, `Cantilever(...)`, `PortalFrame(...)`, `ContinuousBeam(...)`, `FlatSlab(...)`, `ShearWall(...)`, `Column(...)`. Each expands a terse description into the Tier-1 model *plus* a clean conforming mesh. A modest library of archetypes covers a large share of real requests, and it grows one generator at a time.
- **The coding AI is the natural-language front-end.** Prose like "a simply supported RC beam, 6 m span, 300×600, 4×⌀25 bottom bars, 25% corrosion loss, cover spalled" is translated by the AI into Tier-2 (or, for anything non-standard, Tier-1) calls. Design the Tier-2 API to be clean, explicit, and well-documented — that is precisely what makes the AI a reliable translator and keeps the generated models verifiable.

Because models come from parametric archetypes rather than arbitrary imported geometry, meshing is largely *structured/mapped per archetype* (Section 11.5) — which sidesteps most of the hard unstructured-CAD meshing problem entirely.

---

## 3. Degrees of freedom, assembly, and the direct stiffness method (the non-negotiable core)

Everything reduces to forming and solving a system of equations. For statics:

```
K u = F          (linear)
r(u) = F_ext − F_int(u) = 0    (nonlinear residual form; solved iteratively)
```

**The direct stiffness method, mechanically:**
1. Each node carries a fixed set of DOFs (a 3D frame node has 6: three translations, three rotations; a solid node has 3; a plate node has ~5–6).
2. Each element computes a local stiffness `k_e` in its own coordinate system, then transforms it to global coordinates via a rotation/transformation matrix `T`: `K_e = Tᵀ k_e T`.
3. Element contributions are **scattered/assembled** into the global system according to each element's DOF connectivity map (the "location array" / ID vector that maps element-local DOFs to global equation numbers).
4. Constraints are applied (Section 6), the system is solved, and results are **gathered** back to elements to recover member forces and stresses.

**This assembly/DOF-mapping machinery is the first thing to build and the thing to get bulletproof.** Every element formulation and every analysis type depends on it. Represent it explicitly (a per-element `ID` array of global equation numbers; a global `AnalysisModel` that owns the numbering). Free DOFs get equation numbers ≥ 0; constrained DOFs get negative flags and are excluded from the solve.

---

## 4. The element library — build order and formulations

Add elements roughly in this order. Each is a self-contained implementation of the `Element` interface.

**4.1 Truss (2-node, axial only).** The "hello world" of FEA. Linear version first; then a corotational truss for large displacement; then a nonlinear-material truss (uniaxial `Material`). Use it to validate assembly, transformation, and the nonlinear loop before anything complex.

**4.2 Frame / beam-column (2-node, 6 DOF/node in 3D).** The workhorse of structural engineering. There are two fundamentally different formulations — implement both, understanding the trade-off:
- **Displacement-based (stiffness) element.** Interpolates displacements with shape functions (Hermitian cubics for bending), integrates section stiffness along the length by Gauss quadrature. Simple, standard. For material nonlinearity it needs *many* elements per member to capture the spread of plasticity accurately.
- **Force-based (flexibility) element.** Interpolates the *internal forces* exactly (they satisfy equilibrium along the member for a prismatic element), then integrates section flexibility. This is a hallmark of OpenSees and a major reason it is accurate for nonlinear frames: **one force-based element per physical member** captures distributed plasticity that would need ~4–8 displacement-based elements. The trade-off is an inner element-level iteration (state determination) and more care in implementation. Strongly recommended for serious nonlinear frame work.
- Include **Timoshenko (shear-deformable)** as well as Euler-Bernoulli behavior; add member end releases (moment hinges) and rigid end offsets.

**4.3 Shell / plate elements (3–4 node).** Needed for walls, slabs, and plated structures. Combine **membrane** (in-plane) and **plate-bending** behavior. Robust choices: MITC4 (mixed interpolation, avoids shear locking) for the quad; DKT/DKQ (discrete Kirchhoff) as an alternative. For nonlinear material, use a **layered/multi-layer shell**: integrate an `NDMaterial` (e.g., J2 plasticity) through the thickness at each integration point. Watch for the classic pathologies: **shear locking**, **membrane locking**, and **hourglassing** (zero-energy modes) with reduced integration.

**4.4 Solid (continuum) elements.** 8-node hex (brick) and 4-node tet, isoparametric. Needed for detailed 3D stress analysis, thick components, soil. Same integration/quadrature machinery as shells but simpler kinematics (translations only).

**4.5 Link / spring / support elements (2-node or grounded).** The vehicle for a huge fraction of "advanced" nonlinear behavior with minimal new machinery: linear springs, gap (compression-only), hook (tension-only), multilinear plastic, viscous dampers (force ∝ velocityᵃ), friction-pendulum isolators, hysteretic devices, and **steel–concrete bond-slip interfaces** (a bond-stress–slip law used for deterioration modeling, Section 5.6). Because they are low-DOF, they are the cheapest way to add impressive capability.

**4.6 Cables, tendons, and prestress.** Three related capabilities, all tension-oriented:
- **Catenary cable element** — a slender member carrying only axial tension with a draped, load-adapting shape; large-displacement tension stiffening is inherent once geometric nonlinearity is on (nonlinear-static, staged, and direct-integration cases). A simpler "straight cable of known shape" (a tension-only, tension-stiffened truss) is enough when only tension stiffening matters and the geometry is fixed — build that first, the true catenary later.
- **Tendons (prestress)** — post-tensioned/pretensioned members applied either as load-carrying elements or as an equivalent set of loads, with a prescribed jacking force and the full loss chain: friction along the profile, anchorage set (seating), elastic shortening, and long-term creep/shrinkage/relaxation (couple with time-dependent materials, Section 5.4).
- **Target-force / target-tension solution** — an *outer* iteration that adjusts a member's prestrain or jacking force until a specified force is achieved in the deformed, redistributed structure. Architecturally this is a specialized integrator/solver wrapper around the static solve, not a new element; schedule it alongside staged construction (Section 9.6).

**Shared element machinery (build once, reuse everywhere):**
- **Isoparametric mapping**: shape functions `N(ξ,η,ζ)`, the Jacobian `J`, and the strain-displacement matrix `B`.
- **Numerical (Gauss) quadrature**: integrate `∫ Bᵀ D B |J| dΩ` over the natural domain.
- **Coordinate transformations**: local↔global, including the geometric (corotational) transformation for large displacement.

---

## 5. Materials and sections — where nonlinearity lives

Elements should be *dumb about physics*: they ask a `Section` for a force-deformation response, and the Section asks its `Material`s for stress-strain response. This layering (Element → Section → Material) is what lets one frame element model an elastic steel beam, a yielding reinforced-concrete column, or a composite section without changing the element code.

**5.1 Uniaxial material models** (the `Material` interface: strain → stress + tangent, with committable state):
- Linear elastic.
- Elastic-perfectly-plastic and bilinear (kinematic/isotropic hardening).
- Steel models with hardening and Bauschinger effect (e.g., Menegotto-Pinto style).
- Concrete models with tension softening, compression crushing, and hysteretic unloading/reloading.
- **Confined vs. unconfined concrete** (e.g., the Mander model): stirrup confinement raises core strength and ductility — needed for columns and, crucially, for modeling *confinement loss* as ties corrode (Section 5.6).
- Rate/temperature-dependent variants later.

**5.2 Multiaxial (nD) material models** (stress vector + consistent tangent matrix):
- **J2 (von Mises) plasticity** with a **radial-return** integration and a **consistent (algorithmic) tangent** — this is the canonical elastoplastic implementation and the pattern most others follow. Getting the consistent tangent right (not the continuum tangent) is what preserves the quadratic convergence of Newton's method.
- Drucker-Prager / Mohr-Coulomb for soils and concrete confinement later.

**5.3 Sections** (aggregate material response over a cross-section):
- **Elastic section**: just EA, EI, GJ, GA — for linear/elastic frames.
- **Fiber section**: discretize the cross-section into many small fibers, each assigned a uniaxial material. Under the plane-sections-remain-plane assumption, a fiber's axial strain is a linear function of the section's axial strain and curvatures. The section's force resultants (axial force, bending moments) and tangent stiffness come from summing the fiber contributions. This one construct gives you: axial-moment (P-M) interaction, spread of plasticity, and realistic hysteresis — essentially for free, for any cross-section shape. It is the backbone of modern nonlinear frame analysis.

**5.4 Time-dependent material behavior (creep, shrinkage, aging).** Concrete gains stiffness as it cures while creep and shrinkage strains accumulate under sustained load. Implement this as a material *wrapper* that evolves the elastic modulus and superimposes imposed (creep/shrinkage) strains as a function of elapsed time, following a model code (CEB-FIP / *fib*, ACI 209, or Eurocode 2). The `Material` interface therefore needs a notion of a **time/age step** alongside the usual strain step. These effects only matter under sustained and sequential loading, so they pair tightly with staged construction (Section 9.6): each construction stage advances the clock and accrues creep/shrinkage. Skip entirely for short-duration or purely elastic analyses.

**5.5 Cross-section property and moment-curvature tool.** Independent of the global solver, provide a utility that takes an arbitrary built-up cross-section (multiple materials, reinforcement, holes) and computes: (a) **elastic/geometric properties** — area, moments of inertia I₂/I₃, torsion constant J, shear areas, centroid, and principal axes — for elastic-section definitions; and (b) nonlinear **moment-curvature** curves and **P-M (axial-force/moment) interaction** surfaces, by incrementally imposing curvature and axial strain on the fiber discretization and integrating the fiber response. This is the engine behind SAP2000's "Section Designer." It reuses the fiber-section and uniaxial-material machinery from Section 5.3, so build it immediately after fiber sections; its outputs feed both elastic-section properties (Section 5.3) and concentrated plastic-hinge definitions (Section 1, material nonlinearity).

**Critical correctness note:** the whole nonlinear method depends on the **tangent** returned at each level being the *true derivative* of the returned force/stress with respect to the deformation/strain. Inconsistent tangents silently destroy Newton convergence and are the #1 source of "why won't my model converge" bugs. Verify every constitutive tangent numerically (finite-difference the stress w.r.t. strain and compare) as an automated test.

**5.6 Modeling deterioration and damage (corrosion, section loss, bond loss, spalling).** A central use case is assessing *degraded* structures, and the payoff of the fiber/material/section/interface architecture is that deterioration is expressed as **parameterization of existing constructs, not new solver code.** Map each physical effect onto the model:
- **Rebar section loss (uniform corrosion)** → reduce the *area* of the affected steel fibers. Drive the amount from a corrosion model (Faraday's law: corrosion current density × time → mass/area loss) or specify a percentage directly.
- **Pitting corrosion** → a localized, deeper area loss (apply a pitting factor to the mean penetration) *and* — importantly — a reduced rupture strain; pitting embrittles the bar and can cause premature fracture.
- **Reduced steel strength/ductility** → modify the steel uniaxial material: lower `f_y`/`f_u` on the nominal area and, critically, cut the ultimate strain `ε_u`. This can shift a section from ductile-flexural to brittle bar-fracture failure.
- **Bond deterioration** → a **bond-slip interface** between steel and concrete (Section 4.5): a zero-length or distributed interface material with a bond-stress–slip law (e.g., CEB-FIP / Eligehausen) whose peak strength degrades with corrosion — or, more simply, a bond-slip material in series with the steel fiber (as in OpenSees `Bond_SP01`) and/or a degraded tension-stiffening branch in the concrete. Lumping bond-slip at member ends as a fixed-end-rotation spring is a pragmatic first version.
- **Cover cracking and spalling** → **deactivate the cover-concrete fibers** once a spalling criterion is met, reusing the staged/time deactivation machinery (Section 9.6) at the fiber level; the section area and geometry update accordingly.
- **Loss of confinement** → migrate the core-concrete fibers from a *confined* toward an *unconfined* concrete model (Section 5.1) as ties corrode and cover spalls, lowering core strength and ductility; reduce stirrup/tie area for shear checks.
- **Time evolution** → wrap the above in a **deterioration driver** that updates fiber areas, material parameters, bond strength, and spalling flags as a function of exposure time (or evaluates discrete scenarios), then continues/re-runs the analysis via the staged/time machinery.

Two fidelity levels answer different questions:
- **Fiber-frame model** (1D elements, fiber sections): efficient; captures section loss, ductility loss, confinement change, and lumped bond-slip — the right default for global capacity and pushover of a corroded member.
- **Continuum RC model** (solid concrete + truss/embedded rebar + explicit bond interface): captures cover-cracking/spalling geometry, local bond, and shear-dominated failures — high fidelity but expensive, and it *requires* the softening regularization of Section 11.4 to stay mesh-objective.

*Worked example — simply supported RC beam with corrosion.* The description "6 m simply supported beam, 300×600 section, 4×⌀25 bottom bars, 40 mm cover, 25% corrosion section loss with cover spalled, midspan point load" expands (via the Tier-2 archetype, Section 2) to: frame elements along the span; pin + roller supports (SP constraints); a fiber section with unconfined-cover, confined-core, and steel fibers. The deterioration overlay then: reduces the bottom-bar steel fibers to 75% area with a lowered `ε_u`; deactivates the bottom/tension cover fibers (spalled); attaches a degraded bond-slip material to the bottom bars; and, if stirrups are affected, applies a confinement/shear reduction. Run displacement-controlled to obtain the corroded moment-curvature and load-deflection, and compare against the pristine beam. Every step reuses machinery already in the plan — which is the whole point of the architecture.

---

## 6. Constraints — restraints and multi-point constraints

- **Single-point constraints (SP):** a DOF is fixed or prescribed to a value (supports, settlements). Simplest to handle: remove the DOF from the free set (or prescribe it).
- **Multi-point constraints (MP):** a linear relationship among several DOFs — rigid diaphragms (all nodes on a floor share in-plane rigid-body motion), rigid links, equal-DOF, master-slave. Essential for real buildings.

Three standard enforcement strategies (implement as swappable `ConstraintHandler`s):
1. **Transformation method** — eliminate constrained DOFs by substituting the constraint relations into the system, reducing its size. Exact, no conditioning penalty, but more bookkeeping. Best default for rigid diaphragms.
2. **Penalty method** — add very large stiffness terms enforcing the constraint approximately. Trivial to implement, but pollutes conditioning and interacts badly with some convergence tests. Good for a first pass.
3. **Lagrange multipliers** — augment the system with extra unknowns enforcing the constraint exactly. Exact, but enlarges the system and makes it indefinite (needs a solver that tolerates that).

Ship the penalty handler first for speed of development, then the transformation handler for production quality.

---

## 7. Linear algebra and data structures — the performance floor

The solver decides how large a model you can run. Structural stiffness matrices are **large, sparse, symmetric**, and (for stable linear problems) **positive definite**.

**Storage schemes** (support at least one sparse scheme; the classic FE choice is skyline/profile, the modern choice is compressed sparse column/row):
- **Skyline/profile** — the traditional FE storage; pairs naturally with a profile (LDLᵀ) solver and with bandwidth-minimizing orderings.
- **Compressed Sparse Row/Column (CSR/CSC)** — the modern general format; what most external sparse solvers expect.

**DOF ordering matters enormously** (it determines fill-in and therefore speed/memory). Implement a `DOF_Numberer` with:
- **Reverse Cuthill-McKee (RCM)** — cheap bandwidth reduction, great for profile solvers.
- **Approximate Minimum Degree (AMD)** or **METIS nested dissection** — fill-reducing orderings for sparse direct factorization of large 3D models.

**Solvers — do NOT write your own production sparse factorization; wrap a proven library.** Recommended, in rough order of when you'll need them:
- **Eigen** (C++ header-only): dense + built-in sparse `SimplicialLDLT`/`LLT` and iterative `ConjugateGradient`. Perfect for early development and small/medium models. In Python, the analogue is **SciPy sparse** (`scipy.sparse.linalg`, `spsolve`).
- **SuiteSparse** (Tim Davis): **CHOLMOD** (supernodal Cholesky for SPD systems — the sweet spot for linear structural statics), **UMFPACK** (LU for general/indefinite systems, e.g., Lagrange-multiplier-augmented ones), **KLU**. Reliable and fast.
- **MUMPS** or **Intel MKL PARDISO**: parallel multifrontal direct solvers for large models. Independent benchmarks repeatedly rank MKL PARDISO, UMFPACK, and MUMPS among the most reliable and fastest general sparse direct solvers, so any of these is a safe production target.
- **Iterative solvers (PETSc; preconditioned CG/GMRES with AMG)** only when models get very large (hundreds of thousands to millions of DOFs) and factorization memory becomes the bottleneck. Structural systems with poor conditioning (thin shells, mixed stiffness) make iterative solvers tricky — treat this as an advanced, optional path.

**Eigenvalue solvers** (for modal analysis, `K φ = λ M φ`):
- **ARPACK** (or a modern equivalent such as Spectra for C++, `scipy.sparse.linalg.eigsh` in Python) using **shift-invert** to get the lowest modes efficiently. Provide automatic shifting to handle rigid-body modes / ill-conditioning (SAP2000 advertises exactly this "auto-shift" behavior).
- Also implement **subspace iteration** and **Ritz vectors** (load-dependent Ritz vectors converge faster than eigenvectors for a specific excitation and are what SAP2000 recommends for many dynamic problems).

**Design rule:** put the linear solver behind a thin `Solver` interface so you can start with Eigen/SciPy and swap in CHOLMOD/MUMPS/PARDISO later without touching the rest of the code.

---

## 8. Nonlinear solution strategy — the heart of the engine

This is where a structural engine earns its keep, and where most bugs live. The goal is to solve `r(u) = F_ext − F_int(u) = 0` incrementally.

**8.1 The incremental-iterative skeleton.** Apply load (or displacement, or arc-length parameter) in **steps**; within each step, **iterate** to drive the residual to zero:

```
for each load step:
    predict a trial state (integrator's predictor)
    repeat (equilibrium iterations):
        form tangent K_T and residual r at current trial state   # Integrator
        solve  K_T Δu = r                                        # Solver
        update trial displacements u ← u + Δu                    # push to Domain/Elements
        recompute F_int(u) by asking every element               # state determination
        test convergence on ‖r‖, ‖Δu‖, or energy                 # ConvergenceTest
    until converged or max-iters
    commitState()   # advance path-dependent material/element state
```

**8.2 Solution algorithms** (the iteration scheme — swappable `SolutionAlgorithm`s):
- **Full Newton-Raphson**: re-form the tangent every iteration. Quadratic convergence near the solution; most robust; most expensive per iteration.
- **Modified Newton**: form the tangent once per step and reuse it. Cheaper iterations, more of them.
- **Initial-stiffness**: reuse the elastic tangent throughout — very robust past limit points, very slow.
- **Quasi-Newton (BFGS, Broyden)**: update an approximate tangent from residual/displacement history. A good speed/robustness compromise.
- **Line search**: scale the Newton step by a factor that reduces the energy — cheap insurance that dramatically improves robustness on hard problems. Add it early.

**8.3 Integrators / continuation methods** (what "advancing" means — this is the subtle part):
- **Load control**: increase the applied load by a fixed increment each step. Fails at **limit points** (the tangent goes singular at peak load — e.g., buckling, ultimate capacity).
- **Displacement control**: drive a chosen DOF by a fixed increment and solve for the load factor. Passes limit points; the standard method for **pushover** analysis.
- **Arc-length (Riks / Crisfield)**: constrain a combined norm of load *and* displacement increments. The most powerful continuation method — traces **snap-through** and **snap-back** responses that both load and displacement control fail on. Essential for buckling and post-peak behavior. Implement after load and displacement control work.

**8.4 Convergence tests** (swappable):
- **Unbalanced-force norm** (residual `‖r‖`), **displacement-increment norm** (`‖Δu‖`), **energy-increment norm** (`‖Δuᵀ r‖`), and combinations. Different tests suit different constraint handlers (e.g., the residual-norm test interacts badly with penalty constraints — document these gotchas). Provide a relative tolerance scaled by the applied load, as SAP2000 does.

**8.5 Geometric nonlinearity** (how the tangent captures second-order effects):
- **P-Delta**: add a **geometric stiffness matrix** `K_g` (a function of current axial force) to the material stiffness. Captures the destabilizing effect of gravity acting through lateral drift. Cheap and hugely important for tall/slender structures and stability.
- **Large displacement / large rotation**: use a **corotational formulation** — attach a frame that follows the element's rigid-body motion, compute small-strain deformation in that local frame, then transform forces/stiffness back to global. This cleanly separates large rigid-body motion (rotations especially) from local deformation and reuses your existing small-strain element internals. The alternative is a total/updated-Lagrangian continuum formulation (more general, more work). Corotational is the pragmatic structural choice and covers cables, buckling shapes, and mechanisms.

**8.6 Robustness features that separate a toy from a tool** (add these deliberately):
- **Automatic step-size cutting**: if a step fails to converge, halve the increment and retry (SAP2000's "null step" logic is a relative of this).
- **Event-to-event solution** for hinge/link nonlinearity: advance exactly to the next stiffness-change event (a hinge yielding) rather than iterating through it — improves reliability for concentrated-plasticity models.
- **Adaptive switching of algorithm** (e.g., fall back from Newton to modified-Newton-with-line-search on failure).

---

## 9. Dynamics

**9.1 Mass and damping.**
- Lumped (diagonal) and consistent mass matrices.
- **Rayleigh damping** `C = αM + βK` (mass- and stiffness-proportional) is the standard structural choice; support specifying it by target damping ratios at two frequencies. Modal damping for modal methods.

**9.2 Modal analysis.** Solve the generalized eigenproblem for frequencies and mode shapes; compute participation factors and effective modal masses (used to decide how many modes to keep). Offer Ritz vectors as an alternative basis (Section 7).

**9.3 Time-history analysis.**
- **Linear modal superposition**: project onto mode shapes, integrate each decoupled modal equation, recombine. Fast and exact for linear systems.
- **Direct integration** (required for nonlinear): step the coupled equations of motion in time.
  - **Newmark-β** (implicit): the standard; unconditionally stable in its average-acceleration form.
  - **HHT-α** (Hilber-Hughes-Taylor): adds controllable numerical damping of spurious high-frequency modes — often preferred for nonlinear problems.
  - **Central difference** (explicit): conditionally stable, no factorization per step; useful for impact/blast and highly nonlinear contact, less so for typical structural dynamics.
- Each nonlinear time step wraps the Section-8 equilibrium iteration *inside* the time integrator (the integrator forms the effective dynamic tangent and residual; the algorithm iterates).

**9.4 Response-spectrum analysis.** Combine modal maxima with **SRSS** or, for closely-spaced modes, **CQC**. This is a linear method but ubiquitous in seismic design.

**9.5 Buckling (eigenvalue) analysis.** Solve `(K + λ K_g) φ = 0` for buckling load factors and shapes — a generalized eigenproblem reusing the geometric stiffness from Section 8.5.

**9.6 Staged construction.** Let load patterns activate/deactivate element groups while carrying committed state forward. Architecturally this is a Domain feature (elements have an active flag; the AnalysisModel renumbers around inactive DOFs) plus a driving integrator. Powerful and distinctive; schedule it after nonlinear statics is solid. When combined with time-dependent materials (Section 5.4) and tendons (Section 4.6), each stage also advances time and applies prestress — this is how concrete bridges and tall buildings are really analyzed.

**9.7 Floor-vibration and frequency-domain analysis.** Beyond time-domain integration: solve the **steady-state** response to harmonic excitation by forming the complex dynamic stiffness `(K + iωC − ω²M) u = F` and sweeping frequency ω; extend to **power-spectral-density** response for random vibration; and add **footfall/floor-vibration** checks (response of floor systems to walking/rhythmic loads). These reuse the mass and damping matrices and the eigen/modal basis you already built for Section 9.2 — no new element or material work.

**9.8 Energy accounting.** During nonlinear (especially time-history) analysis, track the running energy balance: input work versus recoverable strain energy, kinetic energy, and dissipated **damping** and **hysteretic (plastic)** energy. It is cheap to accumulate incrementally, it is a standard performance-based-design output, and it doubles as a powerful global diagnostic — a violated energy balance is a reliable red flag that state management or the time integration is wrong.

---

## 10. Loads, load generation, and specialized analysis modes

The core solves for response given loads. Several distinctive SAP2000 capabilities live not in the elements or solver but in **how loads are defined and generated** and in **specialized static modes** layered on the same solver. The theme of this whole section is "same solver, different orchestration" — none of it requires changing the core `Element`/`Material`/`Solver` interfaces, which is precisely why the Section 2 architecture pays off.

**10.1 Load patterns, cases, and combinations (the loading data model).** Build this early; every analysis mode consumes it.
- A `LoadPattern` groups spatially distributed loads (nodal forces/moments; member point/distributed/trapezoidal loads; gravity/self-weight; thermal; prestrain; support settlement) and is scaled in time by a `TimeSeries`.
- A load *case* pairs a pattern (or patterns) with an analysis type (linear/nonlinear, static/dynamic) and a *starting state* (so a nonlinear case can continue from the end of a gravity case).
- Load *combinations* combine case results linearly, or by envelope / absolute-sum / SRSS; support step-by-step combination for staged or multi-step cases.

**10.2 Automatic load generation.** Generate code-prescribed load patterns instead of requiring manual entry. Each generator is a self-contained module that emits standard `LoadPattern`s — no solver changes:
- **Seismic**: equivalent-lateral-force distributions and response-spectrum functions per code (base shear distributed up the height).
- **Wind**: pressures on exposed surfaces per code exposure/height profiles.
- **Wave / current / buoyancy** for offshore and marine structures (static and dynamic, capturing drag and inertial/added-mass effects).
- **Vehicle live loads**: standard design-vehicle libraries that feed the moving-load solver (10.3).
Implement only the specific codes you care about; the framework itself is code-agnostic.

**10.3 Moving loads and influence-based analysis.** For bridges and crane rails, the governing response depends on load *position*:
- **Influence lines/surfaces**: the response at a target location due to a unit load placed at every point on the structure. Compute them as a set of unit-load static solves — and do it efficiently by **factorizing the stiffness once and back-substituting for many right-hand sides**.
- **Moving-load analysis**: convolve influence data with vehicle load patterns and **envelope** the extreme (max/min) of every response quantity at every location. This is a specialized static mode that drives the existing linear solver many times, so the multi-RHS reuse above is what makes it practical.

**10.4 Soil-structure interaction and foundation support.** Real structures sit on deformable ground:
- Foundation springs (linear or nonlinear) and elastic (Winkler) support beds beneath members and areas.
- **Compression-only** supports, uplift, and support plasticity — reuse the nonlinear link/spring machinery from Section 4.5 rather than writing anything new.
- Advanced option: an explicit soil continuum (solid elements) with far-field/absorbing boundaries.

**10.5 Response extraction and derived outputs.** Post-processing that is part of the engine, not a GUI:
- **Section cuts**: integrate element stresses/forces across a user-defined plane or line to recover resultant forces (e.g., total shear across a wall, force across a cut).
- **Generalized displacements**: user-defined linear combinations of nodal DOFs reported as one quantity (e.g., inter-story drift), and usable as the **control DOF** for displacement-controlled pushover (Section 8.3).
- Standard recovery: member force/moment diagrams, shell stress resultants, reactions, and modal/step envelopes — all driven by the `Recorder` system (Section 2).

---

## 11. Meshing and mesh quality — where nonlinear results are won or lost

Every section above assumes a mesh already exists. Producing a *good* one is a subsystem in its own right, and for nonlinear analysis it is frequently the difference between trustworthy results and expensive nonsense. The governing principle: **the mesh must be compatible (conforming), well-shaped, and — for softening materials — objective**, or the same structure will give different answers depending on how it happened to be discretized.

**11.1 Mesh compatibility (conformity) — the node-to-node requirement.**
- Adjacent elements must share the *same* nodes along a shared edge or face so the displacement field is continuous across the boundary. A conforming mesh transfers force and displacement across interfaces exactly, with no gaps or overlaps.
- A non-conforming interface — a node landing mid-edge of its neighbor (a "hanging node" / T-junction) — breaks that continuity: the edge can separate or interpenetrate under load, producing spurious stress concentrations and a solution that reflects meshing artifacts rather than physics.
- **Consequence for consistency:** two meshes of the same structure differing only in how interfaces connect can give materially different answers if one is non-conforming. Node-to-node conformity is a *precondition* for reproducible results — exactly as you said.

**11.2 Handling unavoidable mesh mismatch.**
- When two regions must be meshed independently (different densities, dissimilar element types, a fine local zone against a coarse global one), stitch them with **tied / edge constraints** — MPCs that interpolate the finer edge's DOFs onto the coarser edge (SAP2000's "automatic edge constraints"). These reuse the multi-point-constraint machinery from Section 6.
- But edge constraints are an *approximation*: they enforce compatibility in an averaged sense and can smear or concentrate stress at the interface. Prefer a truly conforming mesh; use tied interfaces only where conformity is impractical, and keep them away from regions of interest (plastic hinges, stress concentrations).

**11.3 Mesh quality metrics (check before you analyze).**
- Aspect ratio, skew, interior angles (avoid values near 0°/180°), quad warping (out-of-plane), and — non-negotiable — a **positive Jacobian at every integration point**. A zero or negative Jacobian means an inverted or degenerate element and a meaningless stiffness matrix.
- Distorted elements degrade integration accuracy; under nonlinear iteration those errors compound step over step. Enforce quality thresholds and refuse to run (or auto-repair) below them.

**11.4 Why nonlinear analysis is far less forgiving.**
- **Strain-softening localization / mesh objectivity — the big one.** When a material softens (concrete cracking/crushing, damage), deformation localizes into a band one element wide. Without regularization the dissipated energy — and hence the whole global load-deflection response — *changes with element size*: refine the mesh and you get a different, non-converging answer. This directly violates "consistent results." Fix it with **regularization**: fracture-energy-based softening (the crack-band approach, scaling the softening branch by an element characteristic length so dissipation is mesh-independent), or nonlocal / gradient-enhanced damage models. Any engine that does softening *must* address this or its nonlinear results are mesh artifacts.
- **Refinement where the action is.** Plastic-hinge regions, contact zones, re-entrant corners, and stress concentrations need locally finer meshes; elsewhere coarse is fine. Use graded meshes with conforming transitions to balance cost and accuracy.
- **Element behavior is mesh-coupled.** Locking and hourglassing (Section 4.3) depend on both formulation *and* mesh; a mesh fine enough for a linear run can still lock or hourglass under bending or near-incompressible plastic flow. Validate element behavior on the actual mesh, not in the abstract.
- **Determinism.** For run-to-run reproducibility, mesh generation and node/equation numbering must be deterministic — a stable, defined ordering, with no hash-set iteration order leaking into node numbers. Nondeterministic numbering combined with finite-precision arithmetic is a genuine source of "why did the same model give slightly different results" variation.

**11.5 Meshing approaches.**
- **Structured / mapped meshing** for regular geometries: produces clean, conforming quad (2D) and hex (3D) meshes with excellent quality and predictable connectivity — the preferred default wherever the geometry allows, and the right thing to build first.
- **Unstructured meshing** (Delaunay, advancing-front) for arbitrary geometry: flexible but requires active quality control and tends toward tets/tris, which are stiffer and need more refinement than hexes/quads.
- **Transitions and refinement**: use conforming templates to go coarse-to-fine without hanging nodes; keep transitions gradual.
- **Wrap, don't rewrite** (same logic as the solver in Section 7): consider embedding a proven mesher such as Gmsh, Netgen, or TetGen rather than writing a robust unstructured 3D mesher from scratch.
- **Per-archetype structured meshers (your actual priority).** Because models are *described* via parametric archetypes (Section 2) rather than imported from CAD, each archetype can ship its own small structured mesh generator that emits guaranteed-conforming elements (a beam → a line of frame elements or a mapped brick mesh; a slab/wall → a structured quad grid). This is why robust unstructured CAD meshing is a *low-priority, late* concern for your use case — build the archetype meshers first, and reach for a general unstructured mesher only for genuinely irregular one-off geometries.

**11.6 Validate the mesh, then validate on the mesh.** Run a **mesh-convergence study**: refine and confirm the response converges to a stable value — and, for softening problems, that regularization actually makes it converge rather than drift. This is part of V&V (Section 13) and is the empirical proof that the mesh is fine enough and the results are mesh-independent.

---

## 12. Technology stack — recommendation and trade-offs

There is no single right answer; here are the credible paths with honest trade-offs. The architecture in Section 2 is language-agnostic, so you can even switch languages between the prototype and the production core.

**Path A — Python-first prototype, then hot-path port (recommended for AI-assisted development).**
- Build the entire framework in Python with **NumPy** + **SciPy sparse**. You will get correct, verified results *fast*, and an AI can write, test, and refactor Python far more reliably than C++.
- Validate every formulation against the benchmarks in Section 13.
- Then profile and port only the hot loops (element state determination, assembly, the solve wrapper) to a compiled backend — **C++ via pybind11**, **Rust via PyO3**, or **Numba/Cython** — keeping the verified Python as the executable specification and regression oracle.
- Best for: correctness-first development, tight human-in-the-loop iteration, an AI doing most of the coding.

**Path B — C++ core with a Python scripting layer (mirrors OpenSees exactly).**
- C++ for the engine; **Eigen** for linear algebra early, swapping to **SuiteSparse/MUMPS/PARDISO** later; **pybind11** to expose a Python scripting API that mirrors OpenSees/OpenSeesPy commands (a well-known, battle-tested API design worth imitating).
- Best for: eventual performance and the closest match to the proven reference implementation. Higher up-front cost; C++ is less forgiving for AI-generated code, so pair it with aggressive automated testing.

**Path C — Julia (strong modern middle ground).**
- Near-C performance with high-level expressiveness; excellent native sparse/dense linear algebra (`SparseArrays`, LAPACK/BLAS), `Arpack.jl` for eigenproblems, and an existing low-level FE toolkit (`Ferrite.jl`) you can build on rather than starting from zero.
- Best for: one-language project that stays fast without a prototype/port split. Smaller ecosystem and talent pool than Python/C++.

**Path D — Rust.**
- Memory safety + performance; `nalgebra`, `sprs`, `faer` for linear algebra. Attractive for reliability, but the numerical/FEA ecosystem is thinner than C++/Python, so you'll wrap C libraries (MUMPS, SuiteSparse) via FFI more often.

**Concrete recommendation.** If the goal is to move fast with an AI and *get the physics right*, take **Path A** and design the interfaces (Section 2) so cleanly that a later reimplementation in C++ (Path B) or Julia (Path C) is mechanical. The verified Python engine becomes your regression test suite for the fast version. Whatever you choose, **wrap the linear solver and the eigen-solver behind thin interfaces from day one** so the numerical backend is swappable.

**Model input format.** Skip a GUI, but define a clean text/JSON (or Python-script) model definition from the start — nodes, elements, sections, materials, constraints, load patterns, analysis commands. An OpenSeesPy-style scripting API (imperative commands that populate the Domain) is a proven, ergonomic choice and doubles as your test-authoring language. This is the **Tier-1 API** of the description-driven input layer (Section 2); layer the **Tier-2 parametric archetype library** on top of it, and let the coding AI translate prose descriptions into archetype calls. Keeping Tier-1 explicit and inspectable is what makes AI-generated models auditable.

---

## 13. Verification and validation — the part that makes it trustworthy

An FEA engine that gives wrong answers confidently is worse than useless, and structural results are often not obviously wrong by eye. **Build the test suite alongside the code, not after.** Treat V&V as a first-class subsystem.

**Tiered testing strategy:**
1. **Unit tests on the math**: shape functions form a partition of unity; the Jacobian is correct; an unconstrained element stiffness has the right number of rigid-body (zero-energy) modes; `K` is symmetric; every constitutive **tangent matches a finite-difference of its own force output** (this single test catches most convergence bugs).
2. **Single-element patch tests**: a mesh subjected to a constant-strain field must reproduce that field exactly — the classic FE correctness test for continuum/shell elements.
3. **Closed-form benchmarks**: cantilever tip deflection (`PL³/3EI`), simply-supported beam, Euler buckling load (`π²EI/L²`), a truss with a hand-computable answer, a single-DOF oscillator's natural frequency and its transient response vs. the analytical solution.
4. **Nonlinear benchmarks with known answers**: elastic-plastic cantilever forming a plastic hinge at the analytical collapse load; large-deflection elastica (Euler's elastica has closed-form solutions); a snap-through arch traced by arc-length.
5. **Cross-validation against a reference engine**: run identical models in **OpenSees** (free) — and, where you have access, SAP2000 — and compare displacements, member forces, frequencies, and full response histories. This is the most powerful validation you have; automate it.
6. **Convergence studies**: confirm mesh refinement converges to the right answer at the theoretical rate, and that force-based frame elements need far fewer elements than displacement-based ones for the same nonlinear accuracy (a good sanity check that your force-based formulation is correct).

Wire these into CI so every change is re-verified. The credibility of the entire project rests on this suite.

---

## 14. Phased implementation roadmap

Each phase is independently useful and ends with a working, tested engine. Do not start a phase until the previous phase's tests pass.

**Phase 0 — Skeleton & core abstractions.**
Domain + tagged object storage; Node/Element/Material base interfaces; DOF numbering and assembly machinery; a wrapped linear solver (Eigen or SciPy); the `Analysis` component aggregation (even if only LinearStatic exists); the **Tier-1 scripting API** that populates the Domain (Section 2). *Deliverable: it compiles/runs and assembles a global system.*

**Phase 1 — Linear statics MVP.**
Truss + linear elastic 3D frame element; SP constraints; nodal + member-distributed + self-weight loads; the load-pattern/case/combination data model (Section 10.1); elastic cross-section property computation (Section 5.5); the **first Tier-2 archetypes** (simply supported beam, cantilever, portal frame) that expand a description into a meshed model (Section 2); load cases and combinations. RCM ordering. *Deliverable: matches closed-form beam/truss/frame benchmarks to machine precision. This alone is a usable linear structural analysis program.*

**Phase 2 — Constraints, shells, solids, solver upgrade.**
MP constraints (rigid diaphragm, equalDOF) via transformation handler; MITC4 shell; hex/tet solids; a structured (mapped) **conforming mesher** with mesh-quality checks (aspect ratio, positive Jacobian) and guaranteed node-to-node compatibility, plus edge-constraint stitching for unavoidable mismatch (Section 11); patch tests passing; swap in CHOLMOD/MUMPS for larger models; section cuts and generalized displacements (Section 10.5). *Deliverable: real building models (frames + floors + walls) solve correctly.*

**Phase 3 — Modal & linear dynamics.**
Mass matrices; generalized eigen-solver (ARPACK/Spectra) with shift-invert and auto-shift; participation factors/effective mass; Rayleigh damping; modal time-history and response-spectrum (SRSS/CQC); influence lines and moving-load enveloping via factorize-once/multi-RHS (Section 10.3); optional steady-state/frequency-domain response (Section 9.7). *Deliverable: frequencies, mode shapes, seismic response, and moving-load envelopes match references.*

**Phase 4 — Geometric nonlinearity.**
Geometric stiffness / P-Delta; corotational transformation for large displacement; load control + Newton-Raphson + convergence tests + line search; eigenvalue buckling. *Deliverable: buckling loads and large-deflection benchmarks pass.*

**Phase 5 — Material nonlinearity.**
Uniaxial nonlinear materials; fiber sections; force-based (and displacement-based) nonlinear frame elements; J2 plasticity `NDMaterial` with consistent tangent; confined/unconfined (Mander) concrete and a steel–concrete bond-slip material (Section 5.6); layered nonlinear shell; nonlinear links (gap/hook/multilinear/damper/isolator); displacement control + **pushover**; nonlinear moment-curvature / P-M interaction tool (Section 5.5); energy accounting (Section 9.8); mesh-objectivity regularization for strain-softening (fracture-energy / crack-band) and refinement in plastic-hinge zones (Section 11.4). *Deliverable: distributed-plasticity frames and pushover curves match OpenSees, with verified mesh-independent softening response.*

**Phase 6 — Advanced solution & path-following.**
Arc-length (Riks/Crisfield); automatic step-size control and event-to-event; quasi-Newton/BFGS; nonlinear direct-integration time-history (Newmark, HHT) wrapping the nonlinear iteration; nonlinear foundation springs and compression-only/uplift supports (Section 10.4). *Deliverable: snap-through and nonlinear seismic time-histories.*

**Phase 7 — Staged construction & productionization.**
Element activation/deactivation with state carry-forward; sequential-construction integrator; time-dependent concrete (creep/shrinkage/aging, Section 5.4); tendons/prestress with the full loss chain and target-force iteration (Section 4.6); catenary cables (Section 4.6); the **deterioration overlay** (corrosion section loss, ductility/bond degradation, cover-fiber spalling, confinement loss) driven over time on fiber sections + staged deactivation, plus the RC-assessment archetypes (Section 5.6); automatic code-based load generation for seismic/wind/wave/vehicle (Section 10.2); unstructured (Delaunay/advancing-front) meshing for arbitrary geometry, ideally by wrapping a proven mesher (Gmsh/Netgen/TetGen, Section 11.5); performance tuning, parallel assembly, and (optionally) parallel/iterative solvers for large models; a stable scripting API and model I/O. *Deliverable: staged bridge/tall-building workflows.*

**(Later, separable) Design-check modules.** AISC/ACI/Eurocode member checks as post-processors on analysis results — a large but independent effort outside the analysis core.

---

## 15. Known pitfalls (tell your coding AI to watch for these)

- **Inconsistent tangents** silently kill Newton convergence. Finite-difference-check every stiffness/tangent against its own force output as an automated test.
- **Element locking and hourglassing** in shells/solids give plausible-but-wrong stiffness. Patch tests are mandatory; be deliberate about reduced vs. full integration and hourglass control.
- **Mesh non-conformity and softening mesh-dependence.** Non-matched (hanging-node) interfaces break displacement continuity and produce mesh-artifact results; and under strain-softening, deformation localizes into a one-element band so that, without fracture-energy or nonlocal regularization, refining the mesh *changes* the answer instead of converging it. Both silently destroy result consistency — enforce conforming meshes and regularize softening (Section 11).
- **Rigid-body / zero-energy modes**: an unconstrained element must have exactly the right number of them; too few means it's over-stiff (locking), too many means it's rank-deficient (mechanism). Check eigenvalues of element stiffness.
- **Ill-conditioning from penalty constraints and from mixing very stiff and very soft elements** — prefer transformation/Lagrange handling for critical constraints; watch solver residuals.
- **Large rotations are not vectors** — naive addition of rotational DOFs breaks in 3D large-displacement analysis. Corotational/proper rotation update handling is required.
- **Path-dependence and state management**: never advance material/element state until a step *converges*. Get `commitState()` / `revertToLastCommit()` right or nonlinear results become nonsense on step retries.
- **Force-based element state determination** has its own inner iteration; a subtle bug here shows up as slow/failed global convergence rather than an obvious wrong answer.
- **Mass matrix and unit consistency** errors produce frequencies off by clean factors (e.g., √g) — a good diagnostic when modes look "almost right."
- **Don't hand-roll a sparse direct solver for production.** Wrap SuiteSparse/MUMPS/PARDISO; your time is better spent on formulations and V&V.
- **Deterioration models are empirical — calibrate, don't trust blindly.** The relations linking corrosion to bond loss, effective ductility, spalling onset, and confinement decay are scattered and specimen-dependent. Tie them to experimental benchmarks, expose the key parameters, and present corrosion-vs-capacity predictions as *calibrated estimates* with sensitivity ranges, not first-principles truth.
- **Audit AI-generated models.** When prose is translated into Tier-2/Tier-1 calls, the generated model can be silently wrong (mismatched units, a bar in the wrong layer, an unintended restraint). Keep Tier-1 human-inspectable, echo a model summary (masses, member/section counts, support conditions, total applied load), and sanity-check reactions against statics before trusting any result.

---

## 16. Primary references (authoritative, and useful to feed the coding AI)

**Software architecture (the blueprint):**
- F. McKenna, *Object-Oriented Finite Element Programming: Frameworks for Analysis, Algorithms and Parallel Computing*, PhD thesis, UC Berkeley, 1997 — the design behind OpenSees.
- OpenSees source code, developer wiki, and OpenSeesPy documentation — the living reference implementation of every abstraction in this document (Domain, Element, Material, the Analysis component aggregation, integrators, algorithms). Read the code; it is the single best teacher here.
- CSI, *CSI Analysis Reference Manual* (the SAP2000/ETABS/SAPFire analysis manual) — the authoritative statement of what SAP2000's engine does and the definitions of its options (P-Delta, Ritz vectors, staged construction, link elements, hinge models).

**Structural FE theory:**
- McGuire, Gallagher & Ziemian, *Matrix Structural Analysis* — direct stiffness method, frames, geometric nonlinearity; the standard structural-analysis text (freely available 2nd ed.).
- Bathe, *Finite Element Procedures* — the comprehensive reference for elements, nonlinear solution, and dynamics.
- Crisfield, *Non-linear Finite Element Analysis of Solids and Structures* — nonlinear solution procedures, arc-length, corotational formulations.
- Zienkiewicz & Taylor, *The Finite Element Method* — general FEM, continuum/shell elements, locking.
- Filippou & Fenves and related PEER work on **force-based fiber beam-column elements** — the accurate nonlinear frame formulation.
- Chopra, *Dynamics of Structures* — modal analysis, response spectrum, time-history integration.
- Simo & Hughes, *Computational Inelasticity* — return-mapping plasticity and consistent tangents.

**Numerical libraries (wrap, don't rewrite):**
- Eigen (dense/sparse C++); SciPy sparse (Python prototype).
- SuiteSparse — CHOLMOD (SPD Cholesky), UMFPACK (LU), KLU.
- MUMPS and Intel MKL PARDISO — parallel sparse direct solvers for large models.
- ARPACK / Spectra / `scipy.sparse.linalg.eigsh` — eigenproblems for modal and buckling analysis.
- METIS (fill-reducing ordering); PETSc (parallel iterative solvers, only when needed).

---

### One-paragraph starting instruction for the coding AI

> Build a headless nonlinear structural finite-element engine using the OpenSees object model: a `Domain` holding tagged `Node`, `Element`, `Material`, `Section`, `Constraint`, and `LoadPattern` objects, analyzed by a swappable aggregation of `ConstraintHandler`, `DOF_Numberer`, `SystemOfEqn`+`Solver`, `Integrator`, `SolutionAlgorithm`, and `ConvergenceTest`. Start in Python with NumPy/SciPy for correctness, wrapping the linear solve and eigen-solve behind thin interfaces. Implement Phase 1 (linear truss + 3D elastic frame, assembly, SP constraints, direct solve) and prove it against closed-form beam/truss benchmarks and a cross-check in OpenSees before adding anything else. Enforce the invariant that every element/material returns a resisting force *and* its consistent tangent, verified by an automated finite-difference test. Then proceed phase by phase through the roadmap in Section 14, never advancing until the current phase's verification suite passes.
