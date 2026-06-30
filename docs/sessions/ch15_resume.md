# Ch 15 Benchmark Resume

**Goal:** Verify `GeneralBeam` stiffness solver against Hibbeler *Structural Analysis* Ch 15 problems.

**Solver:** `src/calcs/beam_stiffness.py` — `GeneralBeam` class  
**Tests:** `tests/test_general_beam.py` (85 total tests passing, all green)  
**Python:** `C:\Python314\python.exe`  
**Run tests:** `C:\Python314\python.exe -m pytest tests/ -q`

## GeneralBeam capabilities

```python
from src.calcs.beam_stiffness import GeneralBeam

b = GeneralBeam(total_length_ft, E_ksi, I_in4, n_elements=200)
b.set_bc("pin", "roller")                    # left/right: "free" / "pin" / "roller" / "fixed"
b.add_support(x_ft, bc="pin")               # intermediate support (kwarg is bc=, not support_type=)
b.set_support_settlement(x_ft, delta_in)    # prescribed vertical displacement at support (+ = up)
b.add_hinge(x_ft)                           # internal moment release (Gerber)
b.set_ei(x_start, x_end, E, I)             # variable EI per region
b.add_udl(w_kip_per_ft, x_start_ft, x_end_ft)  # UDL — note kwarg names (not x_start/x_end/w_kpf)
b.add_trapload(x_start, x_end, w_left, w_right)
b.add_point_load(x_ft, P_kips)
b.add_point_moment(x_ft, M_kft)            # concentrated moment (+ = CCW)

r = b.solve()
# r.reactions         dict {x_ft: V_kips}
# r.moment_reactions  dict {x_ft: M_kip_ft}  (fixed supports only)
# r.M_max_kip_ft, r.V_max_kips, r.delta_max_in
# r.x_ft, r.V_kips, r.M_kip_ft, r.delta_in  (arrays)
```

Multi-span shortcut:
```python
b = GeneralBeam.continuous(
    spans_ft=[20, 20],
    E_ksi=29000, I_in4=100,
    left_bc="pin", right_bc="pin",
    n_elements=200
)
```

## Metric-unit convention for Ch 15 problems

All Hibbeler Ch 15 problems use SI (kN, m). The solver is imperial.  
**Mapping:** treat kN as kip, m as ft (so 1 m = 12 solver-inches internally).

For UDL-only problems (EI cancels): use `E_ksi=1, I_in4=1` — answers come out in kN·m directly.

For problems with **prescribed settlement** (EI does NOT cancel):  
- `E_ksi = 1`  
- `I_in4 = EI_actual_kNm2 * 144`  (converts kN·m² → kip·in²: ×144 because 1 ft = 12 in)  
- `delta_in = delta_m * 12`  (metres → solver inches)  
- Moment output is directly in kN·m.

## add_point_moment sign convention

`add_point_moment(M_kip_ft, x_ft)` — **positive = CW (sagging convention)**,
NOT CCW as the docstring originally said (now corrected in the source).  
Hibbeler uses CCW = positive in his Qk vector. When a Hibbeler problem specifies a +M
at a node (CCW), input it as **negative** in the solver: `add_point_moment(-M, x)`.  
Verified by P3 benchmark: the CCW 20 kN·m moment in Hibbeler's Qk requires -20 in solver.

## Benchmark log

| Problem | Description | Textbook answers | Solver result | Status | Benchmark file |
|---|---|---|---|---|---|
| 15-1 | Fixed–roller–fixed, 25 kN/m UDL on 6m span (total 10m) | M₁=90.0, M₃=22.5 kN·m | M₁=90.00, M₃=22.50 | **PASS** | `tests/benchmark_ch15_p1.py` |
| 15-2 | Same geometry + +5mm upward settlement at roller, EI=60,000 kN·m² | M₁=27.5, M₃=116.25 kN·m | M₁=27.50, M₃=116.25 | **PASS** | `tests/benchmark_ch15_p2.py` |
| 15-3 | Fixed–roller–roller, 6 kN/m on 12m span + 20 kN·m CCW at right roller | R_N1=39.64, R_N2=40.21, R_N3=-7.85 kN; M_N1=86.59 kN·m | R_N1=39.65, R_N2=40.21, R_N3=-7.85; M_N1=86.59 | **PASS** | `tests/benchmark_ch15_p3.py` |
| 15-4 | Pin–roller–roller–free, 3 k point load at free end N4, three 10 ft spans | R_N1=+0.75, R_N2=-4.50, R_N3=+6.75 k (textbook prints Q8=-0.75 — sign error; statics confirms +0.75) | R_N1=+0.75, R_N2=-4.50, R_N3=+6.75 | **PASS** | `tests/benchmark_ch15_p4.py` |
| 15-5 | Pin–roller–roller, triangular 0→15 kN/m on 6m span, 8m second span unloaded | R_N1=+12.43, R_N2=+34.5, R_N3=-1.929 kN (roller in tension) | R_N1=+12.4286, R_N2=+34.5000, R_N3=-1.9286 | **PASS** | `tests/benchmark_ch15_p5.py` |

**Note on P3 geometry:** Reference file incorrectly labeled N1 as "roller." Textbook solution constrains θ_N1 (code 5) — N1 is FIXED. Moment reaction at N1 = 86.59 kN·m confirms this.

## Sign convention note — add_point_load

`add_point_load(P_kips, x_ft)` — **positive P = downward** (gravity convention, same as UDL).  
A 3 k downward load uses `P_kips=3`. Using negative here applies the load upward.  
Verified by P4 benchmark.

## Next action

Resume at **Problem 15-6** (or whichever the engineer chooses next).  
P15-6: Fixed at N1 (x=0), rollers at N2 (x=6) and N3 (x=14), UDL 10 kN/m on M1 (x=0 to x=6). Write `tests/benchmark_ch15_p6.py`.
