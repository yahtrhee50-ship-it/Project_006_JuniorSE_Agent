"""
Hibbeler Structural Analysis Ch 15 — Benchmark Problem 15-8

Geometry (metric mapped to solver: kN→kip, m→ft, EI cancels):
  N1  (x=0)  fixed
  N2  (x=4)  internal hinge (Gerber — coincident nodes N2L / N2R)
  N3  (x=7)  roller

Spans: M1 = 4 m (fixed → hinge), M2 = 3 m (hinge → roller)

Load:
  Triangular 15 kN/m at x=4 (N2) → 0 at x=7 (N3), on M2 only.
  Total applied: ½ × 15 × 3 = 22.5 kN downward.

Textbook answers (Hibbeler):
  R₅  = 7.50  kN ↑  (roller at N3)
  R₆  = 15.0  kN ↑  (fixed  at N1, vertical)
  M₇  = 60.0  kN·m  counterclockwise (fixed at N1, moment)

Statics check: 7.50 + 15.0 = 22.5 kN ✓

Note: n_elements=700 chosen so hinge at x=4 lands on an exact node
(4/7 × 700 = 400, an integer).
"""

import sys
sys.path.insert(0, ".")

from src.calcs.beam_stiffness import GeneralBeam

TOL_F = 0.05   # kN  — textbook gives 3 sig figs; expect exact match for this determinate problem
TOL_M = 0.05   # kN·m


def run():
    b = GeneralBeam(total_length_ft=7, E_ksi=1, I_in4=1, n_elements=700)
    b.set_bc("fixed", "roller")   # fixed at N1 (x=0), roller at N3 (x=7)
    b.add_hinge(x_ft=4)           # internal moment release at N2

    # Triangular load: 15 kN/m at x=4 (hinge), 0 at x=7 (roller)
    b.add_trapload(15, 0, x_start_ft=4, x_end_ft=7)

    r = b.solve()

    R_N1 = r.reactions.get(0.0, r.reactions.get(0, None))
    R_N3 = r.reactions.get(7.0, r.reactions.get(7, None))
    M_N1 = r.moment_reactions.get(0.0, r.moment_reactions.get(0, None))
    abs_M_N1 = abs(M_N1) if M_N1 is not None else None

    print("=" * 60)
    print("Hibbeler Ch 15 — Problem 15-8 Benchmark")
    print("=" * 60)
    print(f"  reactions dict       : {r.reactions}")
    print(f"  moment_reactions dict: {r.moment_reactions}")
    print()
    print(f"  R_N1    solver = {R_N1:+.4f} kN      textbook = +15.0")
    print(f"  R_N3    solver = {R_N3:+.4f} kN      textbook =  +7.5")
    print(f"  |M_N1|  solver = {abs_M_N1:.4f} kN·m  textbook =  60.0")
    print()

    applied = -22.5   # 22.5 kN downward
    statics_sum = (R_N1 or 0) + (R_N3 or 0)
    net = statics_sum + applied
    print(f"  Statics SumFy = {statics_sum:.4f} + ({applied}) = {net:.6f}  (expect 0.0)")
    print()

    pass_R_N1 = R_N1 is not None and abs(R_N1 - 15.0) < TOL_F
    pass_R_N3 = R_N3 is not None and abs(R_N3 -  7.5) < TOL_F
    pass_M_N1 = abs_M_N1 is not None and abs(abs_M_N1 - 60.0) < TOL_M

    print(f"  R_N1    {'PASS' if pass_R_N1 else 'FAIL'}")
    print(f"  R_N3    {'PASS' if pass_R_N3 else 'FAIL'}")
    print(f"  |M_N1|  {'PASS' if pass_M_N1 else 'FAIL'}")
    print()

    all_pass = pass_R_N1 and pass_R_N3 and pass_M_N1
    if all_pass:
        print("  RESULT: PASS — solver matches textbook")
    else:
        print("  RESULT: FAIL — discrepancy exceeds tolerance")

    return all_pass


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)
