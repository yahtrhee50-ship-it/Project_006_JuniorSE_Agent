"""
Hibbeler Structural Analysis Ch 15 — Benchmark Problem 15-3

Geometry (metric units treated as ft/kip/kip-ft in solver; EI cancels):
  N1 (x=0)  fixed                     ← NOTE: reference file says "roller" but
  N2 (x=12) roller, push/pull           textbook solution constrains θ_N1 (code 5),
  N3 (x=20) roller, push/pull           so N1 is FIXED (moment reaction exists there).

Loads:
  6 kN/m UDL on span 1 (x=0 to x=12)
  20 kN·m concentrated CCW moment at N3 (x=20)
  — Hibbeler Qk at code 1 (θ_N3) = +20 → CCW in solver convention.

Textbook answers (Hibbeler, p. 520-522):
  R_N3 = -7.853 kN  ≈ 7.85 kN downward  (roller pulls beam down)
  R_N2 = 40.21  kN  upward
  M_N1 = 86.59  kN·m  (moment reaction at fixed support, N1)
  R_N1 = 39.64  kN  upward

Statics check: 39.64 + 40.21 - 7.853 = 71.997 ≈ 72 kN = 6×12  ✓
"""

import sys
sys.path.insert(0, ".")

from src.calcs.beam_stiffness import GeneralBeam

TOL_F = 0.5    # kN tolerance for reaction forces
TOL_M = 1.0   # kN·m tolerance for moment reactions

def run():
    # EI=1 (UDL-only for most results; EI cancels out for statics)
    b = GeneralBeam(total_length_ft=20, E_ksi=1, I_in4=1, n_elements=400)
    b.set_bc("fixed", "roller")       # N1 fixed at x=0, N3 roller at x=20
    b.add_support(x_ft=12, bc="pin") # N2 roller at x=12
    b.add_udl(w_kip_per_ft=6, x_start_ft=0, x_end_ft=12)
    # Solver convention: positive M_kip_ft = CW.  Textbook moment is CCW (+20 in
    # Hibbeler's Qk), so input is -20 here.
    b.add_point_moment(M_kip_ft=-20, x_ft=20)

    r = b.solve()

    # Reactions at roller supports (upward = positive in solver)
    R_N1 = r.reactions.get(0.0,  r.reactions.get(0,  None))
    R_N2 = r.reactions.get(12.0, r.reactions.get(12, None))
    R_N3 = r.reactions.get(20.0, r.reactions.get(20, None))

    # Moment reaction at fixed support N1
    M_N1 = r.moment_reactions.get(0.0, r.moment_reactions.get(0, None))
    abs_M_N1 = abs(M_N1) if M_N1 is not None else None

    print("=" * 58)
    print("Hibbeler Ch 15 — Problem 15-3 Benchmark")
    print("=" * 58)
    print(f"  reactions dict       : {r.reactions}")
    print(f"  moment_reactions dict: {r.moment_reactions}")
    print()
    print(f"  R_N1  solver = {R_N1:.4f}  kN       textbook = +39.64")
    print(f"  R_N2  solver = {R_N2:.4f}  kN       textbook = +40.21")
    print(f"  R_N3  solver = {R_N3:.4f}  kN       textbook = -7.853")
    print(f"  |M_N1| solver = {abs_M_N1:.4f}  kN·m    textbook =  86.59")
    print()

    statics_sum = (R_N1 or 0) + (R_N2 or 0) + (R_N3 or 0)
    print(f"  Statics check SumFy = {statics_sum:.4f}  kN  (expect ~72.0)")
    print()

    pass_R_N1 = R_N1 is not None  and abs(R_N1  - 39.64)  < TOL_F
    pass_R_N2 = R_N2 is not None  and abs(R_N2  - 40.21)  < TOL_F
    pass_R_N3 = R_N3 is not None  and abs(R_N3  - (-7.853)) < TOL_F
    pass_M_N1 = abs_M_N1 is not None and abs(abs_M_N1 - 86.59) < TOL_M

    print(f"  R_N1   {'PASS' if pass_R_N1 else 'FAIL'}")
    print(f"  R_N2   {'PASS' if pass_R_N2 else 'FAIL'}")
    print(f"  R_N3   {'PASS' if pass_R_N3 else 'FAIL'}")
    print(f"  |M_N1| {'PASS' if pass_M_N1 else 'FAIL'}")
    print()

    all_pass = pass_R_N1 and pass_R_N2 and pass_R_N3 and pass_M_N1
    if all_pass:
        print("  RESULT: PASS — solver matches textbook")
    else:
        print("  RESULT: FAIL — discrepancy exceeds tolerance")

    return all_pass

if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)
