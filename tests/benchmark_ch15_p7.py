"""
Hibbeler Structural Analysis Ch 15 — Benchmark Problem 15-7

Geometry (metric mapped to solver: kN→kip, m→ft, EI cancels):
  N1 (x= 0)  fixed
  N2 (x= 6)  roller (intermediate support)
  N3 (x=10)  fixed

Spans: span 1 = 6 m, span 2 = 4 m

Loads:
  UDL  9 kN/m on span 1 (x=0 to x=6)
  UDL  6 kN/m on span 2 (x=6 to x=10)
  EI constant.

Analytical solution (slope-deflection, EI cancels):
  θ_B = −57/(5EI)
  M_A  = −154/5  = −30.8  kN·m  (hogging at fixed A)
  M_C  =  +23/10 =  +2.3  kN·m  (hogging at fixed C)
  Interior beam moment at B = 97/5 = 19.4 kN·m

Reactions:
  R_N1 =  867/30  =  28.9   kN  ↑
  R_N2 = 4965/120 =  41.375 kN  ↑
  R_N3 =  309/40  =   7.725 kN  ↑
  |M_N1| = 30.8  kN·m
  |M_N3| =  2.3  kN·m

Statics check: 28.9 + 41.375 + 7.725 = 78.0 = 9×6 + 6×4 ✓
ΣM_A = 0 verified analytically ✓
"""

import sys
sys.path.insert(0, ".")

from src.calcs.beam_stiffness import GeneralBeam

TOL_F = 0.10   # kN tolerance for reactions
TOL_M = 0.10   # kN·m tolerance for moment reactions

def run():
    b = GeneralBeam(total_length_ft=10, E_ksi=1, I_in4=1, n_elements=400)
    b.set_bc("fixed", "fixed")           # fixed at N1 (x=0) and N3 (x=10)
    b.add_support(x_ft=6, bc="pin")      # roller at N2 (x=6)

    b.add_udl(w_kip_per_ft=9, x_start_ft=0, x_end_ft=6)   # 9 kN/m on span 1
    b.add_udl(w_kip_per_ft=6, x_start_ft=6, x_end_ft=10)  # 6 kN/m on span 2

    r = b.solve()

    R_N1 = r.reactions.get(0.0,  r.reactions.get(0,  None))
    R_N2 = r.reactions.get(6.0,  r.reactions.get(6,  None))
    R_N3 = r.reactions.get(10.0, r.reactions.get(10, None))
    M_N1 = r.moment_reactions.get(0.0, r.moment_reactions.get(0, None))
    M_N3 = r.moment_reactions.get(10.0, r.moment_reactions.get(10, None))
    abs_M_N1 = abs(M_N1) if M_N1 is not None else None
    abs_M_N3 = abs(M_N3) if M_N3 is not None else None

    print("=" * 60)
    print("Hibbeler Ch 15 — Problem 15-7 Benchmark")
    print("=" * 60)
    print(f"  reactions dict       : {r.reactions}")
    print(f"  moment_reactions dict: {r.moment_reactions}")
    print()
    print(f"  R_N1     solver = {R_N1:+.4f} kN      textbook = +28.9")
    print(f"  R_N2     solver = {R_N2:+.4f} kN      textbook = +41.375")
    print(f"  R_N3     solver = {R_N3:+.4f} kN      textbook =  +7.725")
    print(f"  |M_N1|   solver = {abs_M_N1:.4f} kN·m  textbook =  30.8")
    print(f"  |M_N3|   solver = {abs_M_N3:.4f} kN·m  textbook =   2.3")
    print()

    applied = -(9 * 6 + 6 * 4)   # −78 kN
    statics_sum = (R_N1 or 0) + (R_N2 or 0) + (R_N3 or 0)
    net = statics_sum + applied
    print(f"  Statics SumFy = {statics_sum:.4f} + ({applied}) = {net:.6f}  (expect 0.0)")
    print()

    pass_R_N1 = R_N1 is not None and abs(R_N1 -  28.9)   < TOL_F
    pass_R_N2 = R_N2 is not None and abs(R_N2 -  41.375) < TOL_F
    pass_R_N3 = R_N3 is not None and abs(R_N3 -   7.725) < TOL_F
    pass_M_N1 = abs_M_N1 is not None and abs(abs_M_N1 - 30.8) < TOL_M
    pass_M_N3 = abs_M_N3 is not None and abs(abs_M_N3 -  2.3) < TOL_M

    print(f"  R_N1     {'PASS' if pass_R_N1 else 'FAIL'}")
    print(f"  R_N2     {'PASS' if pass_R_N2 else 'FAIL'}")
    print(f"  R_N3     {'PASS' if pass_R_N3 else 'FAIL'}")
    print(f"  |M_N1|   {'PASS' if pass_M_N1 else 'FAIL'}")
    print(f"  |M_N3|   {'PASS' if pass_M_N3 else 'FAIL'}")
    print()

    all_pass = pass_R_N1 and pass_R_N2 and pass_R_N3 and pass_M_N1 and pass_M_N3
    if all_pass:
        print("  RESULT: PASS — solver matches analytical solution")
    else:
        print("  RESULT: FAIL — discrepancy exceeds tolerance")

    return all_pass

if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)
