"""
Hibbeler Structural Analysis Ch 15 — Benchmark Problem 15-6

Geometry (metric mapped to solver: kN→kip, m→ft, EI cancels):
  N1 (x= 0)  fixed
  N2 (x= 6)  roller (intermediate)
  N3 (x=14)  roller (right end)

Spans: M1 = 6 m, M2 = 8 m

Load:
  UDL 10 kN/m on BOTH spans (x=0 to x=14).
  Total applied: 10 × 14 = 140 kN downward.

Textbook answers (Hibbeler):
  Q3 = R_N3 = +32.25 kN  upward  (v at N3)
  Q4 = R_N2 = +85.75 kN  upward  (v at N2)
  Q5 = R_N1 = +22.0  kN  upward  (v at N1)
  Q6 = M_N1 =  14.0  kN·m        (θ at N1, moment reaction at fixed end)

Analytical check (three-moment equation):
  M_A = -14 kN·m, M_B = -62 kN·m → R_N1=22, R_N2=85.75, R_N3=32.25 ✓
"""

import sys
sys.path.insert(0, ".")

from src.calcs.beam_stiffness import GeneralBeam

TOL_F = 0.10   # kN tolerance for reactions
TOL_M = 0.10   # kN·m tolerance for moment reaction

def run():
    b = GeneralBeam(total_length_ft=14, E_ksi=1, I_in4=1, n_elements=560)
    b.set_bc("fixed", "roller")             # fixed at N1 (x=0), roller at N3 (x=14)
    b.add_support(x_ft=6, bc="pin")        # roller at N2 (x=6)

    b.add_udl(w_kip_per_ft=10, x_start_ft=0, x_end_ft=14)  # UDL on BOTH spans

    r = b.solve()

    R_N1 = r.reactions.get(0.0,  r.reactions.get(0,  None))
    R_N2 = r.reactions.get(6.0,  r.reactions.get(6,  None))
    R_N3 = r.reactions.get(14.0, r.reactions.get(14, None))
    M_N1 = r.moment_reactions.get(0.0, r.moment_reactions.get(0, None))
    abs_M_N1 = abs(M_N1) if M_N1 is not None else None

    print("=" * 60)
    print("Hibbeler Ch 15 — Problem 15-6 Benchmark")
    print("=" * 60)
    print(f"  reactions dict       : {r.reactions}")
    print(f"  moment_reactions dict: {r.moment_reactions}")
    print()
    print(f"  R_N3 (Q3) solver = {R_N3:+.4f} kN      textbook = +32.25")
    print(f"  R_N2 (Q4) solver = {R_N2:+.4f} kN      textbook = +85.75")
    print(f"  R_N1 (Q5) solver = {R_N1:+.4f} kN      textbook = +22.0")
    print(f"  |M_N1| (Q6) solver = {abs_M_N1:.4f} kN·m  textbook =  14.0")
    print()

    applied = -140.0
    statics_sum = (R_N1 or 0) + (R_N2 or 0) + (R_N3 or 0)
    net = statics_sum + applied
    print(f"  Statics SumFy = {statics_sum:.4f} + ({applied}) = {net:.4f}  (expect 0.0)")
    print()

    pass_R_N3 = R_N3 is not None and abs(R_N3 -  32.25) < TOL_F
    pass_R_N2 = R_N2 is not None and abs(R_N2 -  85.75) < TOL_F
    pass_R_N1 = R_N1 is not None and abs(R_N1 -  22.0)  < TOL_F
    pass_M_N1 = abs_M_N1 is not None and abs(abs_M_N1 - 14.0) < TOL_M

    print(f"  R_N3 (Q3) {'PASS' if pass_R_N3 else 'FAIL'}")
    print(f"  R_N2 (Q4) {'PASS' if pass_R_N2 else 'FAIL'}")
    print(f"  R_N1 (Q5) {'PASS' if pass_R_N1 else 'FAIL'}")
    print(f"  |M_N1| (Q6) {'PASS' if pass_M_N1 else 'FAIL'}")
    print()

    all_pass = pass_R_N3 and pass_R_N2 and pass_R_N1 and pass_M_N1
    if all_pass:
        print("  RESULT: PASS — solver matches textbook")
    else:
        print("  RESULT: FAIL — discrepancy exceeds tolerance")

    return all_pass

if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)
