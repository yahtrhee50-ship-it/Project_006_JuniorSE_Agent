"""
Hibbeler Structural Analysis Ch 15 — Benchmark Problem 15-5

Geometry (metric mapped to solver: kN→kip, m→ft, EI cancels):
  N1 (x= 0)  pin
  N2 (x= 6)  roller (intermediate)
  N3 (x=14)  roller (right end)

Spans: M1 = 6 m, M2 = 8 m

Load:
  Triangular 0 → 15 kN/m on M1 (x=0 to x=6); zero load on M2.
  Total applied: ½ × 15 × 6 = 45 kN downward.

Textbook answers (Hibbeler, p. 523-524):
  R_N3 (Q4) = -1.929 kN  (1.93 kN in tension — roller pulls down on beam)
  R_N2 (Q5) = +34.5  kN  upward
  R_N1 (Q6) = +12.43 kN  upward  (textbook rounds to 12.4 kN)

Statics check: 12.43 + 34.5 - 1.929 = 45.001 ≈ 45 kN ✓
"""

import sys
sys.path.insert(0, ".")

from src.calcs.beam_stiffness import GeneralBeam

TOL_F = 0.10   # kN tolerance (textbook rounds to 3 sig figs)

def run():
    b = GeneralBeam(total_length_ft=14, E_ksi=1, I_in4=1, n_elements=560)
    b.set_bc("pin", "roller")             # pin at N1 (x=0), roller at N3 (x=14)
    b.add_support(x_ft=6, bc="pin")      # roller at N2 (x=6)

    # Triangular load: 0 at x=0, 15 kN/m at x=6 (positive = downward)
    b.add_trapload(0, 15, x_start_ft=0, x_end_ft=6)

    r = b.solve()

    R_N1 = r.reactions.get(0.0,  r.reactions.get(0,  None))
    R_N2 = r.reactions.get(6.0,  r.reactions.get(6,  None))
    R_N3 = r.reactions.get(14.0, r.reactions.get(14, None))

    print("=" * 58)
    print("Hibbeler Ch 15 — Problem 15-5 Benchmark")
    print("=" * 58)
    print(f"  reactions dict: {r.reactions}")
    print()
    print(f"  R_N1  solver = {R_N1:+.4f} kN   textbook = +12.43")
    print(f"  R_N2  solver = {R_N2:+.4f} kN   textbook = +34.50")
    print(f"  R_N3  solver = {R_N3:+.4f} kN   textbook =  -1.929  (1.93 kN T)")
    print()

    applied = -45.0   # 45 kN downward (negative in ΣFy)
    statics_sum = (R_N1 or 0) + (R_N2 or 0) + (R_N3 or 0)
    net = statics_sum + applied
    print(f"  Statics SumFy = {statics_sum:.4f} + ({applied}) = {net:.4f}  (expect 0.0)")
    print()

    pass_R_N1 = R_N1 is not None and abs(R_N1 - 12.43)  < TOL_F
    pass_R_N2 = R_N2 is not None and abs(R_N2 - 34.50)  < TOL_F
    pass_R_N3 = R_N3 is not None and abs(R_N3 - (-1.929)) < TOL_F

    print(f"  R_N1  {'PASS' if pass_R_N1 else 'FAIL'}")
    print(f"  R_N2  {'PASS' if pass_R_N2 else 'FAIL'}")
    print(f"  R_N3  {'PASS' if pass_R_N3 else 'FAIL'}")
    print()

    all_pass = pass_R_N1 and pass_R_N2 and pass_R_N3
    if all_pass:
        print("  RESULT: PASS — solver matches textbook")
    else:
        print("  RESULT: FAIL — discrepancy exceeds tolerance")

    return all_pass

if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)
