"""
Hibbeler Structural Analysis Ch 15 — Benchmark Problem 15-4

Geometry (imperial — ft, kips; EI cancels):
  N1 (x= 0)  pin
  N2 (x=10)  roller, push/pull
  N3 (x=20)  roller, push/pull
  N4 (x=30)  free end

Loads:
  3 k downward point load at N4 (x=30)

Textbook answers (Hibbeler, p. 521-522):
  R_N3 = +6.75 k  upward   (Q6)
  R_N2 = -4.50 k  downward (Q7)  [roller pulls beam down]
  R_N1 = +0.75 k  upward   (Q8)

NOTE — textbook sign error: the solution manual prints Q8 = -0.75 k, but
statics (ΣFy = 6.75 - 4.5 - 0.75 - 3 = -1.5 ≠ 0) fails.  Recalculating
from the solved displacements (D1=-12.5/EI, D2=25/EI):
  Q8 = EI[0.06·D1 + 0.06·D2] = 0.06(-12.5) + 0.06(25) = +0.75 k
Statics check: 0.75 - 4.50 + 6.75 - 3 = 0.0 ✓
"""

import sys
sys.path.insert(0, ".")

from src.calcs.beam_stiffness import GeneralBeam

TOL_F = 0.05   # kip tolerance for reaction forces

def run():
    b = GeneralBeam(total_length_ft=30, E_ksi=1, I_in4=1, n_elements=600)
    b.set_bc("pin", "free")             # pin at N1 (x=0), free at N4 (x=30)
    b.add_support(x_ft=10, bc="pin")   # roller push/pull at N2
    b.add_support(x_ft=20, bc="pin")   # roller push/pull at N3
    b.add_point_load(x_ft=30, P_kips=3)   # 3 k downward at free end (positive = downward)

    r = b.solve()

    R_N1 = r.reactions.get(0.0,  r.reactions.get(0,  None))
    R_N2 = r.reactions.get(10.0, r.reactions.get(10, None))
    R_N3 = r.reactions.get(20.0, r.reactions.get(20, None))

    print("=" * 58)
    print("Hibbeler Ch 15 — Problem 15-4 Benchmark")
    print("=" * 58)
    print(f"  reactions dict: {r.reactions}")
    print()
    print(f"  R_N1  solver = {R_N1:+.4f} k   textbook = +0.75")
    print(f"  R_N2  solver = {R_N2:+.4f} k   textbook = -4.50")
    print(f"  R_N3  solver = {R_N3:+.4f} k   textbook = +6.75")
    print()

    statics_sum = (R_N1 or 0) + (R_N2 or 0) + (R_N3 or 0)
    applied = -3.0
    net = statics_sum + applied
    print(f"  Statics SumFy = {statics_sum:.4f} + ({applied}) = {net:.4f}  (expect 0.0)")
    print()

    pass_R_N1 = R_N1 is not None and abs(R_N1 -  0.75) < TOL_F
    pass_R_N2 = R_N2 is not None and abs(R_N2 - (-4.50)) < TOL_F
    pass_R_N3 = R_N3 is not None and abs(R_N3 -  6.75) < TOL_F

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
