"""
Hibbeler Structural Analysis Ch 15 — Benchmark Problem 15-2

Same geometry as 15-1:
  N1 (x=0)  fixed
  N2 (x=6)  roller, upward settlement +5 mm = +0.005 m
  N3 (x=10) fixed

Load: 25 kN/m UDL on M1 (x=0 to x=6)
EI = 60×10⁶ N·m² = 60,000 kN·m²

Unit mapping (kN↔kip, m↔ft):
  EI [kip·in²] = 60,000 [kN·m²] × 144 [in²/ft²] = 8,640,000
  delta [in]   = 0.005 [m] × 12 [in/ft] = 0.06

Expected (textbook):
  M1 = 27.5 kN·m  (moment reaction at left fixed support)
  M3 = 116.25 kN·m (moment reaction at right fixed support)
"""

import sys
sys.path.insert(0, ".")

from src.calcs.beam_stiffness import GeneralBeam

TOL = 0.5   # kN·m

def run():
    EI_solver = 60_000 * 144       # 8,640,000 kip·in²  (EI=60,000 kN·m² in solver units)
    E_ksi     = 1.0
    I_in4     = float(EI_solver)   # E=1 ksi, I = 8,640,000 in4

    b = GeneralBeam(total_length_ft=10, E_ksi=E_ksi, I_in4=I_in4, n_elements=400)
    b.set_bc("fixed", "fixed")
    b.add_support(x_ft=6, bc="roller")
    b.set_support_settlement(x_ft=6, delta_in=+0.06)   # +5 mm upward
    b.add_udl(w_kip_per_ft=25, x_start_ft=0, x_end_ft=6)

    r = b.solve()

    M_left  = r.moment_reactions.get(0.0,  r.moment_reactions.get(0, None))
    M_right = r.moment_reactions.get(10.0, r.moment_reactions.get(10, None))

    abs_left  = abs(M_left)  if M_left  is not None else None
    abs_right = abs(M_right) if M_right is not None else None

    print("=" * 55)
    print("Hibbeler Ch 15 — Problem 15-2 Benchmark")
    print("=" * 55)
    print(f"  Moment reactions dict: {r.moment_reactions}")
    print(f"  |M_left|  solver = {abs_left:.4f}  kN·m   textbook = 27.5")
    print(f"  |M_right| solver = {abs_right:.4f}  kN·m   textbook = 116.25")

    pass_left  = abs_left  is not None and abs(abs_left  -  27.5)   < TOL
    pass_right = abs_right is not None and abs(abs_right - 116.25)  < TOL

    print()
    print(f"  M_left  {'PASS' if pass_left  else 'FAIL'}")
    print(f"  M_right {'PASS' if pass_right else 'FAIL'}")
    print()
    if pass_left and pass_right:
        print("  RESULT: PASS — solver matches textbook")
    else:
        print("  RESULT: FAIL — discrepancy exceeds tolerance")

    print()
    print(f"  Vertical reactions: {r.reactions}")
    return pass_left and pass_right

if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)
