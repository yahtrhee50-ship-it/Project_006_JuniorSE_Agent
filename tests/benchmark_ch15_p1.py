"""
Hibbeler Structural Analysis Ch 15 — Benchmark Problem 15-1

Geometry (metric, consistent units treated as ft/kip/kip-ft in solver):
  N1 (x=0)  fixed
  N2 (x=6)  roller
  N3 (x=10) fixed

Load: 25 kN/m UDL on M1 (x=0 to x=6)

Expected (textbook):
  M1 = 90  kN·m (moment reaction at left fixed support)
  M3 = 22.5 kN·m (moment reaction at right fixed support)
  Both are clockwise reactions (sagging convention from solver may differ — check magnitude)
"""

import sys
sys.path.insert(0, ".")

from src.calcs.beam_stiffness import GeneralBeam

TOL = 0.5   # kN·m tolerance against textbook answer

def run():
    # EI=1 dummy (answers scale with EI; only ratios matter for reactions)
    b = GeneralBeam(total_length_ft=10, E_ksi=1, I_in4=1, n_elements=400)
    b.set_bc("fixed", "fixed")
    b.add_support(x_ft=6, bc="roller")
    b.add_udl(w_kip_per_ft=25, x_start_ft=0, x_end_ft=6)

    r = b.solve()

    # Moment reactions at fixed supports
    M_left  = r.moment_reactions.get(0.0,  r.moment_reactions.get(0, None))
    M_right = r.moment_reactions.get(10.0, r.moment_reactions.get(10, None))

    # GeneralBeam sign convention: positive = CCW at support.
    # Textbook quotes magnitude; compare absolute values.
    abs_left  = abs(M_left)  if M_left  is not None else None
    abs_right = abs(M_right) if M_right is not None else None

    print("=" * 55)
    print("Hibbeler Ch 15 — Problem 15-1 Benchmark")
    print("=" * 55)
    print(f"  Moment reactions dict: {r.moment_reactions}")
    print(f"  |M_left|  solver = {abs_left:.4f}  kN·m   textbook = 90.0")
    print(f"  |M_right| solver = {abs_right:.4f}  kN·m   textbook = 22.5")

    pass_left  = abs_left  is not None and abs(abs_left  -  90.0) < TOL
    pass_right = abs_right is not None and abs(abs_right -  22.5) < TOL

    print()
    print(f"  M_left  {'PASS' if pass_left  else 'FAIL'}")
    print(f"  M_right {'PASS' if pass_right else 'FAIL'}")
    print()
    if pass_left and pass_right:
        print("  RESULT: PASS — solver matches textbook")
    else:
        print("  RESULT: FAIL — discrepancy exceeds tolerance")

    # Also print vertical reactions for reference
    print()
    print(f"  Vertical reactions: {r.reactions}")
    return pass_left and pass_right

if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)
