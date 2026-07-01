"""
Hibbeler Structural Analysis Ch 15 — Benchmark Problem 15-10

Geometry (US customary, kips and ft):
  N0 (x= 0)  free overhang end (left)
  N1 (x= 4)  roller
  N2 (x=12)  pin
  N3 (x=20)  roller
  N4 (x=24)  free overhang end (right)

Spans: 4 ft overhang | 8 ft interior span 1 | 8 ft interior span 2 | 4 ft overhang
Load:  3 k/ft UDL over entire 24 ft beam.
Total applied = 3 × 24 = 72 k downward.

Textbook approach (Hibbeler p. 532–533):
  - Only M1 and M2 (interior spans) assemble into the global stiffness matrix.
  - Overhangs contribute equivalent nodal loads at N1 and N3:
      vertical: 3×4 = 12 k downward at each overhang support
      moment  : 3×4²/2 = 24 k·ft (transferred as fixed-end moment at N1 and N3)
  - 3 rotational DOFs (D1, D2, D3 at N1, N2, N3); all vertical DOFs constrained to 0.

Textbook answers (Hibbeler):
  R₁ = 25.5 k ↑  (at N1, x = 4 ft)
  R₂ = 21.0 k ↑  (at N2, x = 12 ft)
  R₃ = 25.5 k ↑  (at N3, x = 20 ft)

Verification checks:
  - Statics: ΣFy = 72.0 k (= 3 × 24)
  - Symmetry: R_N1 = R_N3
  - Each reaction vs. textbook value
"""

import sys
sys.path.insert(0, ".")

from src.calcs.beam_stiffness import GeneralBeam

# n_elements=600: element length = 24/600 = 0.04 ft
# 4 ft / 0.04 = 100  → N1 at x=4  lands on node 100  ✓
# 8 ft / 0.04 = 200  → N2 at x=12 lands on node 300  ✓
# 8 ft / 0.04 = 200  → N3 at x=20 lands on node 500  ✓
N_EL = 600
L    = 24.0   # ft
W    = 3.0    # k/ft UDL

# Textbook expected reactions
EXP_R1    = 25.5   # k  (roller at N1)
EXP_R2    = 21.0   # k  (pin at N2)
EXP_R3    = 25.5   # k  (roller at N3)
EXP_TOTAL = W * L  # 72 k

TOL_F = 0.10   # k  — reaction tolerance


def run():
    b = GeneralBeam(total_length_ft=L, E_ksi=1, I_in4=1, n_elements=N_EL)
    b.set_bc("free", "free")              # free overhang ends at N0 and N4
    b.add_support(x_ft=4.0,  bc="pin")   # roller at N1
    b.add_support(x_ft=12.0, bc="pin")   # pin at N2
    b.add_support(x_ft=20.0, bc="pin")   # roller at N3
    b.add_udl(w_kip_per_ft=W, x_start_ft=0, x_end_ft=L)

    r = b.solve()

    R_N1 = r.reactions.get(4.0,  r.reactions.get(4,  None))
    R_N2 = r.reactions.get(12.0, r.reactions.get(12, None))
    R_N3 = r.reactions.get(20.0, r.reactions.get(20, None))

    print("=" * 65)
    print("Hibbeler Ch 15 — Problem 15-10 Benchmark")
    print("Symmetric overhangs  |  3 k/ft UDL  |  4-8-8-4 ft layout")
    print("=" * 65)
    print(f"  reactions dict       : {r.reactions}")
    print(f"  moment_reactions dict: {r.moment_reactions}")
    print()
    print(f"  R_N1 solver = {R_N1:+.4f} k   expect {EXP_R1:.1f}  (roller at x=4)")
    print(f"  R_N2 solver = {R_N2:+.4f} k   expect {EXP_R2:.1f}  (pin   at x=12)")
    print(f"  R_N3 solver = {R_N3:+.4f} k   expect {EXP_R3:.1f}  (roller at x=20)")
    print()

    statics_sum = (R_N1 or 0) + (R_N2 or 0) + (R_N3 or 0)
    net = statics_sum - EXP_TOTAL
    print(f"  Statics SumFy = {statics_sum:.4f} k  (expect {EXP_TOTAL:.1f})  residual = {net:.6f}")
    print()

    # --- PASS/FAIL checks ---
    pass_R1      = R_N1 is not None and abs(R_N1 - EXP_R1) < TOL_F
    pass_R2      = R_N2 is not None and abs(R_N2 - EXP_R2) < TOL_F
    pass_R3      = R_N3 is not None and abs(R_N3 - EXP_R3) < TOL_F
    pass_symm    = (R_N1 is not None and R_N3 is not None
                    and abs(R_N1 - R_N3) < TOL_F)
    pass_statics = abs(net) < 0.01

    print(f"  R_N1   {'PASS' if pass_R1      else 'FAIL'}  (expect {EXP_R1})")
    print(f"  R_N2   {'PASS' if pass_R2      else 'FAIL'}  (expect {EXP_R2})")
    print(f"  R_N3   {'PASS' if pass_R3      else 'FAIL'}  (expect {EXP_R3})")
    print(f"  Symm   {'PASS' if pass_symm    else 'FAIL'}  (R_N1 = R_N3)")
    print(f"  Stat   {'PASS' if pass_statics else 'FAIL'}  (SumFy = {EXP_TOTAL:.0f} k)")
    print()

    all_pass = pass_R1 and pass_R2 and pass_R3 and pass_symm and pass_statics
    if all_pass:
        print("  RESULT: PASS -- solver matches Hibbeler textbook answers")
    else:
        print("  RESULT: FAIL -- discrepancy exceeds tolerance")

    return all_pass


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)
