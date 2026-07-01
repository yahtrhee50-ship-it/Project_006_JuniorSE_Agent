"""
Hibbeler Structural Analysis Ch 15 — Benchmark Problem 15-9

Geometry (metric, kN and m):
  N1 (x= 0)  roller (left end — simple support)
  N2 (x=12)  roller (intermediate support)
  N3 (x=24)  roller (intermediate support)
  N4 (x=36)  pin   (right end — simple support)

Spans: M1=M2=M3 = 12 m each, constant EI.
Load:  UDL 4 kN/m over all three spans.
Total applied = 4 × 36 = 144 kN downward.

Standard result for 3-equal-span beam under full UDL:
  R₁ = R₄ = 0.4 wL = 19.2 kN
  R₂ = R₃ = 1.1 wL = 52.8 kN
  M at N2, N3 = wL²/10 = 57.6 kN·m  (hogging)

Reference file textbook answers (Hibbeler):
  M₁ = M₄ = 0          ✓ (no moment fixity at roller/pin ends)
  M₂ = M₃ = 44.2 kN·m  ← does NOT match 3-moment equation (57.6 kN·m).
                           Flagged as likely reference-file extraction error.

Since M₁=M₄=0 and M₂=M₃ is confirmed by symmetry, the test validates:
  - Statics (ΣFy = 144 kN)
  - Symmetry (R₁=R₄, R₂=R₃)
  - Interior support bending moment magnitude (solver vs. 57.6 kN·m)
  - Moment at ends = 0
"""

import sys
sys.path.insert(0, ".")

from src.calcs.beam_stiffness import GeneralBeam

# n_elements=900: each 12 m span gets 300 elements → interior nodes land exactly at x=12 and x=24
N_EL = 900
L = 36.0
W = 4.0      # kN/m UDL

# Expected values from 3-moment equation / standard coefficients
EXP_R_END    = 0.4 * W * 12     # 19.2 kN
EXP_R_INT    = 1.1 * W * 12     # 52.8 kN
EXP_M_INT    = W * 12**2 / 10   # 57.6 kN·m  (magnitude, hogging)
EXP_TOTAL    = W * L            # 144 kN

TOL_F = 0.20   # kN  — generous for FEM approximation
TOL_M = 0.30   # kN·m


def run():
    b = GeneralBeam(total_length_ft=L, E_ksi=1, I_in4=1, n_elements=N_EL)
    b.set_bc("pin", "pin")              # roller at N1, pin at N4 (both v=0, θ free)
    b.add_support(x_ft=12.0, bc="pin") # roller at N2
    b.add_support(x_ft=24.0, bc="pin") # roller at N3
    b.add_udl(w_kip_per_ft=W, x_start_ft=0, x_end_ft=L)

    r = b.solve()

    R_N1 = r.reactions.get(0.0,  r.reactions.get(0,  None))
    R_N2 = r.reactions.get(12.0, r.reactions.get(12, None))
    R_N3 = r.reactions.get(24.0, r.reactions.get(24, None))
    R_N4 = r.reactions.get(36.0, r.reactions.get(36, None))

    # Bending moment at interior support locations (from the V/M diagram arrays)
    xs  = r.x_ft
    Ms  = r.M_kip_ft

    def m_at(x_target):
        diffs = [abs(x - x_target) for x in xs]
        idx   = diffs.index(min(diffs))
        return Ms[idx]

    M_at_N2 = m_at(12.0)
    M_at_N3 = m_at(24.0)

    # Moment reactions at ends (should be 0 for pin/roller)
    M_N1 = r.moment_reactions.get(0.0,  r.moment_reactions.get(0,  0))
    M_N4 = r.moment_reactions.get(36.0, r.moment_reactions.get(36, 0))

    print("=" * 65)
    print("Hibbeler Ch 15 — Problem 15-9 Benchmark")
    print("3-span continuous beam  |  4 kN/m UDL  |  12 m each span")
    print("=" * 65)
    print(f"  reactions dict       : {r.reactions}")
    print(f"  moment_reactions dict: {r.moment_reactions}")
    print()
    print(f"  R_N1 solver = {R_N1:+.4f} kN   expect {EXP_R_END:.2f}  (0.4wL)")
    print(f"  R_N2 solver = {R_N2:+.4f} kN   expect {EXP_R_INT:.2f}  (1.1wL)")
    print(f"  R_N3 solver = {R_N3:+.4f} kN   expect {EXP_R_INT:.2f}  (1.1wL)")
    print(f"  R_N4 solver = {R_N4:+.4f} kN   expect {EXP_R_END:.2f}  (0.4wL)")
    print()
    print(f"  M at N2 (x=12) solver = {M_at_N2:+.4f} kN·m   expect ~-{EXP_M_INT:.1f} hogging")
    print(f"  M at N3 (x=24) solver = {M_at_N3:+.4f} kN·m   expect ~-{EXP_M_INT:.1f} hogging")
    print()
    print(f"  M_rxn at N1 = {M_N1:.4f}   M_rxn at N4 = {M_N4:.4f}  (expect 0)")
    print()

    statics_sum = (R_N1 or 0) + (R_N2 or 0) + (R_N3 or 0) + (R_N4 or 0)
    net = statics_sum - EXP_TOTAL
    print(f"  Statics SumFy = {statics_sum:.4f} kN  (expect {EXP_TOTAL:.1f})  residual = {net:.6f}")
    print()

    # --- PASS/FAIL checks ---
    pass_R_N1    = R_N1 is not None and abs(R_N1 - EXP_R_END) < TOL_F
    pass_R_N2    = R_N2 is not None and abs(R_N2 - EXP_R_INT) < TOL_F
    pass_R_N3    = R_N3 is not None and abs(R_N3 - EXP_R_INT) < TOL_F
    pass_R_N4    = R_N4 is not None and abs(R_N4 - EXP_R_END) < TOL_F
    pass_symm    = (R_N1 is not None and R_N4 is not None and abs(R_N1 - R_N4) < TOL_F and
                    R_N2 is not None and R_N3 is not None and abs(R_N2 - R_N3) < TOL_F)
    pass_M_N2    = abs(abs(M_at_N2) - EXP_M_INT) < TOL_M
    pass_M_N3    = abs(abs(M_at_N3) - EXP_M_INT) < TOL_M
    pass_M_ends  = abs(M_N1) < TOL_M and abs(M_N4) < TOL_M
    pass_statics = abs(net) < 0.01

    print(f"  R_N1   {'PASS' if pass_R_N1    else 'FAIL'}  (0.4wL = {EXP_R_END:.1f})")
    print(f"  R_N2   {'PASS' if pass_R_N2    else 'FAIL'}  (1.1wL = {EXP_R_INT:.1f})")
    print(f"  R_N3   {'PASS' if pass_R_N3    else 'FAIL'}  (1.1wL = {EXP_R_INT:.1f})")
    print(f"  R_N4   {'PASS' if pass_R_N4    else 'FAIL'}  (0.4wL = {EXP_R_END:.1f})")
    print(f"  Symm   {'PASS' if pass_symm    else 'FAIL'}  (R1=R4, R2=R3)")
    print(f"  M@N2   {'PASS' if pass_M_N2    else 'FAIL'}  (|M|={EXP_M_INT:.1f})")
    print(f"  M@N3   {'PASS' if pass_M_N3    else 'FAIL'}  (|M|={EXP_M_INT:.1f})")
    print(f"  M_ends {'PASS' if pass_M_ends  else 'FAIL'}  (M1=M4=0)")
    print(f"  Stat   {'PASS' if pass_statics else 'FAIL'}  (SumFy=0)")
    print()

    print("  NOTE: Reference file gives textbook M2=M3=44.2 kN.m.")
    print(f"        3-moment equation gives  M2=M3=wL^2/10 = {EXP_M_INT:.1f} kN.m.")
    print("        Likely reference-file extraction error -- flagged for engineer review.")
    print()

    all_pass = (pass_R_N1 and pass_R_N2 and pass_R_N3 and pass_R_N4 and
                pass_symm and pass_M_N2 and pass_M_N3 and pass_M_ends and pass_statics)
    if all_pass:
        print("  RESULT: PASS -- solver matches 3-moment equation solution")
    else:
        print("  RESULT: FAIL -- discrepancy exceeds tolerance")

    return all_pass


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)
