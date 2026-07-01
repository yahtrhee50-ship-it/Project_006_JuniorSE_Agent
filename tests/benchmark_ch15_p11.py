"""
Hibbeler Structural Analysis Ch 15 — Benchmark Problem 15-11

Geometry (metric, consistent units treated as ft/kip/kip-ft in solver):
  N1 (x=0)  fixed
  N2 (x=4)  smooth vertical slider: v free, theta=0

Load: 30 kN/m UDL on full span (4 m).

Smooth vertical slider at N2:
  - Vertical displacement is FREE (no vertical reaction at N2)
  - Rotation is constrained to ZERO

This BC requires "guided" support type, newly added to GeneralBeam.

Textbook answers (Hibbeler §15):
  R2 = 80  kN·m  (moment at slider N2, x=4m)
  R3 = 120 kN    (vertical reaction at fixed N1, x=0)
  R4 = 160 kN·m  (moment at fixed N1, x=0)

Verification checks:
  - |M_slider|       (x=4m) ≈  80 kN·m
  - R_fixed_vertical (x=0m) ≈ 120 kN
  - |M_fixed|        (x=0m) ≈ 160 kN·m
  - No vertical reaction at N2 (slider, v is free)
  - Statics: ΣFy = 120 kN  (= w × L = 30 × 4)
"""

import sys
sys.path.insert(0, ".")

from src.calcs.beam_stiffness import GeneralBeam

L   = 4.0    # m (treated as ft by solver)
W   = 30.0   # kN/m (treated as kip/ft by solver)

EXP_M_SLIDER  =  80.0   # kN·m  moment at guided end (x=4m)
EXP_R_FIXED   = 120.0   # kN    vertical reaction at fixed end (x=0m)
EXP_M_FIXED   = 160.0   # kN·m  moment at fixed end (x=0m)
EXP_TOTAL     = W * L   # 120 kN (total applied load)

TOL_F = 0.1   # kN
TOL_M = 0.1   # kN·m


def run():
    b = GeneralBeam(total_length_ft=L, E_ksi=1, I_in4=1, n_elements=100)
    b.set_bc("fixed", "guided")
    b.add_udl(w_kip_per_ft=W)

    r = b.solve()

    R_fixed   = r.reactions.get(0.0,  r.reactions.get(0,  None))
    R_slider  = r.reactions.get(L,    r.reactions.get(int(L), None))
    M_fixed   = r.moment_reactions.get(0.0,  r.moment_reactions.get(0,  None))
    M_slider  = r.moment_reactions.get(L,    r.moment_reactions.get(int(L), None))

    abs_M_fixed  = abs(M_fixed)  if M_fixed  is not None else None
    abs_M_slider = abs(M_slider) if M_slider is not None else None

    print("=" * 65)
    print("Hibbeler Ch 15 — Problem 15-11 Benchmark")
    print("Fixed (x=0) — guided slider (x=4m) — UDL 30 kN/m")
    print("=" * 65)
    print(f"  reactions dict       : {r.reactions}")
    print(f"  moment_reactions dict: {r.moment_reactions}")
    print()
    print(f"  R_fixed   solver = {R_fixed:+.4f} kN    expect +{EXP_R_FIXED:.1f}")
    print(f"  R_slider  solver = {R_slider}  expect None (v is free at slider)")
    print(f"  |M_fixed| solver = {abs_M_fixed:.4f} kN·m  expect {EXP_M_FIXED:.1f}")
    print(f"  |M_slide| solver = {abs_M_slider:.4f} kN·m  expect {EXP_M_SLIDER:.1f}")
    print()

    statics_sum = (R_fixed or 0) + (R_slider or 0)
    net = statics_sum - EXP_TOTAL
    print(f"  Statics SumFy = {statics_sum:.4f} kN  (expect {EXP_TOTAL:.1f})  residual = {net:.6f}")
    print()

    pass_R_fixed   = R_fixed  is not None and abs(R_fixed  - EXP_R_FIXED)  < TOL_F
    pass_R_slider  = R_slider is None                   # no vertical reaction at slider
    pass_M_fixed   = abs_M_fixed  is not None and abs(abs_M_fixed  - EXP_M_FIXED)  < TOL_M
    pass_M_slider  = abs_M_slider is not None and abs(abs_M_slider - EXP_M_SLIDER) < TOL_M
    pass_statics   = abs(net) < 0.01

    print(f"  R_fixed_vert {'PASS' if pass_R_fixed   else 'FAIL'}  (expect +{EXP_R_FIXED:.0f} kN)")
    print(f"  R_slider_nil {'PASS' if pass_R_slider  else 'FAIL'}  (expect None — v is free)")
    print(f"  M_fixed      {'PASS' if pass_M_fixed   else 'FAIL'}  (expect {EXP_M_FIXED:.0f} kN·m)")
    print(f"  M_slider     {'PASS' if pass_M_slider  else 'FAIL'}  (expect {EXP_M_SLIDER:.0f} kN·m)")
    print(f"  Statics      {'PASS' if pass_statics   else 'FAIL'}  (SumFy = {EXP_TOTAL:.0f} kN)")
    print()

    all_pass = (pass_R_fixed and pass_R_slider and pass_M_fixed
                and pass_M_slider and pass_statics)
    if all_pass:
        print("  RESULT: PASS — solver matches Hibbeler textbook answers")
    else:
        print("  RESULT: FAIL — discrepancy exceeds tolerance")

    return all_pass


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)
