"""
AISC 360-22 §F2/F3 flexure and §G2 shear capacity for W-shapes (strong axis bending).
All inputs/outputs in imperial units: kips, in, ft, ksi.

Scope (MVP):
  - Compact and non-compact flanges (§F2, §F3)
  - Compact webs only — §F4/F5 (non-compact/slender web) raise NotImplementedError
  - Shear: §G2.1(a) compact web (φv=1.0) and §G2.1(b) non-compact/slender web
  - W-shapes only; caller is responsible for passing the correct section type
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from .sections import get_section

_E     = 29_000.0  # ksi — AISC assumed modulus (C-B3.1)
_PHI_B = 0.90      # LRFD flexure resistance factor
_PHI_V = 1.00      # LRFD shear — compact web §G2.1(a)
_PHI_V_NC = 0.90   # LRFD shear — non-compact/slender web §G2.1(b)


# ---------------------------------------------------------------------------
# Result dataclasses
# ---------------------------------------------------------------------------

@dataclass
class FlexureResult:
    phi_Mn_kip_ft: float
    Mp_kip_ft:     float
    limit_state:   str    # yielding | inelastic_LTB | elastic_LTB | FLB_noncompact | FLB_slender
    Lb_ft:         float
    Lp_ft:         float
    Lr_ft:         float
    compact_flange: bool
    compact_web:   bool
    DCR:           float
    code_ref:      str


@dataclass
class ShearResult:
    phi_Vn_kips: float
    Cv1:         float
    limit_state: str    # compact_web | noncompact_web
    DCR:         float
    code_ref:    str


@dataclass
class BeamCheck:
    section:   str
    Fy_ksi:    float
    Mu_kip_ft: float
    Vu_kips:   float
    flexure:   FlexureResult
    shear:     ShearResult

    def summary(self) -> str:
        f, v = self.flexure, self.shear
        tag_f = "OK" if f.DCR <= 1.0 else "*** OVERSTRESSED ***"
        tag_v = "OK" if v.DCR <= 1.0 else "*** OVERSTRESSED ***"
        return "\n".join([
            f"AISC 360-22 Beam Check — {self.section}  (Fy = {self.Fy_ksi:.0f} ksi)",
            "-" * 56,
            "FLEXURE (Chapter F):",
            f"  Limit state    : {f.limit_state}",
            f"  phiMn          = {f.phi_Mn_kip_ft:,.1f} kip-ft",
            f"  Mu             = {self.Mu_kip_ft:,.1f} kip-ft",
            f"  DCR            = {f.DCR:.3f}  {tag_f}",
            f"  Lp / Lb / Lr   = {f.Lp_ft:.2f} / {f.Lb_ft:.2f} / {f.Lr_ft:.2f} ft",
            f"  Compact flange : {'Yes' if f.compact_flange else 'No'}"
            f"  |  Compact web : {'Yes' if f.compact_web else 'No'}",
            f"  Ref            : {f.code_ref}",
            "",
            "SHEAR (Chapter G):",
            f"  Limit state    : {v.limit_state}",
            f"  phiVn          = {v.phi_Vn_kips:,.1f} kips",
            f"  Vu             = {self.Vu_kips:,.1f} kips",
            f"  DCR            = {v.DCR:.3f}  {tag_v}",
            f"  Cv1            = {v.Cv1:.4f}",
            f"  Ref            : {v.code_ref}",
        ])


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------

def _get_kdes(sec: dict) -> float:
    raw = sec.get("kdes")
    if isinstance(raw, (int, float)):
        return float(raw)
    return float(sec["tf"]) + 0.5   # fallback (should not occur for W-shapes)


def _check_flexure(
    sec: dict, Mu_kip_ft: float, Fy: float, Lb_ft: float, Cb: float
) -> FlexureResult:
    E    = _E
    Zx   = float(sec["Zx"])
    Sx   = float(sec["Sx"])
    ry   = float(sec["ry"])
    Iy   = float(sec["Iy"])
    Cw   = float(sec["Cw"])
    J    = float(sec["J"])
    d    = float(sec["d"])
    bf   = float(sec["bf"])
    tf   = float(sec["tf"])
    tw   = float(sec["tw"])
    kdes = _get_kdes(sec)

    # ---- Compactness limits (Table B4.1b) ----
    h      = d - 2.0 * kdes               # clear web height, in
    lam_f  = bf / (2.0 * tf)              # flange slenderness
    lam_w  = h / tw                       # web slenderness
    lam_pf = 0.38 * math.sqrt(E / Fy)    # compact flange limit
    lam_rf = 1.00 * math.sqrt(E / Fy)    # non-compact flange limit
    lam_pw = 3.76 * math.sqrt(E / Fy)    # compact web limit
    lam_rw = 5.70 * math.sqrt(E / Fy)    # non-compact web limit

    compact_flange = lam_f <= lam_pf
    compact_web    = lam_w <= lam_pw

    if lam_w > lam_pw:
        tag = "slender" if lam_w > lam_rw else "non-compact"
        raise NotImplementedError(
            f"{tag} web (h/tw={lam_w:.1f}, compact limit={lam_pw:.1f}): "
            "§F4/F5 not in MVP scope — escalate to senior engineer."
        )

    # ---- Compute Lp and Lr (always needed for reporting) ----
    Mp  = Fy * Zx                                      # kip-in
    Lp  = 1.76 * ry * math.sqrt(E / Fy)               # in, §F2.5
    ho  = d - tf                                        # dist. between flange centroids, in
    rts = math.sqrt(math.sqrt(Iy * Cw) / Sx)           # in, §F2.7
    _a  = J / (Sx * ho)
    Lr  = (1.95 * rts * (E / (0.7 * Fy))
           * math.sqrt(_a + math.sqrt(_a**2 + 6.76 * (0.7 * Fy / E)**2)))  # in, §F2.6

    # ---- Non-compact or slender flange → §F3 (overrides LTB) ----
    if not compact_flange:
        if lam_f > lam_rf:                             # slender flange §F3.2(b)
            kc  = max(0.35, min(0.76, 4.0 / math.sqrt(h / tw)))
            Mn  = 0.9 * E * kc * Sx / lam_f**2
            ls  = "FLB_slender"
            ref = "AISC 360-22 Sec.F3.2(b)"
        else:                                           # non-compact flange §F3.2(a)
            Mn  = Mp - (Mp - 0.7 * Fy * Sx) * (lam_f - lam_pf) / (lam_rf - lam_pf)
            ls  = "FLB_noncompact"
            ref = "AISC 360-22 Sec.F3.2(a)"
        Mn     = min(Mn, Mp)
        phi_Mn = _PHI_B * Mn / 12.0                    # kip-ft
        return FlexureResult(
            phi_Mn_kip_ft=phi_Mn, Mp_kip_ft=Mp / 12.0,
            limit_state=ls, Lb_ft=Lb_ft,
            Lp_ft=Lp / 12.0, Lr_ft=Lr / 12.0,
            compact_flange=False, compact_web=compact_web,
            DCR=Mu_kip_ft / phi_Mn, code_ref=ref,
        )

    # ---- Compact section: §F2 with LTB ----
    Lb = Lb_ft * 12.0   # in

    if Lb <= Lp:                                        # §F2.1 — yielding
        Mn  = Mp
        ls  = "yielding"
        ref = "AISC 360-22 Sec.F2.1"
    elif Lb <= Lr:                                      # §F2.2 — inelastic LTB
        Mn  = Cb * (Mp - (Mp - 0.7 * Fy * Sx) * (Lb - Lp) / (Lr - Lp))
        Mn  = min(Mn, Mp)
        ls  = "inelastic_LTB"
        ref = "AISC 360-22 Sec.F2.2"
    else:                                               # §F2.3 — elastic LTB
        Fcr = ((Cb * math.pi**2 * E / (Lb / rts)**2)
               * math.sqrt(1.0 + 0.078 * J / (Sx * ho) * (Lb / rts)**2))
        Mn  = min(Fcr * Sx, Mp)
        ls  = "elastic_LTB"
        ref = "AISC 360-22 Sec.F2.3"

    phi_Mn = _PHI_B * Mn / 12.0   # kip-ft
    return FlexureResult(
        phi_Mn_kip_ft=phi_Mn, Mp_kip_ft=Mp / 12.0,
        limit_state=ls, Lb_ft=Lb_ft,
        Lp_ft=Lp / 12.0, Lr_ft=Lr / 12.0,
        compact_flange=True, compact_web=True,
        DCR=Mu_kip_ft / phi_Mn, code_ref=ref,
    )


def _check_shear(sec: dict, Vu_kips: float, Fy: float) -> ShearResult:
    E    = _E
    d    = float(sec["d"])
    tw   = float(sec["tw"])
    kdes = _get_kdes(sec)
    h    = d - 2.0 * kdes
    Aw   = d * tw
    h_tw = h / tw

    # §G2.1(a): h/tw ≤ 2.24√(E/Fy) → φv=1.0, Cv1=1.0
    if h_tw <= 2.24 * math.sqrt(E / Fy):
        Cv1   = 1.0
        phi_v = _PHI_V
        ls    = "compact_web"
        ref   = "AISC 360-22 Sec.G2.1(a)"
    else:
        # §G2.1(b): kv = 5.34 (no transverse stiffeners, a/h > 3)
        kv    = 5.34
        lam_p = 1.10 * math.sqrt(kv * E / Fy)
        lam_r = 1.37 * math.sqrt(kv * E / Fy)
        if h_tw <= lam_p:
            Cv1 = 1.0
        elif h_tw <= lam_r:
            Cv1 = lam_p / h_tw
        else:
            Cv1 = 1.51 * kv * E / (h_tw**2 * Fy)
        phi_v = _PHI_V_NC
        ls    = "noncompact_web"
        ref   = "AISC 360-22 Sec.G2.1(b)"

    phi_Vn = phi_v * 0.6 * Fy * Aw * Cv1
    return ShearResult(
        phi_Vn_kips=phi_Vn, Cv1=Cv1,
        limit_state=ls, DCR=Vu_kips / phi_Vn, code_ref=ref,
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def check_beam(
    section_label: str,
    Mu_kip_ft: float,
    Vu_kips: float,
    Lb_ft: float = 0.0,
    Fy_ksi: float = 50.0,
    Cb: float = 1.0,
    historical: bool = False,
) -> BeamCheck:
    """
    AISC 360-22 §F2/F3 flexure and §G2 shear check for a W-shape.

    Parameters
    ----------
    section_label  AISC designation, e.g. "W16X40"
    Mu_kip_ft      Factored flexural demand (LRFD, kip-ft)
    Vu_kips        Factored shear demand (LRFD, kips)
    Lb_ft          Unbraced length (ft); 0.0 treated as fully braced (Lb ≤ Lp)
    Fy_ksi         Yield stress (ksi); default 50 for A992
    Cb             Moment gradient factor; default 1.0 (conservative)
    historical     Look up section in historical database
    """
    sec = get_section(section_label, historical=historical)

    required = ("Zx", "Sx", "ry", "Iy", "Cw", "J", "d", "bf", "tf", "tw")
    missing  = [f for f in required if sec.get(f) in (None, "", "–")]
    if missing:
        raise ValueError(
            f"{section_label!r} is missing required fields {missing}. "
            "check_beam() supports W-shapes only."
        )

    flexure = _check_flexure(sec, Mu_kip_ft, Fy_ksi, Lb_ft, Cb)
    shear   = _check_shear(sec, Vu_kips, Fy_ksi)

    return BeamCheck(
        section=section_label.upper(),
        Fy_ksi=Fy_ksi,
        Mu_kip_ft=Mu_kip_ft,
        Vu_kips=Vu_kips,
        flexure=flexure,
        shear=shear,
    )
