"""
ACI 318-19 rectangular singly-reinforced concrete beam design/check.
Units: kips, inches, ksi throughout (inputs and outputs).
f'c and fy are accepted in ksi; internally converted to psi where ACI formulas require psi.

Phase 1 scope:
  - Flexure §22.3 (φ = 0.90 tension-controlled)
  - Shear §22.5 — Table 22.5.5.1 detailed method + stirrups §22.5.10.5.3
  - Deflection §24.2 — Branson effective Ie, immediate + long-term (5-yr λΔ)
  - Development §25.4.2 (straight), §25.4.3 (hook), §25.4.4 (T-head / headed bar)
  - Splices §25.5.2 (Class A/B tension)
  - Curtailment §9.7.3 (simply supported, uniform load only)

Deferred: doubly-reinforced, T-beams, two-way slabs, deep beams, seismic provisions,
          compression development, redistribution.
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field

from . import rebar as _rb

_PHI_FLEX  = 0.90     # LRFD flexure §21.2.1(a), tension-controlled
_PHI_SHEAR = 0.75     # LRFD shear  §21.2.1(d)
_ES_KSI    = 29_000.0 # modulus of elasticity of steel, ksi
_LAMBDA_NW = 1.0      # normal-weight concrete factor λ


# ---------------------------------------------------------------------------
# Material helpers
# ---------------------------------------------------------------------------

def beta1(fc_ksi: float) -> float:
    """Stress block factor β₁ per ACI 318-19 §22.2.2.4.3."""
    fc_psi = fc_ksi * 1000.0
    return max(0.65, 0.85 - 0.05 * (fc_psi - 4000.0) / 1000.0)


def Ec_ksi(fc_ksi: float, wc_pcf: float = 145.0) -> float:
    """Concrete elastic modulus per §19.2.2.1a: Ec = 33·wc^1.5·√f'c (psi → ksi)."""
    fc_psi = fc_ksi * 1000.0
    return 33.0 * wc_pcf**1.5 * math.sqrt(fc_psi) / 1000.0


def modular_ratio(fc_ksi: float, wc_pcf: float = 145.0) -> int:
    """Modular ratio n = Es/Ec, rounded up per §24.2.2.1."""
    return math.ceil(_ES_KSI / Ec_ksi(fc_ksi, wc_pcf))


def min_As(b_in: float, d_in: float, fc_ksi: float, fy_ksi: float) -> float:
    """Minimum longitudinal tension steel per §9.6.1.2 (in²)."""
    fc_psi = fc_ksi * 1000.0
    fy_psi = fy_ksi * 1000.0
    return max(3.0 * math.sqrt(fc_psi) / fy_psi, 200.0 / fy_psi) * b_in * d_in


# ---------------------------------------------------------------------------
# Flexure — §22.3
# ---------------------------------------------------------------------------

@dataclass
class FlexureResult:
    phi_Mn_kip_ft: float
    a_in:          float
    c_in:          float
    eps_t:         float
    tension_controlled: bool
    As_min_in2:    float
    As_provided_in2: float
    DCR:           float
    code_ref:      str
    notes: list[str] = field(default_factory=list)

    def summary_lines(self) -> list[str]:
        tag  = "OK" if self.DCR <= 1.0 else "*** OVERSTRESSED ***"
        tc   = ("tension-controlled (εt ≥ 0.005)"
                if self.tension_controlled else "*** NOT tension-controlled — verify φ ***")
        lines = [
            "FLEXURE (ACI 318-19 §22.3):",
            f"  a / c              = {self.a_in:.3f} in / {self.c_in:.3f} in",
            f"  εt                 = {self.eps_t:.5f}   [{tc}]",
            f"  φMn                = {self.phi_Mn_kip_ft:,.2f} kip-ft",
            f"  As,min             = {self.As_min_in2:.3f} in²   As,prov = {self.As_provided_in2:.3f} in²",
            f"  DCR                = {self.DCR:.3f}  {tag}",
            f"  Ref                : {self.code_ref}",
        ]
        for n in self.notes:
            lines.append(f"  NOTE: {n}")
        return lines


def flexure_check(
    b_in: float, d_in: float, As_in2: float,
    fc_ksi: float, fy_ksi: float, Mu_kip_ft: float,
) -> FlexureResult:
    """ACI 318-19 §22.3 flexural capacity — rectangular singly-reinforced section."""
    b1    = beta1(fc_ksi)
    a     = As_in2 * fy_ksi / (0.85 * fc_ksi * b_in)
    c     = a / b1
    eps_t = 0.003 * (d_in - c) / c           # net tensile strain at extreme steel layer
    tc    = eps_t >= 0.005

    notes: list[str] = []
    if eps_t < 0.004:
        notes.append(
            "Section compression-controlled (εt < 0.004). Reduce As or increase d — §21.2.2."
        )
    elif eps_t < 0.005:
        notes.append(
            "Transition zone (0.004 ≤ εt < 0.005): φ < 0.90 per §21.2.2. "
            "Result uses φ=0.90 (unconservative) — escalate to senior engineer."
        )

    Mn_kip_in   = As_in2 * fy_ksi * (d_in - a / 2.0)
    phi_Mn_kip_ft = _PHI_FLEX * Mn_kip_in / 12.0

    As_min_v = min_As(b_in, d_in, fc_ksi, fy_ksi)
    if As_in2 < As_min_v:
        notes.append(
            f"As,provided ({As_in2:.3f} in²) < As,min ({As_min_v:.3f} in²) — §9.6.1.2."
        )

    DCR = Mu_kip_ft / phi_Mn_kip_ft if phi_Mn_kip_ft > 0 else 9.99

    return FlexureResult(
        phi_Mn_kip_ft=phi_Mn_kip_ft, a_in=a, c_in=c,
        eps_t=eps_t, tension_controlled=tc,
        As_min_in2=As_min_v, As_provided_in2=As_in2,
        DCR=DCR, code_ref="ACI 318-19 §22.3 / §21.2.1",
        notes=notes,
    )


def required_As(
    b_in: float, d_in: float, fc_ksi: float, fy_ksi: float, Mu_kip_ft: float
) -> float:
    """Minimum As to achieve φMn = Mu for rectangular singly-reinforced section (in²).

    Solves: (φfy²/(1.7fc·b))·As² − φ·fy·d·As + Mu = 0.
    Returns the smaller (tension-controlled) root.
    """
    Mu_kip_in = Mu_kip_ft * 12.0
    A = _PHI_FLEX * fy_ksi**2 / (1.7 * fc_ksi * b_in)
    B = -_PHI_FLEX * fy_ksi * d_in
    C = Mu_kip_in
    disc = B**2 - 4.0 * A * C
    if disc < 0:
        raise ValueError(
            "No solution for required As — beam cross-section is too small for the demand. "
            "Increase b or d."
        )
    return (-B - math.sqrt(disc)) / (2.0 * A)


# ---------------------------------------------------------------------------
# Shear — §22.5 Table 22.5.5.1 (detailed method)
# ---------------------------------------------------------------------------

@dataclass
class ShearResult:
    phi_Vn_kips:         float
    Vc_kips:             float
    Vs_kips:             float
    rho_w:               float
    Av_min_per_s:        float   # in²/in
    Av_provided_per_s:   float   # in²/in
    has_min_Av:          bool
    s_provided_in:       float
    s_max_in:            float
    DCR:                 float
    code_ref:            str
    notes: list[str] = field(default_factory=list)

    def summary_lines(self) -> list[str]:
        tag = "OK" if self.DCR <= 1.0 else "*** OVERSTRESSED ***"
        s_tag = "" if self.s_provided_in <= self.s_max_in else "  *** EXCEEDS MAX ***"
        lines = [
            "SHEAR (ACI 318-19 §22.5 / Table 22.5.5.1):",
            f"  ρw                 = {self.rho_w:.5f}",
            f"  Vc                 = {self.Vc_kips:.2f} kips",
            f"  Vs                 = {self.Vs_kips:.2f} kips",
            f"  φVn                = {self.phi_Vn_kips:.2f} kips",
            f"  Av,min/s           = {self.Av_min_per_s:.5f} in²/in  "
            f"  Av,prov/s = {self.Av_provided_per_s:.5f} in²/in",
            f"  Stirrup spacing    = {self.s_provided_in:.2f} in  "
            f"(max allowed {self.s_max_in:.2f} in){s_tag}",
            f"  Has min Av         : {'YES' if self.has_min_Av else 'NO — size-effect λs applied to Vc'}",
            f"  DCR                = {self.DCR:.3f}  {tag}",
            f"  Ref                : {self.code_ref}",
        ]
        for n in self.notes:
            lines.append(f"  NOTE: {n}")
        return lines


def shear_check(
    b_in: float, d_in: float, As_in2: float, h_in: float,
    fc_ksi: float, fy_ksi: float,
    Av_in2: float, s_in: float, fyt_ksi: float,
    Vu_kips: float, Nu_kips: float = 0.0,
) -> ShearResult:
    """ACI 318-19 §22.5 detailed shear check — Table 22.5.5.1 + §22.5.10.5.3.

    Parameters
    ----------
    b_in, d_in   : beam width and effective depth (in)
    As_in2       : total longitudinal tension steel area (in²)
    h_in         : total beam height (in)
    fc_ksi       : concrete compressive strength (ksi)
    fy_ksi       : longitudinal rebar fy (ksi) — used for As,min reference only
    Av_in2       : total stirrup area — both legs (in²), e.g. 2×0.11 = 0.22 for 2-leg #3
    s_in         : stirrup spacing (in)
    fyt_ksi      : stirrup yield strength (ksi)
    Vu_kips      : factored shear demand (kips)
    Nu_kips      : factored axial force (+compression, −tension); 0 for pure bending
    """
    fc_psi  = fc_ksi  * 1000.0
    fyt_psi = fyt_ksi * 1000.0
    Ag_in2  = b_in * h_in
    Nu_lb   = Nu_kips * 1000.0

    rho_w = As_in2 / (b_in * d_in)

    # Minimum shear reinforcement §9.6.3.4
    Av_min_per_s = max(
        0.75 * math.sqrt(fc_psi) / fyt_psi * b_in,
        50.0 * b_in / fyt_psi,
    )
    Av_per_s   = Av_in2 / s_in
    has_min_Av = Av_per_s >= Av_min_per_s

    # Size-effect factor λs — applies only when Av < Av,min (§22.5.5.1)
    lambda_s = min(1.0, math.sqrt(2.0 / (1.0 + d_in / 10.0)))
    lam_eff  = _LAMBDA_NW * (1.0 if has_min_Av else lambda_s)

    # Vc — Table 22.5.5.1 detailed formula (psi units, result in lb)
    Vc_lb = (
        8.0 * lam_eff * max(rho_w, 1e-6)**(1.0/3.0) * math.sqrt(fc_psi)
        + Nu_lb / (6.0 * Ag_in2)
    ) * b_in * d_in
    Vc_kips = Vc_lb / 1000.0

    # Vs — §22.5.10.5.3
    Vs_kips = Av_in2 * fyt_ksi * d_in / s_in

    # Maximum Vs §22.5.10.5.3 limit (cross-section size limit)
    Vs_max_kips = 8.0 * math.sqrt(fc_psi) * b_in * d_in / 1000.0

    # Maximum stirrup spacing §9.7.6.2.2
    Vs_limit_kips = 4.0 * math.sqrt(fc_psi) * b_in * d_in / 1000.0
    s_max = min(d_in / 2.0, 24.0) if Vs_kips <= Vs_limit_kips else min(d_in / 4.0, 12.0)

    notes: list[str] = []
    if Vs_kips > Vs_max_kips:
        notes.append(
            f"Vs ({Vs_kips:.1f} k) > Vs,max ({Vs_max_kips:.1f} k) — cross-section must be "
            "enlarged per §22.5.10.5.3."
        )
    if s_in > s_max:
        notes.append(
            f"Stirrup spacing {s_in:.1f} in > §9.7.6.2.2 max {s_max:.1f} in."
        )
    if not has_min_Av:
        notes.append("Av < Av,min: size-effect factor λs applied to Vc per Table 22.5.5.1.")

    phi_Vn = _PHI_SHEAR * (Vc_kips + Vs_kips)
    DCR    = Vu_kips / phi_Vn if phi_Vn > 0 else 9.99

    return ShearResult(
        phi_Vn_kips=phi_Vn, Vc_kips=Vc_kips, Vs_kips=Vs_kips,
        rho_w=rho_w, Av_min_per_s=Av_min_per_s, Av_provided_per_s=Av_per_s,
        has_min_Av=has_min_Av, s_provided_in=s_in, s_max_in=s_max,
        DCR=DCR,
        code_ref="ACI 318-19 §22.5 Table 22.5.5.1 / §22.5.10.5.3 / §9.7.6.2.2",
        notes=notes,
    )


# ---------------------------------------------------------------------------
# Deflection — §24.2 (Branson Ie, immediate + long-term)
# ---------------------------------------------------------------------------

@dataclass
class DeflectionResult:
    Ec_ksi:            float
    fr_ksi:            float
    Mcr_kip_ft:        float
    Ie_D_in4:          float   # Ie under dead load only
    Ie_DL_in4:         float   # Ie under dead + live
    delta_i_D_in:      float   # immediate dead-load deflection
    delta_i_L_in:      float   # immediate live-load deflection (incremental)
    delta_i_DL_in:     float   # immediate total (D+L)
    lambda_delta:      float   # long-term multiplier (5-yr, singly reinforced)
    delta_lt_add_in:   float   # additional long-term: λΔ × δi_D
    delta_total_in:    float   # δi_D + δi_L + λΔ·δi_D
    span_ft:           float
    limit_L_180_in:    float
    limit_L_360_in:    float
    limit_L_480_in:    float
    code_ref:          str

    def summary_lines(self) -> list[str]:
        def flag(d: float, lim: float) -> str:
            return "OK" if d <= lim else "*** EXCEEDS LIMIT ***"
        lines = [
            "DEFLECTION (ACI 318-19 §24.2):",
            f"  Ec                 = {self.Ec_ksi:.0f} ksi",
            f"  fr                 = {self.fr_ksi * 1000:.1f} psi",
            f"  Mcr                = {self.Mcr_kip_ft:.2f} kip-ft",
            f"  Ie (DL only)       = {self.Ie_D_in4:.1f} in⁴",
            f"  Ie (D+L)           = {self.Ie_DL_in4:.1f} in⁴",
            f"  δi,D               = {self.delta_i_D_in:.4f} in",
            f"  δi,L               = {self.delta_i_L_in:.4f} in",
            f"  δi,total           = {self.delta_i_DL_in:.4f} in",
            f"  λΔ (5-yr, ρ'=0)    = {self.lambda_delta:.2f}",
            f"  δlt,additional     = {self.delta_lt_add_in:.4f} in  (λΔ × δi,D)",
            f"  δtotal             = {self.delta_total_in:.4f} in  (D+L+LT)",
            "",
            "  --- ACI 318-19 Table 24.2.2 ---",
            f"  LL floor   δi,L = {self.delta_i_L_in:.4f} in  vs L/360 = {self.limit_L_360_in:.3f} in"
            f"  [{flag(self.delta_i_L_in, self.limit_L_360_in)}]",
            f"  LT total   δtot = {self.delta_total_in:.4f} in  vs L/480 = {self.limit_L_480_in:.3f} in"
            f"  [{flag(self.delta_total_in, self.limit_L_480_in)}]",
            f"  Flat roof  δi,L = {self.delta_i_L_in:.4f} in  vs L/180 = {self.limit_L_180_in:.3f} in"
            f"  [{flag(self.delta_i_L_in, self.limit_L_180_in)}]",
            f"  Ref: {self.code_ref}",
        ]
        return lines


def _cracked_section(
    b_in: float, d_in: float, h_in: float, n: int, As_in2: float, fc_ksi: float
) -> tuple[float, float, float, float, float]:
    """Return (Ig, Icr, fr_ksi, Mcr_kip_ft, Mcr_kip_in)."""
    Ig   = b_in * h_in**3 / 12.0
    yt   = h_in / 2.0
    fc_psi = fc_ksi * 1000.0
    fr_ksi = 7.5 * _LAMBDA_NW * math.sqrt(fc_psi) / 1000.0   # §19.2.3.1

    Mcr_kip_in = fr_ksi * Ig / yt
    Mcr_kip_ft = Mcr_kip_in / 12.0

    # Cracked neutral axis depth kd:  b(kd)²/2 = nAs(d − kd)
    m  = float(n) * As_in2 / b_in
    kd = -m + math.sqrt(m**2 + 2.0 * m * d_in)
    Icr = b_in * kd**3 / 3.0 + float(n) * As_in2 * (d_in - kd)**2

    return Ig, Icr, fr_ksi, Mcr_kip_ft, Mcr_kip_in


def _branson_Ie(Ig: float, Icr: float, Mcr_in: float, Ma_in: float) -> float:
    """Branson effective moment of inertia per ACI 318-19 §24.2.3.5."""
    if Ma_in <= 0 or Ma_in < Mcr_in:
        return Ig
    ratio = Mcr_in / Ma_in    # ≤ 1.0 since Ma ≥ Mcr
    Ie = ratio**3 * Ig + (1.0 - ratio**3) * Icr
    return min(Ie, Ig)


def deflection_check(
    b_in: float, h_in: float, d_in: float, As_in2: float,
    fc_ksi: float, wD_klf: float, wL_klf: float, span_ft: float,
    support: str = "simple",
) -> DeflectionResult:
    """ACI 318-19 §24.2 deflection for a simply supported beam under UDL.

    Parameters
    ----------
    wD_klf, wL_klf : SERVICE (unfactored) distributed loads (kip/ft)
    """
    if support != "simple":
        raise NotImplementedError(
            "Deflection implemented for simply supported beams only (this phase)."
        )

    Ec   = Ec_ksi(fc_ksi)
    n    = modular_ratio(fc_ksi)
    L_in = span_ft * 12.0

    Ig, Icr, fr_ksi, Mcr_kip_ft, Mcr_kip_in = _cracked_section(
        b_in, d_in, h_in, n, As_in2, fc_ksi
    )

    wD_kin = wD_klf / 12.0
    wL_kin = wL_klf / 12.0

    Ma_D_in  = wD_kin * L_in**2 / 8.0
    Ma_DL_in = (wD_kin + wL_kin) * L_in**2 / 8.0

    Ie_D  = _branson_Ie(Ig, Icr, Mcr_kip_in, Ma_D_in)
    Ie_DL = _branson_Ie(Ig, Icr, Mcr_kip_in, Ma_DL_in)

    def _delta_ss(w_kin: float, Ie_in4: float) -> float:
        return 5.0 * w_kin * L_in**4 / (384.0 * Ec * Ie_in4)

    delta_i_D  = _delta_ss(wD_kin, Ie_D)
    delta_i_DL = _delta_ss(wD_kin + wL_kin, Ie_DL)
    delta_i_L  = delta_i_DL - delta_i_D    # incremental LL contribution

    # Long-term factor §24.2.4.1.3: ξ=2.0 for ≥5 yr; ρ'=0 (singly reinforced)
    lambda_delta  = 2.0 / (1.0 + 50.0 * 0.0)   # = 2.0
    delta_lt_add  = lambda_delta * delta_i_D
    delta_total   = delta_i_D + delta_i_L + delta_lt_add

    return DeflectionResult(
        Ec_ksi=Ec, fr_ksi=fr_ksi, Mcr_kip_ft=Mcr_kip_ft,
        Ie_D_in4=Ie_D, Ie_DL_in4=Ie_DL,
        delta_i_D_in=delta_i_D, delta_i_L_in=delta_i_L, delta_i_DL_in=delta_i_DL,
        lambda_delta=lambda_delta, delta_lt_add_in=delta_lt_add,
        delta_total_in=delta_total, span_ft=span_ft,
        limit_L_180_in=L_in / 180.0,
        limit_L_360_in=L_in / 360.0,
        limit_L_480_in=L_in / 480.0,
        code_ref="ACI 318-19 §24.2 (Branson Ie §24.2.3.5, λΔ §24.2.4, limits Table 24.2.2)",
    )


# ---------------------------------------------------------------------------
# Development lengths — §25.4
# ---------------------------------------------------------------------------

@dataclass
class DevLengthResult:
    end_type:       str    # straight | hook_90 | hook_180 | thead
    ld_in:          float
    db_in:          float
    fy_ksi:         float
    fc_ksi:         float
    psi_t:          float
    psi_e:          float
    psi_s:          float
    psi_g:          float
    cb_Ktr_over_db: float  # confinement term (straight only)
    notes: list[str] = field(default_factory=list)
    code_ref: str = ""

    def summary_lines(self) -> list[str]:
        lines = [
            f"DEVELOPMENT LENGTH ({self.end_type.upper()}) (ACI 318-19):",
            f"  ld                 = {self.ld_in:.2f} in  ({self.ld_in / 12:.2f} ft)",
            f"  db                 = {self.db_in:.4f} in",
            f"  ψt / ψe / ψs / ψg  = {self.psi_t:.2f} / {self.psi_e:.2f} "
            f"/ {self.psi_s:.2f} / {self.psi_g:.2f}",
        ]
        if self.end_type == "straight":
            lines.append(
                f"  (cb+Ktr)/db        = {self.cb_Ktr_over_db:.3f}  (cap 2.5 applied)"
            )
        for n in self.notes:
            lines.append(f"  NOTE: {n}")
        lines.append(f"  Ref: {self.code_ref}")
        return lines


def development_length_straight(
    bar_desig: str,
    fc_ksi: float,
    fy_ksi: float,
    bar_location: str = "other",      # "top" (>12 in fresh concrete) or "other"
    cover_in: float = 1.5,
    clear_spacing_in: float | None = None,
    coated: bool = False,
    grade: int = 60,                  # ASTM grade: 40, 60, 80, 100
    Ktr_in: float = 0.0,              # transverse reinf. index; 0 = conservative
) -> DevLengthResult:
    """ACI 318-19 §25.4.2.4 straight-bar development length in tension."""
    bar    = _rb.get_bar(bar_desig)
    db     = bar["db_in"]
    fc_psi = fc_ksi * 1000.0
    fy_psi = fy_ksi * 1000.0

    # ψt — casting position
    psi_t = 1.3 if bar_location == "top" else 1.0

    # ψe — epoxy coating
    if coated:
        if clear_spacing_in is not None and clear_spacing_in < 6.0 * db:
            psi_e = 1.5
        elif cover_in < 3.0 * db:
            psi_e = 1.5
        else:
            psi_e = 1.2
    else:
        psi_e = 1.0

    # ψt × ψe product capped at 1.7 §25.4.2.4
    if psi_t * psi_e > 1.7:
        psi_t = 1.7 / psi_e

    # ψs — bar size
    psi_s = 0.8 if db <= 0.750 else 1.0   # #6 and smaller

    # ψg — reinforcement grade §25.4.2.4 Table 25.4.2.5
    _grade_psi_g = {40: 0.75, 60: 1.00, 80: 1.15, 100: 1.30}
    psi_g = _grade_psi_g.get(grade, 1.00)

    # Confinement term: cb + Ktr / db, capped at 2.5
    cb_cover    = cover_in + db / 2.0
    if clear_spacing_in is not None:
        cb_spacing = db / 2.0 + clear_spacing_in / 2.0
        cb = min(cb_cover, cb_spacing)
    else:
        cb = cb_cover
    cb_Ktr_over_db = min((cb + Ktr_in) / db, 2.5)

    ld = (3.0 / 40.0) * (fy_psi / (_LAMBDA_NW * math.sqrt(fc_psi))) * \
         (psi_t * psi_e * psi_s * psi_g / cb_Ktr_over_db) * db
    ld = max(ld, 12.0)   # §25.4.2.1(c) minimum

    notes: list[str] = []
    if bar_location == "top":
        notes.append("Top bar (>12 in fresh concrete below): ψt = 1.3 applied.")
    if Ktr_in == 0.0:
        notes.append("Ktr = 0 (conservative — transverse reinforcement credit not taken).")

    return DevLengthResult(
        end_type="straight", ld_in=ld, db_in=db,
        fy_ksi=fy_ksi, fc_ksi=fc_ksi,
        psi_t=psi_t, psi_e=psi_e, psi_s=psi_s, psi_g=psi_g,
        cb_Ktr_over_db=cb_Ktr_over_db,
        notes=notes,
        code_ref="ACI 318-19 §25.4.2.4",
    )


def development_length_hook(
    bar_desig: str,
    fc_ksi: float,
    fy_ksi: float,
    hook_angle: int = 90,
    coated: bool = False,
    confining_reinf: bool = False,   # True → ψr = 0.8 (ties/stirrups ≤ 3db around hook)
    cover_ok: bool = False,          # True → ψo = 0.8 (side cov ≥ 2.5 in, end cov ≥ 2 in)
    enclosed_in_ties: bool = False,  # True → ψc = 0.8 (hook within 0-3db tie spacing)
) -> DevLengthResult:
    """ACI 318-19 §25.4.3.1 standard-hook development length (90° or 180°).

    All modification factors default to 1.0 (conservative) unless the engineer
    confirms the qualifying condition and passes the flag as True.
    """
    bar    = _rb.get_bar(bar_desig)
    db     = bar["db_in"]
    fc_psi = fc_ksi * 1000.0
    fy_psi = fy_ksi * 1000.0

    psi_e = 1.2 if coated else 1.0
    psi_r = 0.8 if confining_reinf else 1.0
    psi_o = 0.8 if cover_ok else 1.0
    psi_c = 0.8 if enclosed_in_ties else 1.0

    ldh = (fy_psi * db) / (55.0 * _LAMBDA_NW * math.sqrt(fc_psi)) * (psi_e * psi_r * psi_o * psi_c)
    ldh = max(ldh, 8.0 * db, 6.0)   # §25.4.3.1(c)

    end_type = f"hook_{hook_angle}"
    notes: list[str] = [f"{hook_angle}° standard hook."]
    notes.append(
        f"ψr={'0.8 (confining reinf. confirmed)' if confining_reinf else '1.0 (default — no confining reinf. credited)'}  "
        f"ψo={'0.8 (cover conditions confirmed)' if cover_ok else '1.0 (default — cover conditions not confirmed)'}  "
        f"ψc={'0.8 (enclosed in ties confirmed)' if enclosed_in_ties else '1.0 (default)'}"
    )

    return DevLengthResult(
        end_type=end_type, ld_in=ldh, db_in=db,
        fy_ksi=fy_ksi, fc_ksi=fc_ksi,
        psi_t=1.0, psi_e=psi_e, psi_s=psi_r, psi_g=psi_o * psi_c,
        cb_Ktr_over_db=0.0,
        notes=notes,
        code_ref="ACI 318-19 §25.4.3.1 / Table 25.4.3.2",
    )


def development_length_thead(
    bar_desig: str,
    fc_ksi: float,
    fy_ksi: float,
    coated: bool = False,
    generous_bearing: bool = False,   # Abrg ≥ 4Ab → ψp = 0.8
) -> DevLengthResult:
    """ACI 318-19 §25.4.4.1 headed deformed bar (T-head) development length.

    Applicable for #3–#11, fy ≤ 80 ksi, normal-weight concrete.
    Constraints per §25.4.4.1: clear cover ≥ 2db, clear spacing ≥ 4db.
    """
    bar    = _rb.get_bar(bar_desig)
    db     = bar["db_in"]
    fc_psi = fc_ksi * 1000.0
    fy_psi = fy_ksi * 1000.0

    if bar_desig not in _rb.THEAD_ELIGIBLE:
        raise NotImplementedError(
            f"§25.4.4 headed bars apply to #3–#11 only. {bar_desig} is not eligible."
        )
    if fy_ksi > 80.0:
        raise NotImplementedError(
            f"§25.4.4 requires fy ≤ 80 ksi (without additional qualification). "
            f"fy = {fy_ksi} ksi — escalate to senior engineer."
        )

    psi_e = 1.2 if coated else 1.0
    psi_p = 0.8 if generous_bearing else 1.0   # Abrg ≥ 4Ab

    ldt = (fy_psi * db) / (75.0 * _LAMBDA_NW * math.sqrt(fc_psi)) * (psi_e * psi_p)
    ldt = max(ldt, 8.0 * db, 6.0)   # §25.4.4.2(c)

    notes: list[str] = [
        "T-head (headed deformed bar). Verify: clear cover ≥ 2db, clear spacing ≥ 4db (§25.4.4.1)."
    ]
    if generous_bearing:
        notes.append("ψp = 0.8 applied (Abrg ≥ 4Ab confirmed).")
    else:
        notes.append("ψp = 1.0 (standard head; verify bearing area if reduction desired).")

    return DevLengthResult(
        end_type="thead", ld_in=ldt, db_in=db,
        fy_ksi=fy_ksi, fc_ksi=fc_ksi,
        psi_t=1.0, psi_e=psi_e, psi_s=1.0, psi_g=psi_p,
        cb_Ktr_over_db=0.0,
        notes=notes,
        code_ref="ACI 318-19 §25.4.4.1",
    )


# ---------------------------------------------------------------------------
# Splices — §25.5.2
# ---------------------------------------------------------------------------

def splice_length(
    bar_desig: str,
    fc_ksi: float,
    fy_ksi: float,
    splice_class: str = "B",
    bar_location: str = "other",
    cover_in: float = 1.5,
    clear_spacing_in: float | None = None,
    coated: bool = False,
    grade: int = 60,
) -> tuple[float, DevLengthResult]:
    """ACI 318-19 §25.5.2 tension lap splice.

    Returns (splice_length_in, base_ld_result).
    Class A: 1.0 × ld  (As,prov/As,req ≥ 2, ≤ 50% spliced at section)
    Class B: 1.3 × ld  (all other cases — conservative default)
    """
    dev = development_length_straight(
        bar_desig, fc_ksi, fy_ksi,
        bar_location=bar_location, cover_in=cover_in,
        clear_spacing_in=clear_spacing_in, coated=coated, grade=grade,
    )
    factor = 1.0 if splice_class.upper() == "A" else 1.3
    ls = factor * dev.ld_in
    return ls, dev


# ---------------------------------------------------------------------------
# Curtailment — §9.7.3 (simply supported, UDL)
# ---------------------------------------------------------------------------

@dataclass
class CurtailmentResult:
    n_total:                   int
    n_cut:                     int
    bar_designation:           str
    phi_Mn_remain_kip_ft:      float
    x_theoretical_ft:          float   # distance from support where Mu = φMn_remain
    extension_req_in:          float   # max(d, 12db) §9.7.3.3
    x_cutoff_ft:               float   # bars must extend AT LEAST this far from support
    ld_support_in:             float   # ld into support for remaining bars §9.7.3.5
    notes: list[str] = field(default_factory=list)

    def summary_lines(self) -> list[str]:
        lines = [
            f"CURTAILMENT (ACI 318-19 §9.7.3) — cut {self.n_cut} of {self.n_total} "
            f"{self.bar_designation} bars:",
            f"  φMn (remaining)    = {self.phi_Mn_remain_kip_ft:.2f} kip-ft",
            f"  Theoretical cutoff = {self.x_theoretical_ft:.2f} ft from support",
            f"  Extension required = {self.extension_req_in:.1f} in [max(d, 12db)]  §9.7.3.3",
            f"  Cut bars extend to ≥ {self.x_cutoff_ft:.2f} ft from support "
            f"({self.x_theoretical_ft:.2f} + {self.extension_req_in / 12:.2f} ft)",
            f"  Remaining bars ld at support ≥ {self.ld_support_in:.1f} in  §9.7.3.5",
        ]
        for n in self.notes:
            lines.append(f"  NOTE: {n}")
        return lines


def bar_curtailment(
    b_in: float, d_in: float,
    n_total: int, n_cut: int, bar_desig: str,
    fc_ksi: float, fy_ksi: float,
    wu_klf: float, span_ft: float,
) -> CurtailmentResult:
    """ACI 318-19 §9.7.3 curtailment for simply supported beam with uniform factored load.

    Finds where n_cut bars can be terminated near the supports.
    The cut bars must extend past the theoretical cutoff by max(d, 12db) toward midspan.

    Parameters
    ----------
    wu_klf   : governing factored UDL (kip/ft)
    """
    bar    = _rb.get_bar(bar_desig)
    db     = bar["db_in"]
    Ab     = bar["Ab_in2"]

    if n_cut >= n_total:
        raise ValueError("n_cut must be less than n_total (cannot cut all bars).")

    n_remain   = n_total - n_cut
    As_remain  = n_remain * Ab
    a_rem      = As_remain * fy_ksi / (0.85 * fc_ksi * b_in)
    phi_Mn_rem = _PHI_FLEX * As_remain * fy_ksi * (d_in - a_rem / 2.0) / 12.0  # kip-ft

    # Solve Mu(x) = φMn_remain for x (from support)
    # wu/2 × x × (L − x) = φMn_rem  →  x² − L·x + 2φMn_rem/wu = 0
    disc = span_ft**2 - 8.0 * phi_Mn_rem / wu_klf
    notes: list[str] = []

    if disc < 0:
        # Remaining bars carry full span — no cutoff needed
        return CurtailmentResult(
            n_total=n_total, n_cut=n_cut, bar_designation=bar_desig,
            phi_Mn_remain_kip_ft=phi_Mn_rem,
            x_theoretical_ft=0.0, extension_req_in=0.0,
            x_cutoff_ft=0.0, ld_support_in=0.0,
            notes=["Remaining bars have sufficient capacity for the full span — no cutoff required."],
        )

    x_t = (span_ft - math.sqrt(disc)) / 2.0   # closer to the support

    # §9.7.3.3: cut bars must extend at least max(d, 12db) past x_t (into the span)
    ext_in = max(d_in, 12.0 * db)
    ext_ft = ext_in / 12.0
    x_cutoff = x_t + ext_ft

    # §9.7.3.5: remaining positive-moment bars must develop ld into support
    dev = development_length_straight(bar_desig, fc_ksi, fy_ksi)
    ld_sup = dev.ld_in

    notes.append(
        f"Remaining {n_remain} bars must extend full span and develop ≥ {ld_sup:.1f} in into support."
    )
    notes.append(
        "Cut bars must also be anchored at their starting end (hook or ld from start point)."
    )
    if x_cutoff > span_ft / 2.0:
        notes.append(
            "WARNING: Required cutoff distance exceeds L/2 — cutting bars may not be practical."
        )

    return CurtailmentResult(
        n_total=n_total, n_cut=n_cut, bar_designation=bar_desig,
        phi_Mn_remain_kip_ft=phi_Mn_rem,
        x_theoretical_ft=x_t, extension_req_in=ext_in,
        x_cutoff_ft=x_cutoff, ld_support_in=ld_sup,
        notes=notes,
    )


# ---------------------------------------------------------------------------
# Composite beam-check result (for MCP workflow tools)
# ---------------------------------------------------------------------------

@dataclass
class ConcreteBeamCheck:
    b_in:            float
    h_in:            float
    d_in:            float
    n_bars:          int
    bar_designation: str
    As_in2:          float
    fc_ksi:          float
    fy_ksi:          float
    Mu_kip_ft:       float
    Vu_kips:         float
    wu_klf:          float
    governing_combo: str
    flexure:         FlexureResult
    shear:           ShearResult
    deflection:      DeflectionResult
    dev_straight:    DevLengthResult

    def summary(self) -> str:
        max_dcr = max(self.flexure.DCR, self.shear.DCR)
        tag = "OK" if max_dcr <= 1.0 else "*** OVERSTRESSED ***"
        lines = [
            f"ACI 318-19 Concrete Beam Check — {self.n_bars}{self.bar_designation}  {tag}",
            f"  Section      : {self.b_in:.0f} in × {self.h_in:.0f} in,  d = {self.d_in:.3f} in",
            f"  Material     : f'c = {self.fc_ksi:.1f} ksi,  fy = {self.fy_ksi:.0f} ksi",
            f"  Steel        : {self.n_bars}{self.bar_designation},  As = {self.As_in2:.3f} in²",
            f"  Gov combo    : {self.governing_combo} = {self.wu_klf:.3f} klf",
            f"  Demands      : Mu = {self.Mu_kip_ft:.1f} kip-ft,  Vu = {self.Vu_kips:.1f} kips",
            "-" * 60,
        ]
        lines += self.flexure.summary_lines()
        lines += [""]
        lines += self.shear.summary_lines()
        lines += [""]
        lines += self.deflection.summary_lines()
        lines += [""]
        lines += self.dev_straight.summary_lines()
        lines += [
            "",
            "Assumptions: cover = 1.5 in to stirrup face; self-weight not included in input loads;",
            "  ASCE 7 §4.7 LL reduction not applied; λΔ = 2.0 (5-yr, no comp. reinf.).",
        ]
        return "\n".join(lines)
