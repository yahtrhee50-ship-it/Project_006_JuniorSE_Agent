"""
Beam analysis using the matrix stiffness (direct stiffness) method.

SimpleBeam   — simply-supported (pinned-pinned) beam; backward-compatible.
GeneralBeam  — general-purpose: any end conditions (free/pin/roller/fixed),
               intermediate supports, internal hinges, variable EI per span,
               partial UDL, trapezoidal (linearly-varying) loads, point
               loads, and applied point moments.

Sign conventions (both classes)
    x       : positive rightward
    Loads   : positive = downward
    V       : positive = left-face-up / right-face-down
    M       : positive = sagging (tension at bottom)
    delta   : positive = downward
    Units   : kips, ft, in, ksi, in⁴
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SimpleBeam — simply supported, unchanged (backward-compatible)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

@dataclass
class BeamResults:
    x_ft: np.ndarray       # station positions along beam (ft)
    V_kips: np.ndarray     # shear at each station (kips)
    M_kip_ft: np.ndarray   # moment at each station (kip-ft)
    delta_in: np.ndarray   # deflection at each station (in, positive = downward)
    R_left_kips: float     # left reaction (kips, positive = upward)
    R_right_kips: float    # right reaction (kips, positive = upward)
    span_ft: float

    def summary(self) -> str:
        i_max_m = int(np.argmax(self.M_kip_ft))
        i_max_d = int(np.argmax(self.delta_in))
        return (
            f"Span         = {self.span_ft:.1f} ft\n"
            f"R_left       = {self.R_left_kips:.3f} kips\n"
            f"R_right      = {self.R_right_kips:.3f} kips\n"
            f"V_max        = {np.max(np.abs(self.V_kips)):.3f} kips  "
            f"@ x = {self.x_ft[int(np.argmax(np.abs(self.V_kips)))]:.2f} ft\n"
            f"M_max        = {self.M_kip_ft[i_max_m]:.3f} kip-ft  "
            f"@ x = {self.x_ft[i_max_m]:.2f} ft\n"
            f"delta_max    = {self.delta_in[i_max_d]:.4f} in  "
            f"@ x = {self.x_ft[i_max_d]:.2f} ft\n"
            f"L/delta      = {self.span_ft * 12 / self.delta_in[i_max_d]:.0f}"
        )

    def as_csv(self) -> str:
        """Tab-separated table for direct paste into Excel to generate diagrams."""
        lines = ["x (ft)\tV (kips)\tM (kip-ft)\tdelta (in)"]
        for x, v, m, d in zip(self.x_ft, self.V_kips, self.M_kip_ft, self.delta_in):
            lines.append(f"{x:.3f}\t{v:.3f}\t{m:.3f}\t{d:.4f}")
        return "\n".join(lines)


class SimpleBeam:
    """
    Simply supported (pinned-pinned) beam analyzed by the direct stiffness method.

    Workflow:
        beam = SimpleBeam(span_ft=30, E_ksi=29000, I_in4=518, n_elements=20)
        beam.add_udl(w_kip_per_ft=3.0)
        beam.add_point_load(P_kips=10.0, x_ft=10.0)
        results = beam.solve()
    """

    def __init__(
        self,
        span_ft: float,
        E_ksi: float,
        I_in4: float,
        n_elements: int = 20,
    ):
        if span_ft <= 0:
            raise ValueError(f"span_ft must be positive, got {span_ft}")
        if E_ksi <= 0 or I_in4 <= 0:
            raise ValueError("E and I must be positive")
        if n_elements < 4:
            raise ValueError("n_elements must be >= 4 for accuracy")

        self.span_ft = span_ft
        self.L = span_ft * 12.0       # total length (in)
        self.E = E_ksi                 # ksi
        self.I = I_in4                 # in⁴
        self.n = n_elements
        self.le = self.L / n_elements  # element length (in)

        self._udl: list[dict] = []
        self._point_loads: list[dict] = []

    def add_udl(
        self,
        w_kip_per_ft: float,
        x_start_ft: float = 0.0,
        x_end_ft: Optional[float] = None,
    ) -> None:
        """Add a uniform distributed load (positive = downward).

        Args:
            w_kip_per_ft : intensity (kip/ft, positive downward)
            x_start_ft   : load start position from left support (ft)
            x_end_ft     : load end position (ft); defaults to full span
        """
        if x_end_ft is None:
            x_end_ft = self.span_ft
        if x_start_ft < 0 or x_end_ft > self.span_ft or x_start_ft >= x_end_ft:
            raise ValueError(
                f"UDL extents [{x_start_ft}, {x_end_ft}] invalid for span {self.span_ft} ft"
            )
        self._udl.append(
            dict(w=w_kip_per_ft / 12.0, x0=x_start_ft * 12.0, x1=x_end_ft * 12.0)
        )

    def add_point_load(self, P_kips: float, x_ft: float) -> None:
        """Add a concentrated point load (positive = downward).

        Args:
            P_kips : magnitude (kips, positive downward)
            x_ft   : position from left support (ft)
        """
        if x_ft < 0 or x_ft > self.span_ft:
            raise ValueError(f"Point load position {x_ft} ft outside span {self.span_ft} ft")
        self._point_loads.append(dict(P=P_kips, x=x_ft * 12.0))

    # ── internal helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _element_stiffness(EI: float, le: float) -> np.ndarray:
        """4×4 Bernoulli beam element stiffness matrix."""
        c = EI / le**3
        return c * np.array([
            [ 12.0,    6.0*le,   -12.0,    6.0*le   ],
            [  6.0*le, 4.0*le**2, -6.0*le,  2.0*le**2],
            [-12.0,   -6.0*le,    12.0,   -6.0*le   ],
            [  6.0*le, 2.0*le**2, -6.0*le,  4.0*le**2],
        ])

    @staticmethod
    def _hermite_integrals(α: float, β: float) -> tuple[float, float, float, float]:
        """Definite integrals of Hermitian shape functions over [α, β] in ξ ∈ [0,1].

        N1 = 1 - 3ξ² + 2ξ³       → antiderivative: ξ - ξ³ + ξ⁴/2
        N2 = ξ - 2ξ² + ξ³        → antiderivative: ξ²/2 - 2ξ³/3 + ξ⁴/4
        N3 = 3ξ² - 2ξ³            → antiderivative: ξ³ - ξ⁴/2
        N4 = -ξ² + ξ³             → antiderivative: -ξ³/3 + ξ⁴/4
        """
        def s1(ξ): return ξ - ξ**3 + ξ**4 / 2
        def s2(ξ): return ξ**2 / 2 - 2*ξ**3 / 3 + ξ**4 / 4
        def s3(ξ): return ξ**3 - ξ**4 / 2
        def s4(ξ): return -ξ**3 / 3 + ξ**4 / 4
        return s1(β) - s1(α), s2(β) - s2(α), s3(β) - s3(α), s4(β) - s4(α)

    def _udl_nodal_loads(self) -> np.ndarray:
        """Assemble global load vector for all UDL cases (upward positive)."""
        n, le = self.n, self.le
        F = np.zeros(2 * (n + 1))
        for load in self._udl:
            w = load["w"]    # kip/in, positive downward
            x0, x1 = load["x0"], load["x1"]
            for i in range(n):
                el0, el1 = i * le, (i + 1) * le
                ov0 = max(el0, x0)
                ov1 = min(el1, x1)
                if ov1 <= ov0:
                    continue
                α = (ov0 - el0) / le
                β = (ov1 - el0) / le
                i1, i2, i3, i4 = self._hermite_integrals(α, β)
                # Downward load → negative in upward-positive convention
                dofs = [2*i, 2*i+1, 2*i+2, 2*i+3]
                F[dofs[0]] -= w * le * i1
                F[dofs[1]] -= w * le**2 * i2
                F[dofs[2]] -= w * le * i3
                F[dofs[3]] -= w * le**2 * i4
        return F

    def _point_load_nodal_loads(self) -> np.ndarray:
        """Assemble global load vector for all point loads (upward positive)."""
        n, le = self.n, self.le
        F = np.zeros(2 * (n + 1))
        for load in self._point_loads:
            P = load["P"]    # kips, positive downward
            x = load["x"]   # in
            i = min(int(x / le), n - 1)
            a = x - i * le   # distance from left node of element (in)
            b = le - a
            # Fixed-end forces for concentrated load (upward positive → negate downward P)
            f1 =  P * b**2 * (3*a + b) / le**3
            f2 =  P * a * b**2 / le**2
            f3 =  P * a**2 * (a + 3*b) / le**3
            f4 = -P * a**2 * b / le**2
            dofs = [2*i, 2*i+1, 2*i+2, 2*i+3]
            # Downward → negative
            F[dofs[0]] -= f1
            F[dofs[1]] -= f2
            F[dofs[2]] -= f3
            F[dofs[3]] -= f4
        return F

    # ── public solver ─────────────────────────────────────────────────────────

    def solve(self) -> BeamResults:
        """Run matrix stiffness analysis. Returns shear, moment, deflection."""
        n, le = self.n, self.le
        EI = self.E * self.I
        ndof = 2 * (n + 1)

        # ── assemble global stiffness ─────────────────────────────────────────
        K = np.zeros((ndof, ndof))
        ke = self._element_stiffness(EI, le)
        for i in range(n):
            dofs = [2*i, 2*i+1, 2*i+2, 2*i+3]
            for a, da in enumerate(dofs):
                for b, db in enumerate(dofs):
                    K[da, db] += ke[a, b]

        # ── assemble load vector ──────────────────────────────────────────────
        F = self._udl_nodal_loads() + self._point_load_nodal_loads()

        # ── apply boundary conditions (simply supported: v = 0 at x=0 and x=L) ─
        constrained = {0, 2 * n}
        free = [i for i in range(ndof) if i not in constrained]
        d = np.zeros(ndof)
        d[free] = np.linalg.solve(K[np.ix_(free, free)], F[free])

        # ── reactions at supports ─────────────────────────────────────────────
        R = K @ d - F     # upward positive
        R_left  = R[0]
        R_right = R[2 * n]

        # ── output stations: node + midpoint of each element ─────────────────
        x_pts_in = np.array(
            [coord for i in range(n) for coord in (i * le, (i + 0.5) * le)] + [n * le]
        )

        # deflection: Hermitian interpolation within each element
        delta_in = np.zeros(len(x_pts_in))
        for j, x in enumerate(x_pts_in):
            i = min(int(x / le), n - 1)
            xi = (x - i * le) / le   # ξ ∈ [0, 1]
            N1 = 1 - 3*xi**2 + 2*xi**3
            N2 = le * (xi - 2*xi**2 + xi**3)
            N3 = 3*xi**2 - 2*xi**3
            N4 = le * (-xi**2 + xi**3)
            dofs = [2*i, 2*i+1, 2*i+2, 2*i+3]
            delta_in[j] = -(N1*d[dofs[0]] + N2*d[dofs[1]] + N3*d[dofs[2]] + N4*d[dofs[3]])
            # negated: FEM d is upward positive; delta_out is downward positive

        # shear and moment: equilibrium from left support
        x_ft_arr = x_pts_in / 12.0
        V_kips    = np.zeros(len(x_pts_in))
        M_kip_ft  = np.zeros(len(x_pts_in))

        for j, x_ft in enumerate(x_ft_arr):
            V = R_left
            M = R_left * x_ft
            for load in self._udl:
                w_kft  = load["w"] * 12.0    # kip/ft
                x0_ft  = load["x0"] / 12.0
                x1_ft  = load["x1"] / 12.0
                x_cov  = min(x1_ft, x_ft)
                if x_cov > x0_ft:
                    L_cov = x_cov - x0_ft
                    cen   = x0_ft + L_cov / 2
                    V -= w_kft * L_cov
                    M -= w_kft * L_cov * (x_ft - cen)
            for load in self._point_loads:
                P    = load["P"]
                xP_ft = load["x"] / 12.0
                if xP_ft < x_ft - 1e-9:
                    V -= P
                    M -= P * (x_ft - xP_ft)
            V_kips[j]   = V
            M_kip_ft[j] = M

        return BeamResults(
            x_ft=x_ft_arr,
            V_kips=V_kips,
            M_kip_ft=M_kip_ft,
            delta_in=delta_in,
            R_left_kips=float(R_left),
            R_right_kips=float(R_right),
            span_ft=self.span_ft,
        )


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# GeneralBeam — general-purpose continuous/fixed/cantilever beam solver
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

_VALID_BC = frozenset({"free", "pin", "roller", "fixed"})

# 8-point Gauss-Legendre nodes and weights on [-1, 1]
_GAUSS_XI, _GAUSS_WT = np.polynomial.legendre.leggauss(8)


@dataclass
class GeneralBeamResults:
    x_ft: np.ndarray
    V_kips: np.ndarray
    M_kip_ft: np.ndarray
    delta_in: np.ndarray
    reactions: dict          # {x_ft (float): R_kips (float), upward positive}
    moment_reactions: dict   # {x_ft (float): M_kip_ft (float)} at fixed supports
    total_length_ft: float

    def summary(self) -> str:
        i_Mpos = int(np.argmax(self.M_kip_ft))
        i_Mneg = int(np.argmin(self.M_kip_ft))
        i_d    = int(np.argmax(self.delta_in))
        Vmax   = float(np.max(np.abs(self.V_kips)))
        lines  = [f"Total length  = {self.total_length_ft:.1f} ft", "Reactions (kips, upward+):"]
        for x, R in sorted(self.reactions.items()):
            lines.append(f"  x = {x:.3f} ft  R = {R:.3f} kips")
        for x, M in sorted(self.moment_reactions.items()):
            lines.append(f"  x = {x:.3f} ft  M_reaction = {M:.3f} kip-ft")
        lines += [
            f"V_max         = {Vmax:.3f} kips",
            f"M_max (+sag)  = {self.M_kip_ft[i_Mpos]:.3f} kip-ft  @ x = {self.x_ft[i_Mpos]:.2f} ft",
            f"M_max (-hog)  = {self.M_kip_ft[i_Mneg]:.3f} kip-ft  @ x = {self.x_ft[i_Mneg]:.2f} ft",
            f"delta_max     = {self.delta_in[i_d]:.4f} in  @ x = {self.x_ft[i_d]:.2f} ft",
        ]
        if self.delta_in[i_d] > 1e-9:
            lines.append(f"L/delta       = {self.total_length_ft * 12 / self.delta_in[i_d]:.0f}")
        return "\n".join(lines)

    def as_csv(self) -> str:
        lines = ["x (ft)\tV (kips)\tM (kip-ft)\tdelta (in)"]
        for x, v, m, d in zip(self.x_ft, self.V_kips, self.M_kip_ft, self.delta_in):
            lines.append(f"{x:.4f}\t{v:.4f}\t{m:.4f}\t{d:.5f}")
        return "\n".join(lines)


class GeneralBeam:
    """
    General continuous beam solver — direct stiffness method (Euler-Bernoulli).

    Supports any combination of:
    - End BCs: free, pin/roller, fixed
    - Intermediate supports: pin/roller (via add_support)
    - Internal moment releases (hinges) at arbitrary positions (via add_hinge)
    - Variable EI per span (via set_ei)
    - Loads: full/partial UDL, trapezoidal (linearly-varying), point loads,
             applied point moments (CCW positive)

    Quick examples::

        # Fixed-fixed beam, full UDL
        b = GeneralBeam(30, E_ksi=29000, I_in4=518)
        b.set_bc("fixed", "fixed")
        b.add_udl(2.0)
        r = b.solve()

        # Two-span continuous beam
        b = GeneralBeam.continuous([20, 25], E_ksi=29000, I_in4=518)
        b.add_udl(2.0, x_end_ft=20)
        b.add_udl(1.5, x_start_ft=20)
        r = b.solve()

        # Propped cantilever with hinge at 10 ft
        b = GeneralBeam(30, E_ksi=29000, I_in4=518)
        b.set_bc("fixed", "pin")
        b.add_hinge(10.0)
        b.add_udl(2.0)
        r = b.solve()
    """

    # ── construction ──────────────────────────────────────────────────────────

    def __init__(
        self,
        total_length_ft: float,
        E_ksi: float = 29000.0,
        I_in4: float = 100.0,
        n_elements: int = 20,
    ):
        """
        Args:
            total_length_ft : total beam length (ft).
            E_ksi           : default elastic modulus (ksi); override per span with set_ei().
            I_in4           : default moment of inertia (in⁴); override per span with set_ei().
            n_elements      : mesh elements per span (≥ 4); more = more accurate deflection.
        """
        if total_length_ft <= 0:
            raise ValueError("total_length_ft must be positive")
        if n_elements < 4:
            raise ValueError("n_elements must be >= 4")

        self._L_ft = float(total_length_ft)
        self._default_E = float(E_ksi)
        self._default_I = float(I_in4)
        self._n_per_span = int(n_elements)

        self._left_bc: str = "pin"
        self._right_bc: str = "pin"
        self._supports: dict[float, str] = {}      # x_in → bc
        self._hinges: set[float] = set()           # x_in
        self._ei_segs: list[tuple[float, float, float, float]] = []  # (x0_in, x1_in, E, I)

        self._udls: list[dict] = []
        self._trap_loads: list[dict] = []
        self._point_loads: list[dict] = []
        self._point_moments: list[dict] = []       # {M_kip_in, x_in} (CCW +)
        self._prescribed_v: dict[float, float] = {}  # x_in → delta_in (prescribed settlement)

    @classmethod
    def continuous(
        cls,
        spans_ft: list[float],
        E_ksi: float = 29000.0,
        I_in4: "float | list[float]" = 100.0,
        left_bc: str = "pin",
        right_bc: str = "pin",
        n_elements: int = 20,
    ) -> "GeneralBeam":
        """
        Convenience constructor: continuous beam with interior pin supports.

        Args:
            spans_ft  : list of span lengths (ft), e.g. [20, 15, 25]
            E_ksi     : elastic modulus (ksi)
            I_in4     : moment of inertia — scalar (all spans) or list per span
            left_bc   : "pin", "fixed", or "free"
            right_bc  : "pin", "fixed", or "free"
            n_elements: mesh elements per span
        """
        if isinstance(I_in4, (int, float)):
            I_list = [float(I_in4)] * len(spans_ft)
        else:
            if len(I_in4) != len(spans_ft):
                raise ValueError("len(I_in4) must equal len(spans_ft)")
            I_list = [float(v) for v in I_in4]

        beam = cls(sum(spans_ft), E_ksi, I_list[0], n_elements)
        beam.set_bc(left_bc, right_bc)

        x = 0.0
        for k, (span, I_span) in enumerate(zip(spans_ft, I_list)):
            beam.set_ei(x, x + span, E_ksi, I_span)
            if k < len(spans_ft) - 1:
                beam.add_support(x + span, bc="pin")
            x += span

        return beam

    # ── geometry / BCs ───────────────────────────────────────────────────────

    def set_bc(self, left: str = "pin", right: str = "pin") -> None:
        """Set left and right end boundary conditions."""
        _check_bc(left)
        _check_bc(right)
        self._left_bc = left
        self._right_bc = right

    def add_support(self, x_ft: float, bc: str = "pin") -> None:
        """Add an intermediate support (pin or roller) at x_ft (ft from left)."""
        _check_bc(bc)
        if x_ft <= 0 or x_ft >= self._L_ft:
            raise ValueError(f"Intermediate support x={x_ft} must be strictly inside (0, {self._L_ft})")
        self._supports[x_ft * 12.0] = bc

    def set_support_settlement(self, x_ft: float, delta_in: float) -> None:
        """Prescribe a vertical displacement at a support location (positive = upward)."""
        self._prescribed_v[x_ft * 12.0] = float(delta_in)

    def add_hinge(self, x_ft: float) -> None:
        """Add an internal moment release (hinge) at x_ft (ft from left)."""
        if x_ft <= 0 or x_ft >= self._L_ft:
            raise ValueError(f"Hinge x={x_ft} must be strictly inside the beam")
        self._hinges.add(x_ft * 12.0)

    def set_ei(
        self,
        x_start_ft: float,
        x_end_ft: float,
        E_ksi: float,
        I_in4: float,
    ) -> None:
        """Override E and I for span [x_start_ft, x_end_ft] (ft). Last override wins."""
        self._ei_segs.append((x_start_ft * 12.0, x_end_ft * 12.0, float(E_ksi), float(I_in4)))

    # ── loads ─────────────────────────────────────────────────────────────────

    def add_udl(
        self,
        w_kip_per_ft: float,
        x_start_ft: float = 0.0,
        x_end_ft: Optional[float] = None,
    ) -> None:
        """Uniform distributed load (kip/ft, positive = downward)."""
        if x_end_ft is None:
            x_end_ft = self._L_ft
        self._check_range(x_start_ft, x_end_ft)
        self._udls.append(dict(
            w=w_kip_per_ft / 12.0,     # kip/in
            x0=x_start_ft * 12.0,
            x1=x_end_ft * 12.0,
        ))

    def add_trapload(
        self,
        w_start_kip_per_ft: float,
        w_end_kip_per_ft: float,
        x_start_ft: float = 0.0,
        x_end_ft: Optional[float] = None,
    ) -> None:
        """
        Linearly-varying distributed load (kip/ft, positive = downward).
        Intensity varies linearly from w_start at x_start to w_end at x_end.
        Useful for hydrostatic or soil-pressure loads.
        """
        if x_end_ft is None:
            x_end_ft = self._L_ft
        self._check_range(x_start_ft, x_end_ft)
        self._trap_loads.append(dict(
            w0=w_start_kip_per_ft / 12.0,   # kip/in at x_start
            w1=w_end_kip_per_ft / 12.0,     # kip/in at x_end
            x0=x_start_ft * 12.0,
            x1=x_end_ft * 12.0,
        ))

    def add_point_load(self, P_kips: float, x_ft: float) -> None:
        """Concentrated load (kips, positive = downward) at x_ft."""
        if x_ft < 0 or x_ft > self._L_ft:
            raise ValueError(f"Point load x={x_ft} outside beam")
        self._point_loads.append(dict(P=float(P_kips), x=x_ft * 12.0))

    def add_point_moment(self, M_kip_ft: float, x_ft: float) -> None:
        """
        Applied point moment (kip-ft, positive = CCW) at x_ft.
        Positive CCW increases sagging moment to the right of the application point.
        """
        if x_ft < 0 or x_ft > self._L_ft:
            raise ValueError(f"Point moment x={x_ft} outside beam")
        self._point_moments.append(dict(M=M_kip_ft * 12.0, x=x_ft * 12.0))

    # ── solver ────────────────────────────────────────────────────────────────

    def solve(self) -> GeneralBeamResults:
        """Run matrix stiffness analysis. Returns V/M/delta diagrams and reactions."""
        node_x, is_hinge_node = self._build_mesh()
        v_dof, th_left, th_right, n_dof = self._assign_dofs(node_x, is_hinge_node)
        K = self._assemble_K(node_x, v_dof, th_right, th_left, n_dof)
        F = self._assemble_F(node_x, v_dof, th_right, th_left, n_dof)
        constrained = self._constrained_dofs(node_x, v_dof, th_left, th_right)

        free = [i for i in range(n_dof) if i not in constrained]
        d = np.zeros(n_dof)
        if len(constrained) < 2:
            raise np.linalg.LinAlgError(
                "Singular stiffness matrix — beam has fewer than 2 support DOFs"
            )
        # Apply prescribed support displacements (settlement / imposed movement)
        for x_in, delta in self._prescribed_v.items():
            node_idx = min(range(len(node_x)), key=lambda j: abs(node_x[j] - x_in))
            d[v_dof[node_idx]] = delta
        if free:
            constrained_list = sorted(constrained)
            F_eff = F[free] - K[np.ix_(free, constrained_list)] @ d[constrained_list]
            d[free] = np.linalg.solve(K[np.ix_(free, free)], F_eff)

        R_all = K @ d - F   # reaction vector (upward + at constrained DOFs)

        reactions, moment_reactions = self._extract_reactions(
            node_x, v_dof, th_left, R_all
        )

        x_out, delta_out = self._interp_delta(node_x, v_dof, th_right, th_left, d)
        V_out, M_out = self._equilibrium_vm(x_out, reactions, moment_reactions)

        return GeneralBeamResults(
            x_ft=x_out / 12.0,
            V_kips=V_out,
            M_kip_ft=M_out,
            delta_in=delta_out,
            reactions={x / 12.0: R for x, R in reactions.items()},
            moment_reactions={x / 12.0: M for x, M in moment_reactions.items()},
            total_length_ft=self._L_ft,
        )

    # ── private: mesh ─────────────────────────────────────────────────────────

    def _build_mesh(self) -> tuple[list[float], list[bool]]:
        """
        Build mesh between structural breakpoints (ends + supports + hinges).
        Returns (node_x_in, is_hinge_per_node).
        """
        L = self._L_ft * 12.0
        breakpts = sorted({0.0, L} | set(self._supports.keys()) | self._hinges)

        node_x: list[float] = []
        is_hinge: list[bool] = []

        for k in range(len(breakpts) - 1):
            x0, x1 = breakpts[k], breakpts[k + 1]
            n = self._n_per_span
            le = (x1 - x0) / n

            start_j = 0 if k == 0 else 1  # first span adds node 0; subsequent skip shared node
            for j in range(start_j, n + 1):
                xj = x0 + j * le if j < n else x1   # snap last node to exact breakpoint
                node_x.append(xj)
                is_hinge.append(any(abs(xj - xh) < 1e-9 for xh in self._hinges))

        return node_x, is_hinge

    # ── private: DOF assignment ───────────────────────────────────────────────

    @staticmethod
    def _assign_dofs(
        node_x: list[float],
        is_hinge: list[bool],
    ) -> tuple[list[int], list[int], list[int], int]:
        """
        Assign global DOF indices.

        Each node i has:
          v_dof[i]      = 2*i       (vertical displacement, shared)
          th_left[i]    = 2*i + 1   (rotation — used by element arriving from left)
          th_right[i]   = 2*i + 1   (same, unless hinge → gets an extra independent DOF)

        For element from node i to node i+1:
          element DOFs = [v_dof[i], th_right[i], v_dof[i+1], th_left[i+1]]

        At a hinge node: th_left[i] ≠ th_right[i] → no moment coupling between
        the element to the left and the element to the right.
        """
        N = len(node_x)
        v_dof = [2 * i for i in range(N)]
        th_left = [2 * i + 1 for i in range(N)]
        th_right = list(th_left)   # same by default

        n_extra = 0
        for i in range(N):
            if is_hinge[i]:
                th_right[i] = 2 * N + n_extra
                n_extra += 1

        return v_dof, th_left, th_right, 2 * N + n_extra

    # ── private: stiffness assembly ───────────────────────────────────────────

    def _assemble_K(
        self,
        node_x: list[float],
        v_dof: list[int],
        th_right: list[int],
        th_left: list[int],
        n_dof: int,
    ) -> np.ndarray:
        K = np.zeros((n_dof, n_dof))
        for i in range(len(node_x) - 1):
            xa, xb = node_x[i], node_x[i + 1]
            le = xb - xa
            E, I = self._get_ei(0.5 * (xa + xb))
            ke = _bernoulli_4x4(E * I, le)
            dofs = [v_dof[i], th_right[i], v_dof[i + 1], th_left[i + 1]]
            for a, da in enumerate(dofs):
                for b, db in enumerate(dofs):
                    K[da, db] += ke[a, b]
        return K

    # ── private: load vector ──────────────────────────────────────────────────

    def _assemble_F(
        self,
        node_x: list[float],
        v_dof: list[int],
        th_right: list[int],
        th_left: list[int],
        n_dof: int,
    ) -> np.ndarray:
        """
        Assemble global load vector (upward positive).
        UDL and trapezoid loads use Gauss quadrature; point loads use exact FEF;
        point moments use the derivative of Hermitian shape functions.
        """
        F = np.zeros(n_dof)
        n_elem = len(node_x) - 1

        for i in range(n_elem):
            xa, xb = node_x[i], node_x[i + 1]
            le = xb - xa
            dofs = [v_dof[i], th_right[i], v_dof[i + 1], th_left[i + 1]]

            # ── UDLs (closed-form via Hermite integrals) ──────────────────────
            for load in self._udls:
                w, x0, x1 = load["w"], load["x0"], load["x1"]
                ov0, ov1 = max(xa, x0), min(xb, x1)
                if ov1 <= ov0:
                    continue
                α, β = (ov0 - xa) / le, (ov1 - xa) / le
                i1, i2, i3, i4 = _hermite_ints(α, β)
                F[dofs[0]] -= w * le * i1
                F[dofs[1]] -= w * le**2 * i2
                F[dofs[2]] -= w * le * i3
                F[dofs[3]] -= w * le**2 * i4

            # ── Trapezoidal loads (8-point Gauss quadrature) ──────────────────
            for load in self._trap_loads:
                w0, w1, x0, x1 = load["w0"], load["w1"], load["x0"], load["x1"]
                ov0, ov1 = max(xa, x0), min(xb, x1)
                if ov1 <= ov0:
                    continue
                # Map [ov0, ov1] → ξ ∈ [0, 1] within element
                α_in = ov0 - xa          # inches from element left
                β_in = ov1 - xa
                # 8-pt Gauss on [α_in, β_in]
                xi_g = α_in + (β_in - α_in) * (_GAUSS_XI + 1) / 2    # inches
                wt_g = _GAUSS_WT * (β_in - α_in) / 2
                for xg, wg in zip(xi_g, wt_g):
                    xi = xg / le          # ξ ∈ [0, 1]
                    # Intensity at this Gauss point (linear interpolation in x)
                    t = (xa + xg - x0) / (x1 - x0)   # fraction along load extent
                    w_g = w0 + (w1 - w0) * t
                    N1 = 1 - 3*xi**2 + 2*xi**3
                    N2 = xi - 2*xi**2 + xi**3
                    N3 = 3*xi**2 - 2*xi**3
                    N4 = -xi**2 + xi**3
                    F[dofs[0]] -= w_g * N1 * wg
                    F[dofs[1]] -= w_g * le * N2 * wg
                    F[dofs[2]] -= w_g * N3 * wg
                    F[dofs[3]] -= w_g * le * N4 * wg

            # ── Point loads (exact fixed-end forces) ──────────────────────────
            for load in self._point_loads:
                P, xp = load["P"], load["x"]
                # Half-open [xa, xb) prevents double-counting at shared nodes;
                # last element uses closed [xa, xb] to capture loads at the tip.
                if i < n_elem - 1:
                    if not (xa - 1e-9 <= xp < xb - 1e-9):
                        continue
                else:
                    if not (xa - 1e-9 <= xp <= xb + 1e-9):
                        continue
                a = max(0.0, min(xp - xa, le))
                b = le - a
                F[dofs[0]] -= P * b**2 * (3*a + b) / le**3
                F[dofs[1]] -= P * a * b**2 / le**2
                F[dofs[2]] -= P * a**2 * (a + 3*b) / le**3
                F[dofs[3]] += P * a**2 * b / le**2

            # ── Point moments (derivative of Hermitian shape functions) ───────
            for load in self._point_moments:
                M_ext, xm = load["M"], load["x"]    # kip-in, CCW positive
                if i < n_elem - 1:
                    if not (xa - 1e-9 <= xm < xb - 1e-9):
                        continue
                else:
                    if not (xa - 1e-9 <= xm <= xb + 1e-9):
                        continue
                xi = max(0.0, min((xm - xa) / le, 1.0))
                dN1 = (-6*xi + 6*xi**2) / le        # dN1/dx
                dN2 = 1 - 4*xi + 3*xi**2            # dN2/dξ (dimensionless)
                dN3 = (6*xi - 6*xi**2) / le         # dN3/dx
                dN4 = -2*xi + 3*xi**2               # dN4/dξ (dimensionless)
                F[dofs[0]] -= M_ext * dN1
                F[dofs[1]] -= M_ext * dN2
                F[dofs[2]] -= M_ext * dN3
                F[dofs[3]] -= M_ext * dN4

        return F

    # ── private: boundary conditions ──────────────────────────────────────────

    def _constrained_dofs(
        self,
        node_x: list[float],
        v_dof: list[int],
        th_left: list[int],
        th_right: list[int],
    ) -> set[int]:
        """Return set of constrained DOF indices from end and intermediate BCs."""
        constrained: set[int] = set()
        N = len(node_x)

        def apply_bc(node_idx: int, bc: str) -> None:
            if bc in ("pin", "roller"):
                constrained.add(v_dof[node_idx])
            elif bc == "fixed":
                constrained.add(v_dof[node_idx])
                constrained.add(th_left[node_idx])
                if th_right[node_idx] != th_left[node_idx]:
                    constrained.add(th_right[node_idx])

        apply_bc(0, self._left_bc)
        apply_bc(N - 1, self._right_bc)

        for x_in, bc in self._supports.items():
            idx = min(range(N), key=lambda j: abs(node_x[j] - x_in))
            apply_bc(idx, bc)

        return constrained

    # ── private: reaction extraction ──────────────────────────────────────────

    def _extract_reactions(
        self,
        node_x: list[float],
        v_dof: list[int],
        th_left: list[int],
        R_all: np.ndarray,
    ) -> tuple[dict[float, float], dict[float, float]]:
        """Return (vertical_reactions, moment_reactions) dicts keyed by x_in."""
        N = len(node_x)
        reactions: dict[float, float] = {}
        moment_reactions: dict[float, float] = {}

        def record(node_idx: int, bc: str) -> None:
            xi = node_x[node_idx]
            if bc in ("pin", "roller", "fixed"):
                reactions[xi] = float(R_all[v_dof[node_idx]])
            if bc == "fixed":
                M_rxn = float(R_all[th_left[node_idx]])
                if abs(M_rxn) > 1e-10:
                    moment_reactions[xi] = M_rxn / 12.0  # kip-in → kip-ft

        record(0, self._left_bc)
        record(N - 1, self._right_bc)

        for x_in, bc in self._supports.items():
            idx = min(range(N), key=lambda j: abs(node_x[j] - x_in))
            record(idx, bc)

        return reactions, moment_reactions

    # ── private: deflection post-processing ───────────────────────────────────

    def _interp_delta(
        self,
        node_x: list[float],
        v_dof: list[int],
        th_right: list[int],
        th_left: list[int],
        d: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Hermitian interpolation for deflection at output stations.
        Output stations: left node + midpoint of each element, plus final node.
        """
        n_elem = len(node_x) - 1
        x_out: list[float] = []
        delta_out: list[float] = []

        for i in range(n_elem):
            xa, xb = node_x[i], node_x[i + 1]
            le = xb - xa
            dofs = [v_dof[i], th_right[i], v_dof[i + 1], th_left[i + 1]]

            for xp in (xa, 0.5 * (xa + xb)):
                xi = (xp - xa) / le
                N1 = 1 - 3*xi**2 + 2*xi**3
                N2 = le * (xi - 2*xi**2 + xi**3)
                N3 = 3*xi**2 - 2*xi**3
                N4 = le * (-xi**2 + xi**3)
                v_up = N1*d[dofs[0]] + N2*d[dofs[1]] + N3*d[dofs[2]] + N4*d[dofs[3]]
                x_out.append(xp)
                delta_out.append(-v_up)   # upward + → downward + output

        # Final node
        xi = 1.0
        i = n_elem - 1
        xa = node_x[i]
        le = node_x[i + 1] - xa
        dofs = [v_dof[i], th_right[i], v_dof[i + 1], th_left[i + 1]]
        N1, N2, N3, N4 = 0.0, 0.0, 1.0, 0.0
        v_up = N1*d[dofs[0]] + N2*d[dofs[1]] + N3*d[dofs[2]] + N4*d[dofs[3]]
        x_out.append(node_x[-1])
        delta_out.append(-v_up)

        return np.array(x_out), np.array(delta_out)

    # ── private: V/M by equilibrium ───────────────────────────────────────────

    def _equilibrium_vm(
        self,
        x_out: np.ndarray,
        reactions: dict[float, float],
        moment_reactions: dict[float, float] | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute shear V(x) and moment M(x) at each output station by summing
        all forces/moments to the LEFT of the station (method of sections).

        V positive = left-face-up; M positive = sagging.
        Reactions are upward positive; applied loads are downward positive.
        moment_reactions: {x_in: M_kip_ft} — CCW positive moment from fixed supports.
        """
        V = np.zeros(len(x_out))
        M = np.zeros(len(x_out))

        for j, x_in in enumerate(x_out):
            x_ft = x_in / 12.0
            v = 0.0
            m = 0.0

            # Upward support reactions to the left of (and at) x
            for x_rxn, R in reactions.items():
                if x_rxn <= x_in + 1e-9:
                    v += R
                    m += R * (x_ft - x_rxn / 12.0)

            # Moment reactions at fixed supports.
            # FEM R_all[θ] = moment the support exerts on the beam (CCW+), but
            # in the method-of-sections sum the sign is opposite: m -= M_rxn.
            # Include only reactions strictly to the LEFT of x, except the left
            # end (x_rxn ≈ 0) is included when cutting AT x = 0.
            if moment_reactions:
                for x_rxn, M_rxn in moment_reactions.items():
                    if x_rxn < x_in - 1e-9 or (x_rxn < 1e-9 and x_in < 1e-9):
                        m -= M_rxn

            # UDLs
            for load in self._udls:
                w_kft = load["w"] * 12.0           # kip/ft
                x0_ft = load["x0"] / 12.0
                x1_ft = load["x1"] / 12.0
                x_cov = min(x1_ft, x_ft)
                if x_cov > x0_ft:
                    L_cov = x_cov - x0_ft
                    cen   = x0_ft + L_cov / 2.0
                    v -= w_kft * L_cov
                    m -= w_kft * L_cov * (x_ft - cen)

            # Trapezoidal loads
            for load in self._trap_loads:
                w0_kft = load["w0"] * 12.0
                w1_kft = load["w1"] * 12.0
                x0_ft = load["x0"] / 12.0
                x1_ft = load["x1"] / 12.0
                x_cov = min(x1_ft, x_ft)
                if x_cov > x0_ft:
                    frac = (x_cov - x0_ft) / (x1_ft - x0_ft) if x1_ft > x0_ft else 1.0
                    w_cov = w0_kft + frac * (w1_kft - w0_kft)
                    W = 0.5 * (w0_kft + w_cov) * (x_cov - x0_ft)
                    if abs(w_cov - w0_kft) < 1e-12:
                        cen = x0_ft + (x_cov - x0_ft) / 2.0
                    else:
                        cen = x0_ft + (x_cov - x0_ft) * (2*w_cov + w0_kft) / (3*(w0_kft + w_cov))
                    v -= W
                    m -= W * (x_ft - cen)

            # Point loads (strictly to the left of x)
            for load in self._point_loads:
                xP_ft = load["x"] / 12.0
                if xP_ft < x_ft - 1e-9:
                    v -= load["P"]
                    m -= load["P"] * (x_ft - xP_ft)

            # Applied point moments (CCW + → adds to M for stations to the right)
            for load in self._point_moments:
                xM_ft = load["x"] / 12.0
                if xM_ft < x_ft - 1e-9:
                    m += load["M"] / 12.0   # kip-in → kip-ft

            V[j] = v
            M[j] = m

        return V, M

    # ── private: utilities ────────────────────────────────────────────────────

    def _get_ei(self, x_in: float) -> tuple[float, float]:
        """Return (E_ksi, I_in4) at position x_in; last matching override wins."""
        E, I = self._default_E, self._default_I
        for x0, x1, Eo, Io in self._ei_segs:
            if x0 <= x_in <= x1:
                E, I = Eo, Io
        return E, I

    def _check_range(self, x0: float, x1: float) -> None:
        if x0 < 0 or x1 > self._L_ft or x0 >= x1:
            raise ValueError(f"Range [{x0}, {x1}] invalid for beam {self._L_ft} ft")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Module-level helpers (used by GeneralBeam)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _check_bc(bc: str) -> None:
    if bc not in _VALID_BC:
        raise ValueError(f"bc must be one of {set(_VALID_BC)}, got '{bc}'")


def _bernoulli_4x4(EI: float, le: float) -> np.ndarray:
    """4×4 Euler-Bernoulli beam element stiffness matrix."""
    c = EI / le**3
    return c * np.array([
        [ 12.0,   6.0*le,  -12.0,   6.0*le  ],
        [  6.0*le, 4.0*le**2, -6.0*le, 2.0*le**2],
        [-12.0,  -6.0*le,   12.0,  -6.0*le  ],
        [  6.0*le, 2.0*le**2, -6.0*le, 4.0*le**2],
    ], dtype=float)


def _hermite_ints(α: float, β: float) -> tuple[float, float, float, float]:
    """Definite integrals of Hermite shape functions over [α, β] ⊆ [0, 1]."""
    def s1(x): return x - x**3 + x**4 / 2
    def s2(x): return x**2 / 2 - 2*x**3 / 3 + x**4 / 4
    def s3(x): return x**3 - x**4 / 2
    def s4(x): return -x**3 / 3 + x**4 / 4
    return s1(β)-s1(α), s2(β)-s2(α), s3(β)-s3(α), s4(β)-s4(α)
