"""
Simply supported beam analysis using the matrix stiffness (direct stiffness) method.

Sign conventions:
    FEM internals : upward positive for forces, CCW positive for moments
    Output V      : positive = left face up / right face down (structural convention)
    Output M      : positive = sagging (tension on bottom fiber)
    Output delta  : positive = downward

Units (all imperial):
    Input  : span (ft), loads (kip/ft or kips), E (ksi), I (in⁴)
    Output : V (kips), M (kip-ft), delta (in), reactions (kips)
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional


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
