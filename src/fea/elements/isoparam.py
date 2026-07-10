"""
Isoparametric element machinery shared by the continuum/shell family
(Phase 2, brief §4 "shared element machinery"): parent-domain shape
functions with parametric derivatives, Gauss quadrature rules, the
Jacobian mapping, and strain-displacement (B) builders.

Quad4 (Step 2) consumes the 2D pieces; the MITC4 shell membrane part
(Step 5) and Hex8/Tet4 solids (Step 6) reuse the same kernels, so this
module is built and unit-tested once (partition of unity, derivative sums,
identity Jacobian on the parent square).

Parent domains: Q4 is the bi-unit square [-1, 1]^2 with CCW corner order
(-1,-1), (1,-1), (1,1), (-1,1); H8 is the bi-unit cube with the same
corner pattern repeated at zeta = -1 then zeta = +1.
"""
from __future__ import annotations

from typing import Tuple

import numpy as np

# Q4 parent-square corner coordinates, CCW from (-1, -1).
Q4_CORNERS = np.array([
    [-1.0, -1.0],
    [ 1.0, -1.0],
    [ 1.0,  1.0],
    [-1.0,  1.0],
])


def shape_q4(xi: float, eta: float) -> Tuple[np.ndarray, np.ndarray]:
    """Bilinear Q4 shape functions on the parent square.

    Returns (N, dN_dxi): N is (4,), dN_dxi is (4, 2) with columns
    [dN/dxi, dN/deta], rows in the CCW corner order of Q4_CORNERS.
    """
    xc = Q4_CORNERS[:, 0]
    ec = Q4_CORNERS[:, 1]
    N = 0.25 * (1.0 + xi * xc) * (1.0 + eta * ec)
    dN = np.empty((4, 2))
    dN[:, 0] = 0.25 * xc * (1.0 + eta * ec)
    dN[:, 1] = 0.25 * ec * (1.0 + xi * xc)
    return N, dN


# H8 parent-cube corners: the Q4 CCW pattern at zeta = -1, then at +1.
H8_CORNERS = np.array([
    [-1.0, -1.0, -1.0],
    [ 1.0, -1.0, -1.0],
    [ 1.0,  1.0, -1.0],
    [-1.0,  1.0, -1.0],
    [-1.0, -1.0,  1.0],
    [ 1.0, -1.0,  1.0],
    [ 1.0,  1.0,  1.0],
    [-1.0,  1.0,  1.0],
])


def shape_h8(xi: float, eta: float, zeta: float
             ) -> Tuple[np.ndarray, np.ndarray]:
    """Trilinear H8 shape functions on the parent cube.

    Returns (N, dN_dxi): N is (8,), dN_dxi is (8, 3) with columns
    [dN/dxi, dN/deta, dN/dzeta], rows in the corner order of H8_CORNERS.
    """
    xc = H8_CORNERS[:, 0]
    ec = H8_CORNERS[:, 1]
    zc = H8_CORNERS[:, 2]
    N = 0.125 * (1.0 + xi * xc) * (1.0 + eta * ec) * (1.0 + zeta * zc)
    dN = np.empty((8, 3))
    dN[:, 0] = 0.125 * xc * (1.0 + eta * ec) * (1.0 + zeta * zc)
    dN[:, 1] = 0.125 * ec * (1.0 + xi * xc) * (1.0 + zeta * zc)
    dN[:, 2] = 0.125 * zc * (1.0 + xi * xc) * (1.0 + eta * ec)
    return N, dN


def gauss_1d(n: int) -> Tuple[np.ndarray, np.ndarray]:
    """Gauss-Legendre rule on [-1, 1]: (points, weights) for n = 1, 2, 3."""
    if n == 1:
        return np.array([0.0]), np.array([2.0])
    if n == 2:
        g = 1.0 / np.sqrt(3.0)
        return np.array([-g, g]), np.array([1.0, 1.0])
    if n == 3:
        g = np.sqrt(3.0 / 5.0)
        return np.array([-g, 0.0, g]), np.array([5.0, 8.0, 5.0]) / 9.0
    raise ValueError(f"gauss_1d: unsupported rule n={n} (use 1, 2 or 3)")


def gauss_2d(n: int = 2) -> Tuple[np.ndarray, np.ndarray]:
    """Tensor-product rule on the parent square: (points (n*n, 2),
    weights (n*n,)), xi varying fastest within each eta row."""
    p, w = gauss_1d(n)
    pts = np.array([[xi, eta] for eta in p for xi in p])
    wts = np.array([we * wx for we in w for wx in w])
    return pts, wts


def gauss_3d(n: int = 2) -> Tuple[np.ndarray, np.ndarray]:
    """Tensor-product rule on the parent cube: (points (n^3, 3),
    weights (n^3,)), xi fastest, then eta, then zeta."""
    p, w = gauss_1d(n)
    pts = np.array([[xi, eta, ze] for ze in p for eta in p for xi in p])
    wts = np.array([wz * we * wx for wz in w for we in w for wx in w])
    return pts, wts


def jacobian(coords: np.ndarray, dN_dxi: np.ndarray
             ) -> Tuple[np.ndarray, float, np.ndarray]:
    """Isoparametric Jacobian at one integration point.

    coords : (n_nodes, ndm) physical node coordinates
    dN_dxi : (n_nodes, ndm) parametric shape-function derivatives

    Returns (J, detJ, dN_dx) with J[i, j] = dx_i/dxi_j and
    dN_dx = dN_dxi @ inv(J), the physical derivatives. detJ is signed —
    callers gate on detJ > 0 (a non-positive value means the element is
    inverted or its nodes are numbered clockwise).
    """
    J = coords.T @ dN_dxi
    detJ = float(np.linalg.det(J))
    if detJ == 0.0:
        raise ValueError("Singular isoparametric Jacobian (degenerate element)")
    dN_dx = dN_dxi @ np.linalg.inv(J)
    return J, detJ, dN_dx


def b_matrix_2d(dN_dx: np.ndarray) -> np.ndarray:
    """2D strain-displacement matrix, engineering (Voigt) strains
    [eps_xx, eps_yy, gamma_xy] from DOFs ordered [u1, v1, u2, v2, ...].

    dN_dx : (n_nodes, 2) physical shape-function derivatives.
    Returns B (3, 2*n_nodes).
    """
    n = dN_dx.shape[0]
    B = np.zeros((3, 2 * n))
    for a in range(n):
        B[0, 2 * a] = dN_dx[a, 0]
        B[1, 2 * a + 1] = dN_dx[a, 1]
        B[2, 2 * a] = dN_dx[a, 1]
        B[2, 2 * a + 1] = dN_dx[a, 0]
    return B


def b_matrix_3d(dN_dx: np.ndarray) -> np.ndarray:
    """3D strain-displacement matrix, engineering (Voigt) strains
    [eps_xx, eps_yy, eps_zz, gamma_xy, gamma_yz, gamma_zx] (the solid3d
    NDMaterial order) from DOFs ordered [u1, v1, w1, u2, v2, w2, ...].

    dN_dx : (n_nodes, 3) physical shape-function derivatives.
    Returns B (6, 3*n_nodes).
    """
    n = dN_dx.shape[0]
    B = np.zeros((6, 3 * n))
    for a in range(n):
        dx, dy, dz = dN_dx[a]
        B[0, 3 * a] = dx
        B[1, 3 * a + 1] = dy
        B[2, 3 * a + 2] = dz
        B[3, 3 * a] = dy
        B[3, 3 * a + 1] = dx
        B[4, 3 * a + 1] = dz
        B[4, 3 * a + 2] = dy
        B[5, 3 * a] = dz
        B[5, 3 * a + 2] = dx
    return B
