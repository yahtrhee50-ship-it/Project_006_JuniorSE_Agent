"""
Reusable verification helpers (guiding principle #3: consistent tangents).

`check_element_tangent` is the single highest-value test in the engine: it
verifies that an element's tangent stiffness is the true derivative of its
resisting force by central finite differences on the nodes' trial
displacements. Every element (linear or not) must pass it.
"""
from __future__ import annotations

import numpy as np


def finite_difference_tangent(element, h: float = 1e-7) -> np.ndarray:
    """Central-difference d(resisting force)/d(element DOFs), perturbing the
    element's node trial displacements. Restores the original trial state."""
    n = element.num_dofs
    K_fd = np.zeros((n, n))
    col = 0
    for node, dofs in zip(element.nodes, element.get_node_dofs()):
        original = node.get_trial_disp()
        for d in dofs:
            node.set_trial_disp_component(d, original[d] + h)
            f_plus = element.get_resisting_force()
            node.set_trial_disp_component(d, original[d] - h)
            f_minus = element.get_resisting_force()
            node.set_trial_disp_component(d, original[d])
            K_fd[:, col] = (f_plus - f_minus) / (2.0 * h)
            col += 1
    return K_fd


def check_element_tangent(element, h: float = 1e-7,
                          rtol: float = 1e-5, atol: float = 1e-6) -> float:
    """Assert tangent == FD(resisting force); return the max abs error."""
    K = element.get_tangent_stiff()
    K_fd = finite_difference_tangent(element, h)
    scale = max(1.0, float(np.max(np.abs(K))))
    err = float(np.max(np.abs(K - K_fd)))
    if err > atol + rtol * scale:
        raise AssertionError(
            f"Element {element.tag} ({type(element).__name__}): tangent is "
            f"not the derivative of the resisting force "
            f"(max |K - K_fd| = {err:.3e}, allowed {atol + rtol * scale:.3e})")
    return err


def check_material_tangent(material, strain: float, h: float = 1e-7,
                           rtol: float = 1e-5, atol: float = 1e-8) -> float:
    """Assert material tangent == FD(stress) at the given strain; return the
    abs error. Leaves the material at trial strain = `strain`."""
    material.set_trial_strain(strain + h)
    s_plus = material.get_stress()
    material.set_trial_strain(strain - h)
    s_minus = material.get_stress()
    fd = (s_plus - s_minus) / (2.0 * h)
    material.set_trial_strain(strain)
    tangent = material.get_tangent()
    err = abs(tangent - fd)
    if err > atol + rtol * max(1.0, abs(tangent)):
        raise AssertionError(
            f"Material {material.tag} ({type(material).__name__}): tangent "
            f"{tangent:.6e} != FD {fd:.6e} at strain {strain}")
    return err
