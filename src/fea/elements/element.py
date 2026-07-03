"""
Element — one of the four core contracts of the engine.

An element must return its resisting force and tangent stiffness for the
*current trial displacements* of its nodes, and must be able to commit /
revert its state. The tangent must be the true derivative of the resisting
force (verified by the finite-difference check in src/fea/testing.py).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Sequence, Tuple

import numpy as np


class Element(ABC):
    """Base class for all elements.

    Subclasses set `self.node_tags` in __init__ and receive resolved Node
    references in `set_domain` (stored in `self.nodes`).
    """

    def __init__(self, tag: int, node_tags: Sequence[int]) -> None:
        self.tag = int(tag)
        self.node_tags: Tuple[int, ...] = tuple(int(t) for t in node_tags)
        self.nodes: List = []

    def set_domain(self, domain) -> None:
        """Resolve node tags to Node objects. Called by Domain.add_element."""
        self.nodes = [domain.get_node(t) for t in self.node_tags]

    @abstractmethod
    def get_node_dofs(self) -> Sequence[Sequence[int]]:
        """Local DOF indices this element attaches to at each of its nodes,
        e.g. [(0, 1), (0, 1)] for a 2D truss. Defines the element DOF order
        used by get_tangent_stiff / get_resisting_force."""

    @abstractmethod
    def get_tangent_stiff(self) -> np.ndarray:
        """(n x n) tangent stiffness at the current trial state."""

    @abstractmethod
    def get_resisting_force(self) -> np.ndarray:
        """(n,) resisting force at the current trial state."""

    def commit_state(self) -> None:
        """Accept the current trial state as converged."""

    def revert_to_last_commit(self) -> None:
        """Discard the trial state, return to the last committed state."""

    @property
    def num_dofs(self) -> int:
        return sum(len(d) for d in self.get_node_dofs())
