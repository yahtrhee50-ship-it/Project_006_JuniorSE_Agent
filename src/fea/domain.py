"""
Domain — tagged storage of everything the structural model *is*.

The Domain never knows how it is solved. Analysis components read from it,
scatter results back into it (node trial displacements), and tell it when to
commit or revert state.
"""
from __future__ import annotations

from typing import Dict, Iterable, List

import numpy as np


class Node:
    """A node: geometry, `ndf` displacement DOFs, trial/committed response.

    Parameters
    ----------
    tag    : unique integer identifier
    coords : spatial coordinates, length = ndm (2 or 3)
    ndf    : number of DOFs carried by this node (2 = 2D truss node,
             3 = 2D frame node, 6 = 3D frame node, ...)
    """

    def __init__(self, tag: int, coords, ndf: int) -> None:
        self.tag = int(tag)
        self.coords = np.asarray(coords, dtype=float)
        self.ndf = int(ndf)
        if self.ndf < 1:
            raise ValueError(f"Node {tag}: ndf must be >= 1")
        self._trial_disp = np.zeros(self.ndf)
        self._committed_disp = np.zeros(self.ndf)
        self.mass = np.zeros(self.ndf)
        self.reaction = np.zeros(self.ndf)

    @property
    def ndm(self) -> int:
        return self.coords.shape[0]

    # -- trial state -----------------------------------------------------

    def get_trial_disp(self) -> np.ndarray:
        return self._trial_disp.copy()

    def set_trial_disp(self, disp) -> None:
        disp = np.asarray(disp, dtype=float)
        if disp.shape != (self.ndf,):
            raise ValueError(f"Node {self.tag}: expected {self.ndf} disp values")
        self._trial_disp = disp.copy()

    def set_trial_disp_component(self, dof: int, value: float) -> None:
        self._trial_disp[dof] = value

    def incr_trial_disp_component(self, dof: int, dvalue: float) -> None:
        self._trial_disp[dof] += dvalue

    # -- committed state -------------------------------------------------

    def get_committed_disp(self) -> np.ndarray:
        return self._committed_disp.copy()

    def commit_state(self) -> None:
        self._committed_disp = self._trial_disp.copy()

    def revert_to_last_commit(self) -> None:
        self._trial_disp = self._committed_disp.copy()

    def __repr__(self) -> str:
        return f"Node(tag={self.tag}, coords={self.coords.tolist()}, ndf={self.ndf})"


class Domain:
    """Container for the structural model, keyed by integer tags.

    Iteration order over nodes/elements is insertion order, but analysis
    components must never rely on it for equation numbering — the
    DOF_Numberer defines the deterministic ordering.
    """

    def __init__(self) -> None:
        self._nodes: Dict[int, Node] = {}
        self._elements: Dict[int, "Element"] = {}
        self._sp_constraints: List["SP_Constraint"] = []
        self._patterns: Dict[int, "LoadPattern"] = {}
        # None = all patterns active; else only these tags load the model.
        self._active_pattern_tags: "List[int] | None" = None
        self.current_time = 0.0

    # -- add / get ---------------------------------------------------------

    def add_node(self, node: Node) -> None:
        if node.tag in self._nodes:
            raise ValueError(f"Duplicate node tag {node.tag}")
        self._nodes[node.tag] = node

    def add_element(self, element) -> None:
        if element.tag in self._elements:
            raise ValueError(f"Duplicate element tag {element.tag}")
        element.set_domain(self)
        self._elements[element.tag] = element

    def add_sp_constraint(self, sp) -> None:
        node = self.get_node(sp.node_tag)
        if not 0 <= sp.dof < node.ndf:
            raise ValueError(
                f"SP constraint on node {sp.node_tag}: dof {sp.dof} out of "
                f"range for ndf={node.ndf}")
        self._sp_constraints.append(sp)

    def add_load_pattern(self, pattern) -> None:
        if pattern.tag in self._patterns:
            raise ValueError(f"Duplicate load pattern tag {pattern.tag}")
        self._patterns[pattern.tag] = pattern

    def get_node(self, tag: int) -> Node:
        try:
            return self._nodes[tag]
        except KeyError:
            raise KeyError(f"No node with tag {tag}") from None

    def get_element(self, tag: int):
        try:
            return self._elements[tag]
        except KeyError:
            raise KeyError(f"No element with tag {tag}") from None

    def get_load_pattern(self, tag: int):
        try:
            return self._patterns[tag]
        except KeyError:
            raise KeyError(f"No load pattern with tag {tag}") from None

    @property
    def node_tags(self) -> List[int]:
        return list(self._nodes.keys())

    @property
    def element_tags(self) -> List[int]:
        return list(self._elements.keys())

    def nodes(self) -> Iterable[Node]:
        return self._nodes.values()

    def elements(self) -> Iterable:
        return self._elements.values()

    def sp_constraints(self) -> Iterable:
        return iter(self._sp_constraints)

    def load_patterns(self) -> Iterable:
        """Active load patterns (all of them unless set_active_patterns
        narrowed the set — used to run one load case at a time)."""
        if self._active_pattern_tags is None:
            return self._patterns.values()
        return [self._patterns[t] for t in self._active_pattern_tags]

    def set_active_patterns(self, tags: "List[int] | None") -> None:
        """Restrict loading to the given pattern tags (None = all active)."""
        if tags is not None:
            missing = [t for t in tags if t not in self._patterns]
            if missing:
                raise KeyError(f"No load pattern with tag(s) {missing}")
            tags = list(tags)
        self._active_pattern_tags = tags

    def reset(self) -> None:
        """Wipe the response state (not the model): zero all node trial and
        committed displacements and reactions, zero element loads, commit the
        zero state, and rewind pseudo-time. Used between load-case runs."""
        for node in self._nodes.values():
            node.set_trial_disp(np.zeros(node.ndf))
            node.reaction = np.zeros(node.ndf)
            node.commit_state()
        for elem in self._elements.values():
            elem.zero_loads()
            elem.commit_state()
        self.current_time = 0.0

    # -- state -------------------------------------------------------------

    def commit(self) -> None:
        for node in self._nodes.values():
            node.commit_state()
        for elem in self._elements.values():
            elem.commit_state()

    def revert_to_last_commit(self) -> None:
        for node in self._nodes.values():
            node.revert_to_last_commit()
        for elem in self._elements.values():
            elem.revert_to_last_commit()
