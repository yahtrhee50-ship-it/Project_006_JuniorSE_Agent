"""
DOF_Numberer — defines the deterministic node ordering used to assign
global equation numbers.

Equation numbers must never depend on dict/set iteration order; the numberer
is the single place ordering is decided (guiding principle #6 of the plan).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from typing import Dict, List, Set

from src.fea.domain import Domain


class DOF_Numberer(ABC):
    @abstractmethod
    def node_order(self, domain: Domain) -> List[int]:
        """Return node tags in the order their DOFs get equation numbers."""


class PlainNumberer(DOF_Numberer):
    """Numbers node DOFs in ascending node-tag order."""

    def node_order(self, domain: Domain) -> List[int]:
        return sorted(domain.node_tags)


class RCMNumberer(DOF_Numberer):
    """Reverse Cuthill-McKee bandwidth-reducing ordering.

    Builds the node adjacency graph from element connectivity, runs
    Cuthill-McKee breadth-first from a lowest-degree start node per
    connected component, then reverses. All ties break on ascending node
    tag, so the ordering is fully deterministic (principle #6). Nodes not
    attached to any element are appended last in tag order.
    """

    def node_order(self, domain: Domain) -> List[int]:
        adj: Dict[int, Set[int]] = {tag: set() for tag in domain.node_tags}
        for elem in domain.elements():
            tags = [int(t) for t in elem.node_tags]
            for a in tags:
                for b in tags:
                    if a != b:
                        adj[a].add(b)

        degree = {t: len(nbrs) for t, nbrs in adj.items()}
        connected = sorted(t for t, d in degree.items() if d > 0)
        isolated = sorted(t for t, d in degree.items() if d == 0)

        order: List[int] = []
        visited: Set[int] = set()
        remaining = set(connected)
        while remaining:
            start = min(remaining, key=lambda t: (degree[t], t))
            queue = deque([start])
            visited.add(start)
            while queue:
                current = queue.popleft()
                order.append(current)
                for nbr in sorted(adj[current] - visited,
                                  key=lambda t: (degree[t], t)):
                    visited.add(nbr)
                    queue.append(nbr)
            remaining -= visited

        order.reverse()
        return order + isolated
