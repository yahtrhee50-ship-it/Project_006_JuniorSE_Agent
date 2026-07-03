"""
DOF_Numberer — defines the deterministic node ordering used to assign
global equation numbers.

Equation numbers must never depend on dict/set iteration order; the numberer
is the single place ordering is decided (guiding principle #6 of the plan).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

from src.fea.domain import Domain


class DOF_Numberer(ABC):
    @abstractmethod
    def node_order(self, domain: Domain) -> List[int]:
        """Return node tags in the order their DOFs get equation numbers."""


class PlainNumberer(DOF_Numberer):
    """Numbers node DOFs in ascending node-tag order."""

    def node_order(self, domain: Domain) -> List[int]:
        return sorted(domain.node_tags)


# RCM (reverse Cuthill-McKee) bandwidth-reducing numberer: Phase 1.
