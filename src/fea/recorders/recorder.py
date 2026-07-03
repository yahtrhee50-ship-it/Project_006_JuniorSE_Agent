"""
Recorders — capture response history at each committed step (to memory;
disk output arrives when result sets get large).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Sequence

import numpy as np

from src.fea.domain import Domain


class Recorder(ABC):
    @abstractmethod
    def record(self, domain: Domain, time: float) -> None: ...


class NodeRecorder(Recorder):
    """Records node response ('disp' or 'reaction') per step.

    .times : list of pseudo-times
    .data  : list of rows; each row concatenates the response vectors of the
             requested nodes in the order given.
    """

    def __init__(self, node_tags: Sequence[int], response: str = "disp") -> None:
        if response not in ("disp", "reaction"):
            raise ValueError(f"NodeRecorder: unknown response '{response}'")
        self.node_tags = list(node_tags)
        self.response = response
        self.times: List[float] = []
        self.data: List[np.ndarray] = []

    def record(self, domain: Domain, time: float) -> None:
        row = []
        for tag in self.node_tags:
            node = domain.get_node(tag)
            row.append(node.get_committed_disp() if self.response == "disp"
                       else node.reaction.copy())
        self.times.append(time)
        self.data.append(np.concatenate(row))


class ElementRecorder(Recorder):
    """Records an element response per step. Uses element.get_response(name)
    when available, else falls back to the resisting force vector."""

    def __init__(self, element_tags: Sequence[int], response: str = "force") -> None:
        self.element_tags = list(element_tags)
        self.response = response
        self.times: List[float] = []
        self.data: List[list] = []

    def record(self, domain: Domain, time: float) -> None:
        row = []
        for tag in self.element_tags:
            elem = domain.get_element(tag)
            if hasattr(elem, "get_response"):
                row.append(elem.get_response(self.response))
            else:
                row.append(elem.get_resisting_force())
        self.times.append(time)
        self.data.append(row)
