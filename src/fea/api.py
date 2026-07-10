"""
Tier-1 imperative scripting API (OpenSeesPy-style verbs, held on a Model
object rather than global state).

Example — 2-bar axial chain, tip load::

    from src.fea import Model

    m = Model(ndm=2, ndf=2)
    m.node(1, 0.0, 0.0)
    m.node(2, 60.0, 0.0)
    m.fix(1, 1, 1)
    m.fix(2, 0, 1)
    m.uniaxial_material("Elastic", 1, E=29000.0)
    m.element("Truss", 1, 1, 2, A=2.0, mat=1)
    m.pattern("Plain", 1, "Linear")
    m.load(2, 10.0, 0.0)
    m.analyze(1)
    ux = m.node_disp(2, 0)      # = P*L/(A*E)
    R = m.node_reaction(1)      # = [-10, 0]

Units: any one consistent set in, same set out.
"""
from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from src.fea.analysis_model import AnalysisModel
from src.fea.domain import Domain, Node
from src.fea.elements.frame import Frame
from src.fea.elements.frame3d import Frame3D
from src.fea.elements.quad4 import Quad4
from src.fea.elements.shell_mitc4 import ShellMITC4
from src.fea.elements.truss import Truss
from src.fea.loads.pattern import (ConstantTimeSeries, FramePointLoad,
                                   FramePointLoad3D, FrameUniformLoad,
                                   FrameUniformLoad3D, LinearTimeSeries,
                                   LoadPattern, NodalLoad, QuadBodyLoad,
                                   QuadEdgeLoad, ShellBodyLoad,
                                   ShellPressureLoad, TimeSeries,
                                   TrussStrainLoad)
from src.fea.materials.nd import ElasticIsotropic, NDMaterial
from src.fea.materials.uniaxial import ElasticMaterial, UniaxialMaterial
from src.fea.numberer import PlainNumberer, RCMNumberer
from src.fea.sections.section import ElasticSection, Section
from src.fea import combos as _combos
from src.fea.system.soe import DenseSolver, LinearSOE, SparseSolver
from src.fea.analysis.static_analysis import StaticAnalysis
from src.fea.constraints.sp import SP_Constraint
from src.fea.constraints.mp import (TransformationHandler, equal_dof,
                                    rigid_link)
from src.fea import mesh as _mesh


class Model:
    """One structural model plus registries and analysis defaults."""

    def __init__(self, ndm: int = 2, ndf: int = 2) -> None:
        if ndm not in (1, 2, 3):
            raise ValueError("ndm must be 1, 2 or 3")
        self.ndm = ndm
        self.ndf = ndf
        self.domain = Domain()
        self._materials: Dict[int, UniaxialMaterial] = {}
        self._nd_materials: Dict[int, NDMaterial] = {}
        self._sections: Dict[int, Section] = {}
        self._current_pattern: Optional[LoadPattern] = None
        self._solver_name = "Dense"
        self._numberer_name = "Plain"
        self._analysis_model: Optional[AnalysisModel] = None

    # -- model building ----------------------------------------------------

    def node(self, tag: int, *coords: float, ndf: Optional[int] = None,
             axes=None) -> None:
        """axes: skew nodal axes — an angle in DEGREES (2D: rotates the
        ux/uy pair CCW from global x; rotational DOFs unchanged) or a full
        ndf x ndf rotation matrix. Equations and fix() flags at this node
        then live in the rotated frame (e.g. inclined roller = fix the
        rotated normal DOF)."""
        if len(coords) != self.ndm:
            raise ValueError(f"node {tag}: expected {self.ndm} coordinates")
        n_dof = ndf if ndf else self.ndf
        if axes is not None and np.isscalar(axes):
            if self.ndm != 2:
                raise NotImplementedError(
                    "angle-form skew axes are 2D only; pass a full matrix")
            a = np.radians(float(axes))
            c, s = np.cos(a), np.sin(a)
            R = np.eye(n_dof)
            R[:2, :2] = [[c, -s], [s, c]]
            axes = R
        self.domain.add_node(Node(tag, coords, n_dof, axes=axes))

    def fix(self, node_tag: int, *flags: int) -> None:
        """fix(tag, 1, 1, ...) — one 0/1 flag per node DOF (1 = fixed)."""
        node = self.domain.get_node(node_tag)
        if len(flags) != node.ndf:
            raise ValueError(
                f"fix({node_tag}): expected {node.ndf} flags, got {len(flags)}")
        for dof, flag in enumerate(flags):
            if flag:
                self.domain.add_sp_constraint(SP_Constraint(node_tag, dof, 0.0))

    def sp(self, node_tag: int, dof: int, value: float) -> None:
        """Prescribe a support displacement (settlement) on one DOF."""
        self.domain.add_sp_constraint(SP_Constraint(node_tag, dof, value))

    def equal_dof(self, retained_tag: int, constrained_tag: int,
                  *dofs: int) -> None:
        """Slave the given DOFs of `constrained_tag` to the same DOFs of
        `retained_tag` (identity MP coupling)."""
        if not dofs:
            raise ValueError("equal_dof: give at least one dof index")
        self.domain.add_mp_constraint(
            equal_dof(retained_tag, constrained_tag, dofs))

    def rigid_link(self, kind: str, retained_tag: int,
                   constrained_tag: int) -> None:
        """Rigid link ('beam' = translations + rotation, 'bar' =
        translations only) slaving `constrained_tag` to `retained_tag`."""
        self.domain.add_mp_constraint(
            rigid_link(kind, self.domain.get_node(retained_tag),
                       self.domain.get_node(constrained_tag)))

    def uniaxial_material(self, kind: str, tag: int, **props) -> None:
        if kind == "Elastic":
            self._materials[tag] = ElasticMaterial(tag, E=props["E"])
        else:
            raise ValueError(f"Unknown uniaxial material '{kind}'")

    def nd_material(self, kind: str, tag: int, **props) -> None:
        """Multi-dimensional material, e.g.
        nd_material("ElasticIsotropic", 1, E=29000.0, nu=0.3,
        formulation="plane_stress")."""
        if kind == "ElasticIsotropic":
            self._nd_materials[tag] = ElasticIsotropic(
                tag, E=props["E"], nu=props["nu"],
                formulation=props.get("formulation", "plane_stress"))
        else:
            raise ValueError(f"Unknown nd material '{kind}'")

    def section(self, kind: str, tag: int, **props) -> None:
        if kind == "Elastic":
            self._sections[tag] = ElasticSection(tag, **props)
        else:
            raise ValueError(f"Unknown section '{kind}'")

    def element(self, kind: str, tag: int, *node_tags: int, **props) -> None:
        if kind == "Truss":
            mat = self._materials[props["mat"]]
            elem = Truss(tag, node_tags, A=props["A"], material=mat)
        elif kind == "Frame":
            sec = self._sections[props["section"]]
            elem = Frame(tag, node_tags, section=sec,
                         release_i=props.get("release_i", False),
                         release_j=props.get("release_j", False),
                         offset_i=props.get("offset_i", (0.0, 0.0)),
                         offset_j=props.get("offset_j", (0.0, 0.0)))
        elif kind == "Frame3D":
            sec = self._sections[props["section"]]
            if "vecxz" not in props:
                raise ValueError(
                    f"element Frame3D {tag}: vecxz orientation vector is "
                    f"required (a vector in the local x-z plane, OpenSees "
                    f"geomTransf convention)")
            elem = Frame3D(tag, node_tags, section=sec, vecxz=props["vecxz"],
                           release_i=props.get("release_i", False),
                           release_j=props.get("release_j", False),
                           offset_i=props.get("offset_i", (0.0, 0.0, 0.0)),
                           offset_j=props.get("offset_j", (0.0, 0.0, 0.0)))
        elif kind == "Quad4":
            mat = self._nd_materials[props["mat"]]
            elem = Quad4(tag, node_tags, material=mat, t=props["t"])
        elif kind == "ShellMITC4":
            mat = self._nd_materials[props["mat"]]
            elem = ShellMITC4(
                tag, node_tags, material=mat, t=props["t"],
                drill_alpha=props.get("drill_alpha", 1.0e-3),
                warp_tol=props.get("warp_tol", 0.05))
        else:
            raise ValueError(f"Unknown element '{kind}'")
        self.domain.add_element(elem)

    def pattern(self, kind: str, tag: int, time_series: str = "Linear",
                factor: float = 1.0) -> None:
        if kind != "Plain":
            raise ValueError(f"Unknown pattern '{kind}'")
        ts: TimeSeries = (LinearTimeSeries(factor) if time_series == "Linear"
                          else ConstantTimeSeries(factor))
        p = LoadPattern(tag, ts)
        self.domain.add_load_pattern(p)
        self._current_pattern = p

    def load(self, node_tag: int, *values: float,
             pattern: Optional[int] = None) -> None:
        """Nodal load (one value per node DOF) in the current/named pattern."""
        node = self.domain.get_node(node_tag)
        if len(values) != node.ndf:
            raise ValueError(
                f"load({node_tag}): expected {node.ndf} values, got {len(values)}")
        p = (self.domain.get_load_pattern(pattern) if pattern is not None
             else self._current_pattern)
        if p is None:
            raise ValueError("No load pattern defined — call pattern() first")
        p.add_nodal_load(NodalLoad(node_tag, values))

    def ele_load(self, ele_tag: int, *, delta_L0: float = 0.0,
                 delta_T: float = 0.0, alpha: float = 0.0,
                 wx: float = 0.0, wy: float = 0.0, wz: float = 0.0,
                 Px: float = 0.0, Py: float = 0.0, Pz: float = 0.0,
                 distance_from_i: Optional[float] = None,
                 bx: float = 0.0, by: float = 0.0, bz: float = 0.0,
                 edge: Optional[int] = None,
                 tx: float = 0.0, ty: float = 0.0, pressure: float = 0.0,
                 pattern: Optional[int] = None) -> None:
        """Truss initial-strain load (delta_L0 / delta_T*alpha), Frame
        load (uniform wx/wy, or point Px/Py at distance_from_i; Frame3D
        adds wz/Pz), Quad4 load (body force bx/by per unit volume, or
        edge traction on local edge index `edge`: global tx/ty per unit
        area and/or `pressure` along the outward normal, + = pushing
        inward), or ShellMITC4 load (`pressure` per unit area along the
        LOCAL +z normal, + = along the normal, and/or body force
        bx/by/bz per unit volume, e.g. self-weight bz=-unit_weight) —
        nodal quantities in GLOBAL components."""
        elem = self.domain.get_element(ele_tag)
        p = (self.domain.get_load_pattern(pattern) if pattern is not None
             else self._current_pattern)
        if p is None:
            raise ValueError("No load pattern defined — call pattern() first")
        if isinstance(elem, ShellMITC4):
            if pressure:
                p.add_element_load(ShellPressureLoad(ele_tag, p=pressure))
            if bx or by or bz:
                p.add_element_load(ShellBodyLoad(ele_tag, bx=bx, by=by, bz=bz))
            if not pressure and not (bx or by or bz):
                raise ValueError(
                    f"ele_load({ele_tag}): a ShellMITC4 takes `pressure` "
                    f"and/or a bx/by/bz body force")
        elif isinstance(elem, Frame3D):
            if distance_from_i is not None:
                p.add_element_load(FramePointLoad3D(
                    ele_tag, Px=Px, Py=Py, Pz=Pz,
                    distance_from_i=distance_from_i))
            else:
                p.add_element_load(FrameUniformLoad3D(ele_tag, wx=wx, wy=wy,
                                                      wz=wz))
        elif isinstance(elem, Frame):
            if wz or Pz:
                raise ValueError(
                    f"ele_load({ele_tag}): wz/Pz need a Frame3D element "
                    f"(this is a 2D Frame)")
            if distance_from_i is not None:
                p.add_element_load(FramePointLoad(
                    ele_tag, Px=Px, Py=Py, distance_from_i=distance_from_i))
            else:
                p.add_element_load(FrameUniformLoad(ele_tag, wx=wx, wy=wy))
        elif isinstance(elem, Quad4):
            if edge is not None:
                p.add_element_load(QuadEdgeLoad(
                    ele_tag, edge=edge, tx=tx, ty=ty, pressure=pressure))
            else:
                p.add_element_load(QuadBodyLoad(ele_tag, bx=bx, by=by))
        else:
            p.add_element_load(TrussStrainLoad(ele_tag, delta_L0=delta_L0,
                                               delta_T=delta_T, alpha=alpha))

    # -- meshing (Phase 2 step 3) --------------------------------------------

    def _emit_mesh(self, mesh: "_mesh.Mesh", limits: Optional[dict],
                   make_element) -> "_mesh.Mesh":
        """Quality-gate the mesh, THEN emit nodes + elements into the
        domain (a failing mesh adds nothing). Returns the Mesh so callers
        can use its groups/sides for supports and loads."""
        _mesh.check_quality(mesh, **(limits or {}))
        for tag, coords in mesh.nodes.items():
            self.node(tag, *coords)
        for tag, conn in mesh.elements.items():
            make_element(tag, conn)
        return mesh

    def mesh_line(self, p0, p1, n: int, *, section: int,
                  first_node: int = 1, first_ele: int = 1,
                  limits: Optional[dict] = None) -> "_mesh.Mesh":
        """Straight line of n Frame elements (2D 3-DOF nodes)."""
        mesh = _mesh.line_mesh(p0, p1, n, first_node=first_node,
                               first_ele=first_ele)
        return self._emit_mesh(
            mesh, limits,
            lambda tag, conn: self.element("Frame", tag, *conn,
                                           section=section))

    def mesh_rect(self, Lx: float, Ly: float, nx: int, ny: int, *,
                  mat: int, t: float, x0: float = 0.0, y0: float = 0.0,
                  first_node: int = 1, first_ele: int = 1,
                  limits: Optional[dict] = None) -> "_mesh.Mesh":
        """Structured Quad4 grid on an axis-aligned rectangle."""
        mesh = _mesh.rect_grid(Lx, Ly, nx, ny, x0=x0, y0=y0,
                               first_node=first_node, first_ele=first_ele)
        return self._emit_mesh(
            mesh, limits,
            lambda tag, conn: self.element("Quad4", tag, *conn, mat=mat, t=t))

    def mesh_transfinite(self, bottom, right, top, left,
                         n_u: int, n_v: int, *, mat: int, t: float,
                         grade_u=None, grade_v=None,
                         first_node: int = 1, first_ele: int = 1,
                         limits: Optional[dict] = None) -> "_mesh.Mesh":
        """Mapped (Coons) Quad4 patch on four possibly-curved boundary
        edges — see src.fea.mesh.transfinite_patch for conventions."""
        mesh = _mesh.transfinite_patch(bottom, right, top, left, n_u, n_v,
                                       grade_u=grade_u, grade_v=grade_v,
                                       first_node=first_node,
                                       first_ele=first_ele)
        return self._emit_mesh(
            mesh, limits,
            lambda tag, conn: self.element("Quad4", tag, *conn, mat=mat, t=t))

    def stitch_edge(self, coarse_tags, fine_tags, *,
                    dofs=(0, 1), tol: float = 1e-8) -> int:
        """Tie a fine mesh edge to a coarse one along a shared straight
        interface (equal-DOF at coincident nodes, hanging-node MP
        interpolation elsewhere). Returns the number of constraints added.
        Approximation — keep tied interfaces away from regions of
        interest."""
        coords = {t: self.domain.get_node(t).coords
                  for t in list(coarse_tags) + list(fine_tags)}
        cons = _mesh.stitch_edge(coarse_tags, fine_tags, coords,
                                 dofs=dofs, tol=tol)
        for c in cons:
            self.domain.add_mp_constraint(c)
        return len(cons)

    # -- analysis -----------------------------------------------------------

    def system(self, name: str) -> None:
        """'Dense' (numpy, default) or 'Sparse' (scipy, lazy import)."""
        if name not in ("Dense", "Sparse"):
            raise ValueError(f"Unknown system '{name}'")
        self._solver_name = name

    def numberer(self, name: str) -> None:
        """'Plain' (tag order, default) or 'RCM' (bandwidth-reducing)."""
        if name not in ("Plain", "RCM"):
            raise ValueError(f"Unknown numberer '{name}'")
        self._numberer_name = name

    def analyze(self, num_steps: int = 1) -> None:
        solver = SparseSolver() if self._solver_name == "Sparse" else DenseSolver()
        numberer = (RCMNumberer() if self._numberer_name == "RCM"
                    else PlainNumberer())
        handler = (TransformationHandler()
                   if self.domain.has_mp_constraints() else None)
        analysis = StaticAnalysis(self.domain, constraint_handler=handler,
                                  numberer=numberer, soe=LinearSOE(solver))
        self._analysis_model = analysis.analyze(num_steps)

    def analyze_cases(self, cases: Dict[str, "list[int] | int"]
                      ) -> Dict[str, _combos.CaseResults]:
        """Solve each load case (name -> pattern tag(s)) independently and
        return frozen CaseResults per case. Each run starts from a wiped
        response state; the model is left reset with all patterns active.
        Combine with src.fea.combos.combine / envelope (linear analysis)."""
        if not cases:
            raise ValueError("analyze_cases: no cases given")
        results: Dict[str, _combos.CaseResults] = {}
        try:
            for name, tags in cases.items():
                tag_list = [tags] if isinstance(tags, int) else list(tags)
                self.domain.reset()
                self.domain.set_active_patterns(tag_list)
                self.analyze(1)
                results[name] = _combos.snapshot(self.domain, name)
        finally:
            self.domain.set_active_patterns(None)
            self.domain.reset()
        return results

    # -- results --------------------------------------------------------------

    def node_disp(self, tag: int, dof: Optional[int] = None):
        d = self.domain.get_node(tag).get_committed_disp()
        return float(d[dof]) if dof is not None else d

    def node_reaction(self, tag: int, dof: Optional[int] = None):
        r = self.domain.get_node(tag).reaction.copy()
        return float(r[dof]) if dof is not None else r

    def node_reaction_local(self, tag: int, dof: Optional[int] = None):
        """Reaction rotated into the node's skew axes (e.g. the normal
        force of an inclined roller). Identical to node_reaction for a
        node without skew axes."""
        node = self.domain.get_node(tag)
        r = node.reaction.copy()
        if node.axes is not None:
            r = node.axes.T @ r
        return float(r[dof]) if dof is not None else r

    def ele_force(self, tag: int) -> np.ndarray:
        return self.domain.get_element(tag).get_resisting_force()

    def ele_response(self, tag: int, name: str):
        return self.domain.get_element(tag).get_response(name)
