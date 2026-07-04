"""
Phase 2 step 1 — MP constraints (transformation handler) + native skew
nodal axes.

Covers the plan's verification list:
- equalDOF splice of two disconnected halves == single-mesh answer
- rigid link (beam) == Frame rigid end offset, same model, same answer
- story constraint on a 2-column portal: shear splits by stiffness
- native skew roller reproduces Hibbeler 14-15/16 (no rotated-model
  workaround), matching the published answers and PlaneTruss oracle
- consistency: reduced residual ~ 0 after solve, K_hat symmetric
- hygiene: double-slave / slave+SP / chains / skew-on-slave / PlainHandler
  with MPs all raise
"""
import numpy as np
import pytest

from src.calcs.truss_stiffness import PlaneTruss
from src.fea import Model
from src.fea.analysis_model import AnalysisModel
from src.fea.constraints.mp import MP_Constraint
from src.fea.constraints.sp import PlainHandler
from src.fea.numberer import PlainNumberer

E, A, I = 29000.0, 10.0, 300.0   # ksi, in2, in4 (imperial, consistent in/kip)


# ---------------------------------------------------------------------------
# equalDOF
# ---------------------------------------------------------------------------

def test_equal_dof_splice_matches_merged_mesh():
    """Axial chain split into two disconnected halves, glued by equalDOF at
    the coincident interface nodes, must match the merged single mesh."""
    L1, L2, P = 40.0, 60.0, 12.0

    def merged():
        m = Model(ndm=2, ndf=2)
        m.node(1, 0.0, 0.0)
        m.node(2, L1, 0.0)
        m.node(3, L1 + L2, 0.0)
        m.fix(1, 1, 1)
        m.fix(2, 0, 1)
        m.fix(3, 0, 1)
        m.uniaxial_material("Elastic", 1, E=E)
        m.element("Truss", 1, 1, 2, A=A, mat=1)
        m.element("Truss", 2, 2, 3, A=A, mat=1)
        m.pattern("Plain", 1, "Linear")
        m.load(3, P, 0.0)
        m.analyze(1)
        return m

    def spliced():
        m = Model(ndm=2, ndf=2)
        m.node(1, 0.0, 0.0)
        m.node(2, L1, 0.0)          # end of half A
        m.node(3, L1, 0.0, ndf=2)   # start of half B (coincident, separate)
        m.node(4, L1 + L2, 0.0)
        m.fix(1, 1, 1)
        m.fix(2, 0, 1)
        m.fix(4, 0, 1)
        m.uniaxial_material("Elastic", 1, E=E)
        m.element("Truss", 1, 1, 2, A=A, mat=1)
        m.element("Truss", 2, 3, 4, A=A, mat=1)
        m.equal_dof(2, 3, 0, 1)     # glue node 3 to node 2 (both DOFs)
        m.pattern("Plain", 1, "Linear")
        m.load(4, P, 0.0)
        m.analyze(1)
        return m

    ma, mb = merged(), spliced()
    # tip displacement: PL1/AE + PL2/AE, machine precision both ways
    tip = P * L1 / (A * E) + P * L2 / (A * E)
    assert np.isclose(mb.node_disp(4, 0), tip, rtol=1e-12)
    assert np.isclose(mb.node_disp(4, 0), ma.node_disp(3, 0), rtol=1e-12)
    # slaved node tracks its retained node exactly
    assert np.allclose(mb.node_disp(3), mb.node_disp(2), rtol=1e-12)
    # support reaction unchanged
    assert np.isclose(mb.node_reaction(1, 0), -P, rtol=1e-12)


def test_load_on_slaved_node_transfers_to_retained_equations():
    """Same splice, but the load lands on the slaved interface node — it
    must transfer through T^T to the retained equations."""
    L1, P = 40.0, 12.0
    m = Model(ndm=2, ndf=2)
    m.node(1, 0.0, 0.0)
    m.node(2, L1, 0.0)
    m.node(3, L1, 0.0)
    m.fix(1, 1, 1)
    m.fix(2, 0, 1)
    m.uniaxial_material("Elastic", 1, E=E)
    m.element("Truss", 1, 1, 2, A=A, mat=1)
    m.equal_dof(2, 3, 0, 1)
    m.pattern("Plain", 1, "Linear")
    m.load(3, P, 0.0)            # load applied to the slave
    m.analyze(1)
    assert np.isclose(m.node_disp(2, 0), P * L1 / (A * E), rtol=1e-12)
    assert np.isclose(m.node_reaction(1, 0), -P, rtol=1e-12)


# ---------------------------------------------------------------------------
# rigid link vs Frame rigid end offset
# ---------------------------------------------------------------------------

def test_rigid_link_beam_matches_frame_offset():
    """Cantilever with a rigid zone at the loaded tip, modeled two ways:
    (a) Frame offset_j, (b) explicit tip node slaved by rigid_link('beam')
    to the flexible end. Same physics, same answer."""
    L, b, P = 100.0, 15.0, 5.0
    Lf = L - b

    # (a) offset formulation (Phase 1, already validated closed-form)
    ma = Model(ndm=2, ndf=3)
    ma.node(1, 0.0, 0.0)
    ma.node(2, L, 0.0)
    ma.fix(1, 1, 1, 1)
    ma.section("Elastic", 1, E=E, A=A, Iz=I)
    ma.element("Frame", 10, 1, 2, section=1, offset_j=(-b, 0.0))
    ma.pattern("Plain", 1, "Linear")
    ma.load(2, 0.0, -P, 0.0)
    ma.analyze(1)

    # (b) rigid link formulation
    mb = Model(ndm=2, ndf=3)
    mb.node(1, 0.0, 0.0)
    mb.node(2, Lf, 0.0)          # flexible end (retained)
    mb.node(3, L, 0.0)           # physical tip (slaved)
    mb.fix(1, 1, 1, 1)
    mb.section("Elastic", 1, E=E, A=A, Iz=I)
    mb.element("Frame", 10, 1, 2, section=1)
    mb.rigid_link("beam", 2, 3)
    mb.pattern("Plain", 1, "Linear")
    mb.load(3, 0.0, -P, 0.0)
    mb.analyze(1)

    assert np.allclose(mb.node_disp(3), ma.node_disp(2), rtol=1e-12)
    assert np.allclose(mb.node_reaction(1), ma.node_reaction(1), rtol=1e-12)
    # closed form regression (same as the Phase 1 offset test)
    v_expected = -P * (Lf**3 / 3.0 + b * Lf**2 + b**2 * Lf) / (E * I)
    assert np.isclose(mb.node_disp(3, 1), v_expected, rtol=1e-9)
    assert np.isclose(abs(mb.node_reaction(1, 2)), P * L, rtol=1e-9)


def test_rigid_link_bar_leaves_rotation_free():
    """'bar' link slaves translations only: the constrained node's rotation
    DOF keeps its own equation (here it is unloaded and unrestrained by any
    element, so it stays zero, while translations follow the lever arm)."""
    m = Model(ndm=2, ndf=3)
    m.node(1, 0.0, 0.0)
    m.node(2, 50.0, 0.0)
    m.node(3, 60.0, 0.0)
    m.fix(1, 1, 1, 1)
    m.fix(3, 0, 0, 1)            # pin the free-floating rotation slot
    m.section("Elastic", 1, E=E, A=A, Iz=I)
    m.element("Frame", 10, 1, 2, section=1)
    m.rigid_link("bar", 2, 3)
    m.pattern("Plain", 1, "Linear")
    m.load(2, 0.0, -5.0, 0.0)
    m.analyze(1)
    u2, u3 = m.node_disp(2), m.node_disp(3)
    dx = 10.0
    assert np.isclose(u3[0], u2[0] - 0.0 * u2[2], rtol=1e-12)
    assert np.isclose(u3[1], u2[1] + dx * u2[2], rtol=1e-12)
    assert u3[2] == 0.0          # own (fixed) DOF, not slaved


# ---------------------------------------------------------------------------
# story constraint (shared ux) on a 2-column portal
# ---------------------------------------------------------------------------

def test_story_equal_dof_splits_shear_by_stiffness():
    H, P = 144.0, 10.0
    I1, I2 = 300.0, 900.0        # col 2 is 3x stiffer
    m = Model(ndm=2, ndf=3)
    m.node(1, 0.0, 0.0)
    m.node(2, 0.0, H)
    m.node(3, 200.0, 0.0)
    m.node(4, 200.0, H)
    m.fix(1, 1, 1, 1)
    m.fix(3, 1, 1, 1)
    m.section("Elastic", 1, E=E, A=A, Iz=I1)
    m.section("Elastic", 2, E=E, A=A, Iz=I2)
    m.element("Frame", 1, 1, 2, section=1)
    m.element("Frame", 2, 3, 4, section=2)
    m.equal_dof(2, 4, 0)         # tops share lateral displacement
    m.pattern("Plain", 1, "Linear")
    m.load(4, P, 0.0, 0.0)       # story shear applied at the slaved node
    m.analyze(1)

    k1 = 3.0 * E * I1 / H**3     # cantilever columns, tops free to rotate
    k2 = 3.0 * E * I2 / H**3
    u = P / (k1 + k2)
    assert np.isclose(m.node_disp(2, 0), u, rtol=1e-9)
    assert np.isclose(m.node_disp(4, 0), u, rtol=1e-12)
    # base shears split in stiffness ratio, and satisfy statics
    R1x, R3x = m.node_reaction(1, 0), m.node_reaction(3, 0)
    assert np.isclose(R1x, -P * k1 / (k1 + k2), rtol=1e-9)
    assert np.isclose(R3x, -P * k2 / (k1 + k2), rtol=1e-9)
    assert np.isclose(R1x + R3x, -P, rtol=1e-12)
    # base moments = shear * H (free top rotation)
    assert np.isclose(abs(m.node_reaction(1, 2)), abs(R1x) * H, rtol=1e-9)


# ---------------------------------------------------------------------------
# native skew nodal axes — Hibbeler 14-15/14-16 (units: N, m, EA = 1)
# ---------------------------------------------------------------------------

class TestSkewRollerCH14M6:
    def setup_method(self):
        m = Model(ndm=2, ndf=2)
        m.node(1, 0.0, 0.0, axes=45.0)   # inclined roller, normal at 45 deg
        m.node(2, 0.0, 4.0)
        m.node(3, 3.0, 4.0)
        m.fix(1, 1, 0)                   # fix the rotated (normal) DOF
        m.fix(2, 1, 1)                   # pin
        m.uniaxial_material("Elastic", 1, E=1.0)
        m.element("Truss", 1, 1, 2, A=1.0, mat=1)
        m.element("Truss", 2, 1, 3, A=1.0, mat=1)
        m.element("Truss", 3, 2, 3, A=1.0, mat=1)
        m.pattern("Plain", 1, "Linear")
        m.load(3, 0.0, -3000.0)
        m.analyze(1)
        self.m = m

        t = PlaneTruss()                 # oracle: calcs skew solve
        t.add_node("N1", 0, 0, skew_angle_deg=45.0)
        t.add_node("N2", 0, 4, ux_fixed=True, uy_fixed=True)
        t.add_node("N3", 3, 4)
        t.add_member("M1", "N1", "N2", A=1.0, E=1.0)
        t.add_member("M2", "N1", "N3", A=1.0, E=1.0)
        t.add_member("M3", "N2", "N3", A=1.0, E=1.0)
        t.add_load("N3", Fx=0.0, Fy=-3000.0)
        self.res = t.solve()

    def test_published_answers(self):
        assert abs(self.m.node_disp(3, 1) - (-29250.0)) < 2.0
        Rx, Ry = self.m.node_reaction(2)
        assert abs(Rx - (-2250.0)) < 5.0
        assert abs(Ry - 750.0) < 5.0
        R1 = self.m.node_reaction(1)
        assert abs(float(np.hypot(*R1)) - 3182.0) < 5.0
        assert abs(R1[0] - R1[1]) < 1e-6   # purely normal to the incline

    def test_reaction_in_local_axes(self):
        """Rotated reaction: all normal (local x), zero along the incline."""
        Rn, Rt = self.m.node_reaction_local(1)
        assert abs(Rn - float(np.hypot(*self.m.node_reaction(1)))) < 1e-6
        assert abs(Rt) < 1e-6

    def test_regression_vs_planetruss(self):
        for tag in (1, 2, 3):
            assert np.allclose(self.m.node_disp(tag),
                               self.res.displacements[f"N{tag}"],
                               rtol=1e-9, atol=1e-9)
            assert np.allclose(self.m.node_reaction(tag),
                               self.res.reactions[f"N{tag}"],
                               rtol=1e-9, atol=1e-9)
        for tag in (1, 2, 3):
            assert np.isclose(self.m.ele_response(tag, "axial"),
                              self.res.member_forces[f"M{tag}"],
                              rtol=1e-9, atol=1e-9)

    def test_statics(self):
        total = sum(self.m.node_reaction(t) for t in (1, 2, 3))
        assert abs(total[0]) < 1e-6
        assert abs(total[1] - 3000.0) < 1e-6


# ---------------------------------------------------------------------------
# consistency of the transformed system
# ---------------------------------------------------------------------------

def _skew_mp_model():
    """Small frame with both a skew support and an MP constraint."""
    m = Model(ndm=2, ndf=3)
    m.node(1, 0.0, 0.0, axes=30.0)
    m.node(2, 100.0, 0.0)
    m.node(3, 100.0, 0.0)
    m.node(4, 160.0, 0.0)
    m.fix(1, 1, 1, 0)            # pin in the rotated frame
    m.fix(4, 0, 1, 0)
    m.section("Elastic", 1, E=E, A=A, Iz=I)
    m.element("Frame", 1, 1, 2, section=1)
    m.element("Frame", 2, 3, 4, section=1)
    m.equal_dof(2, 3, 0, 1, 2)   # moment splice
    m.pattern("Plain", 1, "Linear")
    m.load(2, 0.0, -8.0, 0.0)
    m.ele_load(2, wy=-0.1)
    return m


def test_transformed_tangent_symmetric_and_residual_zero():
    from src.fea.system.soe import LinearSOE
    from src.fea.analysis.integrator import LoadControl

    m = _skew_mp_model()
    m.analyze(1)

    # reuse the analysis model that solved (its accumulated u_hat is the
    # solution); a fresh handler pass would zero the free-equation state
    model = m._analysis_model
    soe = LinearSOE()
    soe.set_size(model.num_eq)
    integ = LoadControl(0.0)     # keep time (load factor) as solved
    integ.form_tangent(m.domain, model, soe)
    assert np.allclose(soe.A, soe.A.T, atol=1e-9)
    # reduced residual  T^T (F_ext - F_int)  ~ 0 at the solved state
    integ.new_step(m.domain, model)
    integ.form_unbalance(m.domain, model, soe)
    scale = max(1.0, float(np.max(np.abs(soe.A))))
    assert np.max(np.abs(soe.b)) < 1e-9 * scale


def test_statics_gate_skew_mp_model():
    m = _skew_mp_model()
    m.analyze(1)
    total = sum(m.node_reaction(t) for t in (1, 2, 3, 4))
    # applied: -8 at node 2 plus w*60 element load
    assert abs(total[0]) < 1e-9
    assert abs(total[1] - (8.0 + 0.1 * 60.0)) < 1e-9


# ---------------------------------------------------------------------------
# hygiene — every misuse must raise
# ---------------------------------------------------------------------------

def _two_node_domain(ndf=2):
    m = Model(ndm=2, ndf=ndf)
    m.node(1, 0.0, 0.0)
    m.node(2, 10.0, 0.0)
    m.node(3, 20.0, 0.0)
    return m


def test_double_slave_raises():
    m = _two_node_domain()
    m.equal_dof(1, 2, 0)
    m.equal_dof(3, 2, 0)
    m.fix(1, 1, 1)
    with pytest.raises(ValueError, match="more than one"):
        m.analyze(1)


def test_slave_plus_sp_raises():
    m = _two_node_domain()
    m.equal_dof(1, 2, 0)
    m.fix(2, 1, 0)
    m.fix(1, 1, 1)
    with pytest.raises(ValueError, match="slaved .* and SP"):
        m.analyze(1)


def test_chain_raises():
    m = _two_node_domain()
    m.equal_dof(1, 2, 0)
    m.equal_dof(2, 3, 0)         # 2 is retained here but slaved above
    m.fix(1, 1, 1)
    with pytest.raises(ValueError, match="chains"):
        m.analyze(1)


def test_skew_on_slaved_node_raises():
    m = Model(ndm=2, ndf=2)
    m.node(1, 0.0, 0.0)
    m.node(2, 10.0, 0.0, axes=30.0)
    m.equal_dof(1, 2, 0)
    m.fix(1, 1, 1)
    with pytest.raises(ValueError, match="skew"):
        m.analyze(1)


def test_plain_handler_rejects_mps():
    m = _two_node_domain()
    m.equal_dof(1, 2, 0)
    model = AnalysisModel()
    with pytest.raises(ValueError, match="TransformationHandler"):
        PlainHandler().handle(m.domain, model, PlainNumberer())


def test_mp_matrix_shape_validated():
    with pytest.raises(ValueError, match="shape"):
        MP_Constraint(1, 2, (0, 1), (0,), np.eye(2))
