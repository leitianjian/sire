"""
Modular test framework for LeggedRobotSire.

Each test method is independent — you can run individual tests to verify
specific parts of the Sire migration without running the whole suite.

Usage:
    # run all tests
    python test_sire_methods.py

    # run a single test module
    python test_sire_methods.py --test test_01_env_creation

    # run several
    python test_sire_methods.py --test test_01_env_creation test_05_contact_forces

    # list available tests
    python test_sire_methods.py --list
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from RLGym.envs.base.legged_robot import LeggedRobot
from RLGym.envs.base.legged_robot_sire import LeggedRobotSire
from RLGym.envs.base.legged_robot_config import LeggedRobotCfg


# ═══════════════════════════════════════════════════════════════════════
#  Test runner helpers
# ═══════════════════════════════════════════════════════════════════════
class Result:
    """Simple pass/fail/error result with optional diff info."""
    def __init__(self, name: str):
        self.name   = name
        self.passed = True
        self.errors: list[str] = []
        self.diffs:  list[str] = []
        self.duration_ms = 0.0

    def fail(self, msg: str):
        self.passed = False
        self.errors.append(msg)

    def diff(self, msg: str):
        self.diffs.append(msg)

    def __repr__(self):
        status = "✅" if self.passed else "❌"
        return f"{status} {self.name}  ({self.duration_ms:.1f} ms)"


all_results: list[Result] = []


def _make_envs(cfg=None) -> tuple[LeggedRobot, LeggedRobotSire]:
    """Build both a MuJoCo and a Sire environment for comparison."""
    if cfg is None:
        cfg = LeggedRobotCfg()
        cfg.env.num_envs = 2          # keep small for speed
    env_mj = LeggedRobot(cfg, headless=True)
    env_si = LeggedRobotSire(cfg, headless=True)
    return env_mj, env_si


def _run_test(fn):
    """Decorator-like runner that records timing."""
    global all_results
    r = Result(fn.__name__)
    t0 = time.perf_counter()
    try:
        fn(r)
    except Exception:
        r.fail(traceback.format_exc())
    r.duration_ms = (time.perf_counter() - t0) * 1000
    all_results.append(r)
    print(r)


# ═══════════════════════════════════════════════════════════════════════
#  TEST 01 — Environment creation
# ═══════════════════════════════════════════════════════════════════════
def test_01_env_creation(r: Result):
    """Verify Sire envs are created successfully and have correct counts."""
    _, env_si = _make_envs()

    num = env_si.num_envs
    assert num > 0, "num_envs is zero"
    assert len(env_si.sire_simulators) == num, (
        f"sire_simulators len={len(env_si.sire_simulators)} expected={num}"
    )
    assert len(env_si.sire_models) == num
    assert len(env_si.sire_physics) == num
    assert len(env_si.sire_sim_loops) == num

    # verify each instance is independent (different id())
    ids_sim  = {id(s) for s in env_si.sire_simulators}
    ids_mod  = {id(m) for m in env_si.sire_models}
    assert len(ids_sim) == num, "Simulators are not independent instances"
    assert len(ids_mod) == num, "Models are not independent instances"

    r.diff(f"Created {num} Sire environments successfully")


# ═══════════════════════════════════════════════════════════════════════
#  TEST 02 — Initial state consistency
# ═══════════════════════════════════════════════════════════════════════
def test_02_initial_state(r: Result):
    """Compare initial root state and joint positions between MuJoCo & Sire."""
    env_mj, env_si = _make_envs()

    # MuJoCo
    env_mj._refresh_sim_tensors()
    mj_root  = env_mj.root_states[0].clone()
    mj_dof   = env_mj.dof_pos[0].clone()

    # Sire
    env_si._refresh_sim_tensors_sire()
    si_root  = env_si.root_states[0].clone()
    si_dof   = env_si.dof_pos[0].clone()

    # root position should be within 1 cm
    pos_err = float(torch.norm(mj_root[:3] - si_root[:3]))
    if pos_err > 0.02:
        r.fail(f"Root position discrepancy: {pos_err:.4f} m")

    r.diff(f"Root pos  MJ={mj_root[:3].tolist()}  SI={si_root[:3].tolist()}")
    r.diff(f"Root quat MJ={mj_root[3:7].tolist()}  SI={si_root[3:7].tolist()}")
    r.diff(f"Dof pos[:6] MJ={mj_dof[:6].tolist()}  SI={si_dof[:6].tolist()}")
    r.diff(f"Pos err={pos_err:.6f}, Dof err={float(torch.norm(mj_dof-si_dof)):.6f}")


# ═══════════════════════════════════════════════════════════════════════
#  TEST 03 — Root state reading (step-by-step validation)
# ═══════════════════════════════════════════════════════════════════════
def test_03_root_state_reading(r: Result):
    """After a few Sire steps, root state should be physically plausible."""
    cfg = LeggedRobotCfg()
    cfg.env.num_envs = 1
    _, env_si = _make_envs(cfg)

    # Manually set a known state
    m = env_si.sire_models[0]
    gm = m.generalMotionPool()[0]
    gm.setMpq([0.1, 0.2, 0.5, 1.0, 0.0, 0.0, 0.0])
    gm.setMva([0.5, -0.3, 0.0, 0.0, 0.0, 0.1])
    m.forwardKinematics()

    env_si._refresh_sim_tensors_sire()
    rs = env_si.root_states[0]

    # verify round-trip
    assert abs(float(rs[0]) - 0.1) < 1e-4, f"x mismatch: {rs[0]}"
    assert abs(float(rs[1]) - 0.2) < 1e-4, f"y mismatch: {rs[1]}"
    assert abs(float(rs[2]) - 0.5) < 1e-4, f"z mismatch: {rs[2]}"
    assert abs(float(rs[7]) - 0.5) < 1e-4, f"vx mismatch: {rs[7]}"
    assert abs(float(rs[8]) - (-0.3)) < 1e-4, f"vy mismatch: {rs[8]}"
    r.diff(f"Root state round-trip: OK  pos={rs[:3].tolist()} vel={rs[7:10].tolist()}")


# ═══════════════════════════════════════════════════════════════════════
#  TEST 04 — Joint state reading
# ═══════════════════════════════════════════════════════════════════════
def test_04_joint_state_reading(r: Result):
    """Verify that Motion.mp / Motion.mv round-trip through dof_pos / dof_vel."""
    cfg = LeggedRobotCfg()
    cfg.env.num_envs = 1
    _, env_si = _make_envs(cfg)

    m = env_si.sire_models[0]
    # set known joint values
    test_vals_pos = np.linspace(-0.5, 0.5, env_si.num_dof)
    test_vals_vel = np.linspace(-1.0, 1.0, env_si.num_dof)
    for j in range(env_si.num_dof):
        m.motionPool()[j].mp = float(test_vals_pos[j])
        m.motionPool()[j].mv = float(test_vals_vel[j])
    m.forwardKinematics()

    env_si._refresh_sim_tensors_sire()

    pos_err = float(torch.norm(env_si.dof_pos[0] - torch.tensor(test_vals_pos)))
    vel_err = float(torch.norm(env_si.dof_vel[0] - torch.tensor(test_vals_vel)))
    if pos_err > 1e-4:
        r.fail(f"Joint position readback error: {pos_err:.6f}")
    if vel_err > 1e-4:
        r.fail(f"Joint velocity readback error: {vel_err:.6f}")
    r.diff(f"Joint pos err={pos_err:.6f}, vel err={vel_err:.6f}")


# ═══════════════════════════════════════════════════════════════════════
#  TEST 05 — Contact force reading
# ═══════════════════════════════════════════════════════════════════════
def test_05_contact_forces(r: Result):
    """Verify contact forces are readable and physically plausible."""
    _, env_si = _make_envs()

    # run a few steps with zero action (robot falls → contacts)
    acts = torch.zeros(env_si.num_envs, env_si.num_actions)
    for step_i in range(20):
        env_si.step(acts)
        max_f = float(env_si.contact_forces[0].norm(dim=1).max())
        if max_f > 0:
            break

    cf = env_si.contact_forces[0]
    max_f = float(cf.norm(dim=1).max())

    if max_f <= 0:
        r.fail("No contact forces detected after 20 steps — robot may not be falling")

    # contact forces should be finite
    if torch.isnan(cf).any():
        r.fail("NaN detected in contact forces")
    if torch.isinf(cf).any():
        r.fail("Inf detected in contact forces")

    # check Newton's 3rd law: sum of all contact forces should be ~0
    # (forces on ground + robot should balance in steady state)
    total_f = cf.sum(dim=0)
    r.diff(f"Max contact force: {max_f:.2f} N")
    r.diff(f"Sum of all contact forces (world): {total_f.tolist()}")


# ═══════════════════════════════════════════════════════════════════════
#  TEST 06 — Ground contact detection
# ═══════════════════════════════════════════════════════════════════════
def test_06_ground_contact(r: Result):
    """Verify body_ground_contact is populated when robot touches ground."""
    _, env_si = _make_envs()

    acts = torch.zeros(env_si.num_envs, env_si.num_actions)
    for _ in range(30):
        env_si.step(acts)

    gc = env_si.body_ground_contact[0]
    n_contacts = int(gc.sum())

    if n_contacts == 0:
        r.fail("No ground contacts after 30 steps of free-fall — suspect model issue")
    else:
        contact_bodies = torch.nonzero(gc).squeeze(-1).tolist()
        r.diff(f"Ground-contact bodies ({n_contacts}): {contact_bodies}")

    # foot_ground_contact should be consistent
    fgc = env_si.foot_ground_contact[0]
    r.diff(f"Foot ground contact: {fgc.tolist()}")


# ═══════════════════════════════════════════════════════════════════════
#  TEST 07 — Torque application
# ═══════════════════════════════════════════════════════════════════════
def test_07_torque_application(r: Result):
    """Verify that torques set in forcePool actually change joint accelerations."""
    cfg = LeggedRobotCfg()
    cfg.env.num_envs = 1
    _, env_si = _make_envs(cfg)

    m = env_si.sire_models[0]
    sl = env_si.sire_sim_loops[0]

    # initialise at default pose
    for j in range(env_si.num_dof):
        m.motionPool()[j].mp = float(env_si.default_dof_pos[0, j])
        m.motionPool()[j].mv = 0.0
    m.forwardKinematics()

    # record velocity before
    vel_before = np.array([float(m.motionPool()[j].mv) for j in range(env_si.num_dof)])

    # apply large torque on first joint
    fp = m.forcePool()
    fp[0].fce = 10.0  # 10 N·m
    sl._last_dt_actual = sl.deltaT
    sl.step(1)

    vel_after = np.array([float(m.motionPool()[j].mv) for j in range(env_si.num_dof)])

    # the first joint should have accelerated noticeably
    dv0 = abs(vel_after[0] - vel_before[0])
    if dv0 < 1e-6:
        r.fail("Joint 0 did not accelerate despite 10 N·m torque")

    # other joints should have negligible change (no torque applied)
    dv_others = np.max(np.abs(vel_after[1:] - vel_before[1:]))
    r.diff(f"Joint 0 Δv={dv0:.4f} rad/s,  max other Δv={dv_others:.6f} rad/s")


# ═══════════════════════════════════════════════════════════════════════
#  TEST 08 — Impulse normalisation
# ═══════════════════════════════════════════════════════════════════════
def test_08_impulse_normalisation(r: Result):
    """Verify that force × dt scaling works correctly."""
    _, env_si = _make_envs()

    dt_ctrl = float(env_si.cfg.sim.dt) * env_si.cfg.control.decimation
    m = env_si.sire_models[0]
    p = env_si.sire_physics[0]

    cf_idx = p.contactForceIdx()
    fp = m.forcePool()

    # manually set a known force on part 1
    if cf_idx + 1 < fp.size():
        fp[cf_idx + 1].fce = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # case 1: dt_actual == dt_control → force unchanged
    env_si.sire_sim_loops[0]._last_dt_actual = dt_ctrl
    env_si._refresh_sim_tensors_sire()
    f1 = float(env_si.contact_forces[0, 1, 0])
    if abs(f1 - 100.0) > 0.01:
        r.fail(f"dt_actual==dt_ctrl: expected 100.0, got {f1:.2f}")

    # case 2: dt_actual = 0.1 × dt_control → force should be 10.0
    fp[cf_idx + 1].fce = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    env_si.sire_sim_loops[0]._last_dt_actual = dt_ctrl * 0.1
    env_si._refresh_sim_tensors_sire()
    f2 = float(env_si.contact_forces[0, 1, 0])
    if abs(f2 - 10.0) > 0.01:
        r.fail(f"dt_actual=0.1*dt_ctrl: expected 10.0, got {f2:.2f}")

    r.diff(f"dt_ctrl={dt_ctrl:.6f},  f(dt=1.0x)={f1:.2f}, f(dt=0.1x)={f2:.2f}")


# ═══════════════════════════════════════════════════════════════════════
#  TEST 09 — Reset (dofs + root)
# ═══════════════════════════════════════════════════════════════════════
def test_09_reset(r: Result):
    """Verify that _reset_dofs and _reset_root_states leave a valid state."""
    cfg = LeggedRobotCfg()
    cfg.env.num_envs = 1
    _, env_si = _make_envs(cfg)

    env_ids = torch.tensor([0])

    # run a few steps to drift from initial
    env_si.step(torch.randn(1, env_si.num_actions) * 0.2)
    env_si.step(torch.randn(1, env_si.num_actions) * 0.2)

    # reset
    env_si._reset_dofs(env_ids)
    env_si._reset_root_states(env_ids)

    # read back
    env_si._refresh_sim_tensors_sire()

    # dof_pos should be within joint limits
    for j in range(env_si.num_dof):
        lo = float(env_si.dof_pos_limits[j, 0])
        hi = float(env_si.dof_pos_limits[j, 1])
        v  = float(env_si.dof_pos[0, j])
        if v < lo - 0.01 or v > hi + 0.01:
            r.fail(f"Joint {j} pos {v:.3f} outside limits [{lo:.3f}, {hi:.3f}]")

    # root z should be above ground
    if float(env_si.root_states[0, 2]) < -0.1:
        r.fail(f"Root z={env_si.root_states[0,2]:.3f} below ground after reset")

    r.diff(f"Reset OK: root_z={env_si.root_states[0,2]:.3f}")


# ═══════════════════════════════════════════════════════════════════════
#  TEST 10 — Multi-step stability
# ═══════════════════════════════════════════════════════════════════════
def test_10_stability(r: Result):
    """Run many steps and verify no NaN, no crash, physically bounded."""
    cfg = LeggedRobotCfg()
    cfg.env.num_envs = 1
    _, env_si = _make_envs(cfg)

    pos_z = []
    for step_i in range(100):
        acts = torch.randn(1, env_si.num_actions) * 0.3
        obs, _, _, reset, _ = env_si.step(acts)
        pos_z.append(float(env_si.root_states[0, 2]))

        if np.isnan(obs.numpy()).any():
            r.fail(f"NaN in observations at step {step_i}")
            break
        if reset[0]:
            env_si.reset_idx(torch.tensor([0]))

    min_z, max_z = min(pos_z), max(pos_z)
    if min_z < -10 or max_z > 10:
        r.fail(f"Base height out of bounds: [{min_z:.2f}, {max_z:.2f}]")

    r.diff(f"100 steps OK.  z ∈ [{min_z:.2f}, {max_z:.2f}]")


# ═══════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════
ALL_TESTS = {
    "test_01_env_creation":       test_01_env_creation,
    "test_02_initial_state":      test_02_initial_state,
    "test_03_root_state_reading": test_03_root_state_reading,
    "test_04_joint_state_reading":test_04_joint_state_reading,
    "test_05_contact_forces":     test_05_contact_forces,
    "test_06_ground_contact":     test_06_ground_contact,
    "test_07_torque_application": test_07_torque_application,
    "test_08_impulse_normalisation": test_08_impulse_normalisation,
    "test_09_reset":              test_09_reset,
    "test_10_stability":          test_10_stability,
}


def main():
    ap = argparse.ArgumentParser(description="Modular Sire method tests")
    ap.add_argument("--test", nargs="*", default=None,
                    help="Specific test names to run (space-separated)")
    ap.add_argument("--list", action="store_true",
                    help="List available tests and exit")
    ap.add_argument("--quick", action="store_true",
                    help="Run only fast tests (skip stability)")
    args = ap.parse_args()

    if args.list:
        print("Available tests:")
        for name in ALL_TESTS:
            doc = ALL_TESTS[name].__doc__ or ""
            print(f"  {name:40s} {doc.strip().split(chr(10))[0]}")
        return

    if args.test:
        selected = [n for n in args.test if n in ALL_TESTS]
        unknown  = [n for n in args.test if n not in ALL_TESTS]
        if unknown:
            print(f"Unknown tests: {unknown}")
            return
    elif args.quick:
        selected = [n for n in ALL_TESTS if n != "test_10_stability"]
    else:
        selected = list(ALL_TESTS)

    print(f"Running {len(selected)} test(s)\n")

    for name in selected:
        _run_test(ALL_TESTS[name])

    # summary
    n_pass = sum(1 for r in all_results if r.passed)
    n_fail = len(all_results) - n_pass
    total_ms = sum(r.duration_ms for r in all_results)
    print(f"\n{'='*60}")
    print(f"RESULTS: {n_pass} passed, {n_fail} failed  ({total_ms:.0f} ms total)")

    if n_fail > 0:
        print("\nFAILED TESTS:")
        for r in all_results:
            if not r.passed:
                print(f"  ❌ {r.name}")
                for e in r.errors:
                    for line in e.strip().split("\n"):
                        print(f"       {line}")
    sys.exit(0 if n_fail == 0 else 1)


if __name__ == "__main__":
    main()
