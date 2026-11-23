"""Offline only: AST-load the adapter; never import/call the real DDS entry.

Controller.__init__ is NEVER called. send_cmd and sleep are replaced on a plain
mock object; no SDK, publisher, subscriber, motion switcher, or sport client is
imported or created. This checker does not prove physical robot safety.
"""
import argparse
import ast
import hashlib
import json
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch

from config_go2_sire import Config

HERE = Path(__file__).resolve().parent
REFERENCE_HASHES = {
    "deploy_go2_moe_cts.py": "293c85b31da3e73bb3a6a9e48df576bb8f086ef5ef35c7eaa389ee83a0f547e7",
    "config_go2_moe_cts.py": "881073f49fb3e11e24a62d544d023a104c219d712265bc93ac7c813e26984b97",
    "configs/go2_moe_cts.yaml": "06716b3fafc91885cd605c7b8027549351ec3906d5598471fceba0461f9c2e08",
    "common/command_helper.py": "a06575599538967d24459645b51ac63aaab498ddb9ee4c268d5fb3b977c9a7df",
    "common/remote_controller.py": "16c9e12aaaa2667426ddb90ef89d124f9d4d4a8a6c8b42196ebfea85ddc0b785",
    "common/rotation_helper.py": "236cdb548d8a69f9abf2ec064ce961c502c02f9ab28500f7c88209925448489d",
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def definitions(path, names):
    tree = ast.parse(path.read_text(), filename=str(path))
    body = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in names]
    if {n.name for n in body} != set(names):
        raise AssertionError(f"Missing definitions in {path}: {names}")
    return ast.Module(body=body, type_ignores=[])


def load_mock_controller():
    namespace = {"np": np, "torch": torch, "time": SimpleNamespace(sleep=lambda _: None),
                 "Config": Config, "LowStateGo": object, "LowCmdGo": object, "struct": struct}
    # Only function/class definitions are compiled. Module imports and __main__
    # are deliberately excluded, so the hardware SDK cannot run accidentally.
    for path, names in ((HERE / "common/rotation_helper.py", ["get_gravity_orientation"]),
                        (HERE / "common/remote_controller.py", ["KeyMap", "RemoteController"]),
                        (HERE / "deploy_go2.py", ["Controller"])):
        exec(compile(definitions(path, names), str(path), "exec"), namespace)
    return namespace["Controller"], namespace["RemoteController"], namespace["KeyMap"]


def check_preserved_reference(reference):
    for name, expected in REFERENCE_HASHES.items():
        if sha(reference / name) != expected:
            raise AssertionError(f"Reference source changed since review: {reference / name}")
        if name.startswith("common/") and sha(HERE / name) != expected:
            raise AssertionError(f"Common helper was changed: {name}")
    original_source = (reference / "deploy_go2_moe_cts.py").read_text()
    original = definitions(reference / "deploy_go2_moe_cts.py", ["Controller", "init_cmd_go2"])
    current = definitions(HERE / "deploy_go2.py", ["Controller", "init_cmd_go2"])
    old_cls = next(n for n in original.body if n.name == "Controller")
    new_cls = next(n for n in current.body if n.name == "Controller")
    old_methods = {n.name: n for n in old_cls.body if isinstance(n, ast.FunctionDef)}
    new_methods = {n.name: n for n in new_cls.body if isinstance(n, ast.FunctionDef)}
    names = ["_warm_up", "wait_for_low_state", "LowStateHandler", "send_cmd", "zero_torque_state",
             "move_to_default_pos", "default_pos_state"]
    for name in names:
        assert ast.dump(old_methods[name]) == ast.dump(new_methods[name]), f"Unexpected lifecycle change: {name}"
    original_init = ast.get_source_segment(original_source, old_methods["__init__"])
    current_source = (HERE / "deploy_go2.py").read_text()
    new_init = ast.get_source_segment(current_source, new_methods["__init__"])
    assert new_init == original_init.replace("torch.jit.load(config.policy_path)", "torch.jit.load(config.policy_path, map_location='cpu').eval()")
    assert ast.dump(next(n for n in original.body if n.name == "init_cmd_go2")) == ast.dump(next(n for n in current.body if n.name == "init_cmd_go2"))
    old_main = next(n for n in ast.parse(original_source).body if isinstance(n, ast.If))
    new_main = next(n for n in ast.parse(current_source).body if isinstance(n, ast.If))
    # Only the config path changes in main. DDS initialization, button sequence,
    # mode release and original select/KeyboardInterrupt damping are identical.
    old_main.body = [n for n in old_main.body if not (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "config_path" for t in n.targets))]
    new_main.body = [n for n in new_main.body if not (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "config_path" for t in n.targets))]
    assert ast.dump(old_main) == ast.dump(new_main), "Main startup/shutdown sequence changed"
    return len(names) + 3


def test_observation_and_motor_output(config, policy):
    Controller, RemoteController, KeyMap = load_mock_controller()
    sends = []
    obj = Controller.__new__(Controller)  # NEVER Controller(config).
    obj.config, obj.counter = config, 0
    obj.use_remote_controller = True
    obj.remote_controller = RemoteController()
    obj.qj = np.zeros(12, dtype=np.float32)
    obj.dqj = np.zeros(12, dtype=np.float32)
    obj.action = np.zeros(12, dtype=np.float32)
    obj.obs = np.zeros(45, dtype=np.float32)
    obj.cmd = np.zeros(3, dtype=np.float32)
    obj.low_cmd = SimpleNamespace(motor_cmd=[SimpleNamespace(q=0., dq=0., kp=0., kd=0., tau=0.) for _ in range(20)])
    obj.low_state = SimpleNamespace(motor_state=[SimpleNamespace(q=0., dq=0.) for _ in range(20)],
                                    imu_state=SimpleNamespace(quaternion=[1., 0., 0., 0.], gyroscope=[0., 0., 0.]))
    obj.send_cmd = sends.append  # Memory only; no DDS object exists.
    captured = []

    def spy(obs):
        captured.append(obs.clone())
        return policy(obs)

    obj.policy = spy
    previous = np.linspace(-0.4, 0.5, 12).astype(np.float32)
    delta = np.linspace(-0.12, 0.13, 12).astype(np.float32)
    velocity = np.linspace(-2., 3., 12).astype(np.float32)
    gyro = np.array([0.2, -0.3, 0.7], dtype=np.float32)
    # Test the actual reference wireless-remote byte offsets and signs, including
    # oversized analog inputs being limited to physical [1,1,1] commands.
    packet = bytearray(40)
    struct.pack_into("H", packet, 2, (1 << KeyMap.start) | (1 << KeyMap.A))
    for offset, value in ((4, -1.4), (8, 1.2), (12, 0.), (20, 1.5)):
        struct.pack_into("f", packet, offset, value)
    obj.remote_controller.set(packet)
    assert obj.remote_controller.button[KeyMap.start] == obj.remote_controller.button[KeyMap.A] == 1
    max_error = 0.0
    with torch.no_grad():
        for yaw in (0., 90., 180., 270., 450., 810.):
            half = np.deg2rad(yaw) * 0.5
            # Yaw * roll gives a non-horizontal gravity vector, preventing an
            # accidental yaw-only test from missing wrong quaternion ordering.
            c, s, cr, sr = np.cos(half), np.sin(half), np.cos(0.15), np.sin(0.15)
            quat = [c * cr, c * sr, s * sr, s * cr]
            obj.low_state.imu_state.quaternion = quat
            obj.low_state.imu_state.gyroscope = gyro.tolist()
            for i, motor in enumerate(config.joint2motor_idx):
                obj.low_state.motor_state[motor].q = float(config.default_angles[i] + delta[i])
                obj.low_state.motor_state[motor].dq = float(velocity[i])
            obj.action = previous.copy()
            obj.run()
            # Body angular velocity must stay unchanged regardless of heading.
            expected_gravity = [0., -np.sin(0.3), -np.cos(0.3)]
            expected = np.concatenate((gyro * 0.25, expected_gravity, [2., 2., -0.25],
                                       delta, velocity * 0.05, previous)).astype(np.float32)
            np.testing.assert_allclose(captured[-1].numpy()[0], expected, rtol=1e-6, atol=2e-7)
            reference_action = np.clip(policy(torch.from_numpy(expected)[None]).numpy()[0], -100., 100.)
            max_error = max(max_error, float(np.max(np.abs(obj.action - reference_action))))
            np.testing.assert_allclose(obj.action, reference_action, rtol=1e-5, atol=1e-5)
            targets = config.default_angles + 0.25 * obj.action
            for i, motor in enumerate(config.joint2motor_idx):
                output = obj.low_cmd.motor_cmd[motor]
                assert output.q == targets[i] and output.dq == 0 and output.tau == 0
                assert output.kp == config.kps[i] and output.kd == config.kds[i]
            assert obj.low_cmd.motor_cmd[12].kp == 0  # Spare slots untouched.
    assert len(sends) == 6
    return max_error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, default=HERE.parent / "deploy_code")
    parser.add_argument("--expected-io", type=Path, default=HERE / "validation/expected_io.npz")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    config = Config(HERE / "configs/go2_sire.yaml")
    preserved = check_preserved_reference(args.reference)
    policy = torch.jit.load(config.policy_path, map_location="cpu").eval()
    assert all(torch.isfinite(p).all().item() for p in policy.parameters())
    assert all(name.startswith("actor.") for name in policy.state_dict())
    with np.load(str(args.expected_io), allow_pickle=False) as fixture:
        with torch.no_grad():
            actual = policy(torch.from_numpy(fixture["observations"])).numpy()
        expected = fixture["actions"]
        np.testing.assert_allclose(actual, expected, rtol=1e-5, atol=1e-5)
        cross_platform_error = float(np.max(np.abs(actual - expected)))
    adapter_error = test_observation_and_motor_output(config, policy)
    assert not any(name.startswith(("unitree_sdk2py", "cyclonedds")) for name in sys.modules), "Hardware SDK was unexpectedly imported"
    warning = ("Reference common/command_helper.py writes qd; installed Go2 MotorCmd defines dq. "
               "Helpers intentionally preserved byte-for-byte as requested. This check does not certify zero/damping delivery, "
               "link-loss behavior or physical safety. No hardware communication was performed.")
    result = {"status": "offline_checks_passed", "python": sys.version, "torch": torch.__version__,
              "policy_sha256": sha(Path(config.policy_path)), "preserved_lifecycle_checks": preserved,
              "jit_cross_platform_max_error": cross_platform_error, "adapter_max_error": adapter_error,
              "command_max": config.max_cmd.tolist(), "command_scale": config.command_scale,
              "joint2motor_idx": config.joint2motor_idx, "dds_imported": False, "hardware_commands_sent": 0,
              "warnings": [warning]}
    print(json.dumps(result, indent=2), flush=True)
    if args.report:
        with args.report.open("x") as file:
            json.dump(result, file, indent=2)


if __name__ == "__main__":
    main()
