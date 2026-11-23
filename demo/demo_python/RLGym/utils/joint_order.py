from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import torch


GO2_RL_GYM_MODEL_JOINT_NAMES = [
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
]


@dataclass
class JointOrderAdapter:
    mode: str
    enabled: bool
    env_to_policy_idx: torch.Tensor | None
    policy_to_env_idx: torch.Tensor | None

    @classmethod
    def from_env(cls, env_joint_names, device, mode: str = "none"):
        mode = str(mode or "none").strip().lower()
        if mode in {"", "none", "off"}:
            return cls(mode="none", enabled=False, env_to_policy_idx=None, policy_to_env_idx=None)

        if mode != "go2_rl_gym":
            raise ValueError(f"Unsupported joint-order compatibility mode: {mode}")

        env_joint_names = list(env_joint_names)
        model_joint_names = list(GO2_RL_GYM_MODEL_JOINT_NAMES)
        if sorted(env_joint_names) != sorted(model_joint_names):
            raise ValueError(
                "Current env joint names do not match go2_rl_gym mapping preset.\n"
                f"env_joint_names={env_joint_names}"
            )

        env_to_policy_idx = torch.tensor(
            [env_joint_names.index(name) for name in model_joint_names],
            dtype=torch.long,
            device=device,
        )
        policy_to_env_idx = torch.tensor(
            [model_joint_names.index(name) for name in env_joint_names],
            dtype=torch.long,
            device=device,
        )
        return cls(
            mode=mode,
            enabled=True,
            env_to_policy_idx=env_to_policy_idx,
            policy_to_env_idx=policy_to_env_idx,
        )

    @classmethod
    def from_privileged_obs_dim(cls, env_joint_names, device, privileged_obs_dim: int | None):
        mode = infer_joint_order_mode_from_privileged_obs(privileged_obs_dim)
        return cls.from_env(env_joint_names, device=device, mode=mode)

    def _reorder_joint_block(self, block: torch.Tensor, index: torch.Tensor | None) -> torch.Tensor:
        if not self.enabled or index is None:
            return block
        return block.index_select(1, index)

    def actor_obs_env_to_policy(self, obs: torch.Tensor, num_actions: int) -> torch.Tensor:
        if not self.enabled:
            return obs
        prefix = 9
        total = prefix + 3 * num_actions
        if obs.shape[1] < total:
            return obs
        out = obs.clone()
        out[:, prefix : prefix + num_actions] = self._reorder_joint_block(
            obs[:, prefix : prefix + num_actions], self.env_to_policy_idx
        )
        out[:, prefix + num_actions : prefix + 2 * num_actions] = self._reorder_joint_block(
            obs[:, prefix + num_actions : prefix + 2 * num_actions], self.env_to_policy_idx
        )
        out[:, prefix + 2 * num_actions : prefix + 3 * num_actions] = self._reorder_joint_block(
            obs[:, prefix + 2 * num_actions : prefix + 3 * num_actions], self.env_to_policy_idx
        )
        return out

    def critic_obs_env_to_policy(self, obs: torch.Tensor | None, num_actions: int) -> torch.Tensor | None:
        if obs is None or not self.enabled:
            return obs
        out = obs.clone()

        actor_prefix = 3 + 9
        actor_total = actor_prefix + 3 * num_actions
        if out.shape[1] >= actor_total:
            out[:, actor_prefix : actor_prefix + num_actions] = self._reorder_joint_block(
                obs[:, actor_prefix : actor_prefix + num_actions], self.env_to_policy_idx
            )
            out[:, actor_prefix + num_actions : actor_prefix + 2 * num_actions] = self._reorder_joint_block(
                obs[:, actor_prefix + num_actions : actor_prefix + 2 * num_actions], self.env_to_policy_idx
            )
            out[:, actor_prefix + 2 * num_actions : actor_prefix + 3 * num_actions] = self._reorder_joint_block(
                obs[:, actor_prefix + 2 * num_actions : actor_prefix + 3 * num_actions], self.env_to_policy_idx
            )

        extras_prefix = 3 + 45 + 4
        extras_total = extras_prefix + 2 * num_actions
        if out.shape[1] >= extras_total:
            out[:, extras_prefix : extras_prefix + num_actions] = self._reorder_joint_block(
                obs[:, extras_prefix : extras_prefix + num_actions], self.env_to_policy_idx
            )
            out[:, extras_prefix + num_actions : extras_prefix + 2 * num_actions] = self._reorder_joint_block(
                obs[:, extras_prefix + num_actions : extras_prefix + 2 * num_actions], self.env_to_policy_idx
            )
        return out

    def actions_policy_to_env(self, actions: torch.Tensor) -> torch.Tensor:
        if not self.enabled:
            return actions
        return self._reorder_joint_block(actions, self.policy_to_env_idx)


def infer_checkpoint_privileged_obs_dim(checkpoint_path: str | Path) -> int | None:
    checkpoint = torch.load(str(checkpoint_path), map_location="cpu")
    model_state = checkpoint.get("model_state_dict", checkpoint)
    critic_weight = model_state.get("critic.0.weight")
    if critic_weight is None:
        return None
    return int(critic_weight.shape[1])


def infer_joint_order_mode_from_privileged_obs(privileged_obs_dim: int | None) -> str:
    if int(privileged_obs_dim or 235) == 263:
        return "go2_rl_gym"
    return "none"
