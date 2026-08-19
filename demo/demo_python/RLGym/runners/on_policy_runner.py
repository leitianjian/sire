from __future__ import annotations

import os
import re
import statistics
import time
from collections import deque
from pathlib import Path

import torch
import yaml
from torch.utils.tensorboard import SummaryWriter

from rsl_rl.algorithms import PPO
from rsl_rl.modules import ActorCritic

from RLGym.runners.infinite_scheduler import InfiniteLevelScheduler
from RLGym.utils.helpers import class_to_dict
from RLGym.utils.joint_order import JointOrderAdapter


class OnPolicyRunner:
    def __init__(self, env, train_cfg, log_dir=None, device='cpu'):
        self.cfg = train_cfg['runner']
        self.alg_cfg = train_cfg['algorithm']
        self.policy_cfg = train_cfg['policy']
        self.device = device
        self.env = env
        self.joint_order_adapter = JointOrderAdapter.from_privileged_obs_dim(
            getattr(self.env, 'dof_names', []),
            device=self.device,
            privileged_obs_dim=self.env.num_privileged_obs,
        )

        if self.env.num_privileged_obs is not None:
            num_critic_obs = self.env.num_privileged_obs
        else:
            num_critic_obs = self.env.num_obs

        actor_critic = ActorCritic(self.env.num_obs, num_critic_obs, self.env.num_actions, **self.policy_cfg).to(self.device)
        self.alg = PPO(actor_critic, device=self.device, **self.alg_cfg)

        self.num_steps_per_env = self.cfg['num_steps_per_env']
        self.save_interval = self.cfg['save_interval']
        self.debug_reward = bool(self.cfg.get('debug_reward', False))
        self.log_episode_keys = self.cfg.get('log_episode_keys')
        self.infinite_mode = bool(self.cfg.get('infinite_mode', False))
        self.infinite_scheduler = (
            InfiniteLevelScheduler(self.env, self.cfg, log_dir=log_dir) if self.infinite_mode else None
        )

        self.alg.init_storage(
            self.env.num_envs,
            self.num_steps_per_env,
            [self.env.num_obs],
            [self.env.num_privileged_obs],
            [self.env.num_actions],
        )

        self.log_dir = log_dir
        self.writer = None
        self.tot_timesteps = 0
        self.tot_time = 0
        self.current_learning_iteration = 0

        self.env.reset()

        if self.log_dir is not None:
            Path(self.log_dir).mkdir(parents=True, exist_ok=True)
            all_cfg = {'train_cfg': train_cfg, 'env_cfg': class_to_dict(self.env.cfg)}
            yaml.safe_dump(all_cfg, open(os.path.join(self.log_dir, 'config.yaml'), 'w'))

    def _format_debug_lines(self, pairs, prefix, pad, per_line=3):
        if not pairs:
            return ''
        lines = []
        for start in range(0, len(pairs), per_line):
            chunk = pairs[start:start + per_line]
            label = prefix if start == 0 else ''
            lines.append(f"{label:>{pad}} " + " | ".join(chunk) + "\n")
        return ''.join(lines)

    def load(self, path):
        checkpoint = torch.load(path, map_location=self.device)
        self.alg.actor_critic.load_state_dict(checkpoint['model_state_dict'])
        optimizer_state = checkpoint.get('optimizer_state_dict')
        if optimizer_state is not None:
            self.alg.optimizer.load_state_dict(optimizer_state)
        stored_iter = int(checkpoint.get('iter', 0))
        if stored_iter == 0:
            match = re.match(r'^model_(\d+)\.pt$', Path(path).name)
            if match:
                stored_iter = int(match.group(1))
        self.current_learning_iteration = stored_iter
        return checkpoint

    def learn(self, num_learning_iterations, init_at_random_ep_len=False):
        if self.log_dir is not None and self.writer is None:
            self.writer = SummaryWriter(log_dir=self.log_dir, flush_secs=10)
        if self.infinite_scheduler is not None:
            self.infinite_scheduler.initialize(self.current_learning_iteration, self.tot_time, self.tot_timesteps)

        if init_at_random_ep_len:
            self.env.episode_length_buf = torch.randint_like(self.env.episode_length_buf, high=int(self.env.max_episode_length))

        obs = self.env.get_observations()
        privileged_obs = self.env.get_privileged_observations()
        obs = self.joint_order_adapter.actor_obs_env_to_policy(obs, self.env.num_actions)
        privileged_obs = self.joint_order_adapter.critic_obs_env_to_policy(privileged_obs, self.env.num_actions)
        critic_obs = privileged_obs if privileged_obs is not None else obs
        obs, critic_obs = obs.to(self.device), critic_obs.to(self.device)

        self.alg.train_mode()

        ep_infos = []
        rewbuffer = deque(maxlen=100)
        lenbuffer = deque(maxlen=100)
        cur_reward_sum = torch.zeros(self.env.num_envs, dtype=torch.float, device=self.device)
        cur_episode_length = torch.zeros(self.env.num_envs, dtype=torch.float, device=self.device)

        it = self.current_learning_iteration
        total_iterations = None if self.infinite_mode else (self.current_learning_iteration + num_learning_iterations)
        while self.infinite_mode or it < total_iterations:
            start = time.time()

            with torch.inference_mode():
                for _ in range(self.num_steps_per_env):
                    actions = self.alg.act(obs, critic_obs)
                    env_actions = self.joint_order_adapter.actions_policy_to_env(actions)
                    obs, privileged_obs, rewards, dones, infos = self.env.step(env_actions)
                    if not getattr(self.env, 'headless', True):
                        self.env.render()
                    obs = self.joint_order_adapter.actor_obs_env_to_policy(obs, self.env.num_actions)
                    privileged_obs = self.joint_order_adapter.critic_obs_env_to_policy(privileged_obs, self.env.num_actions)
                    critic_obs = privileged_obs if privileged_obs is not None else obs
                    obs, critic_obs = obs.to(self.device), critic_obs.to(self.device)
                    rewards, dones = rewards.to(self.device), dones.to(self.device)
                    self.alg.process_env_step(rewards, dones, infos)

                    if self.log_dir is not None:
                        if 'episode' in infos:
                            ep_infos.append(infos['episode'])
                        cur_reward_sum += rewards
                        cur_episode_length += 1
                        new_ids = (dones > 0).nonzero(as_tuple=False)
                        rewbuffer.extend(cur_reward_sum[new_ids][:, 0].cpu().numpy().tolist())
                        lenbuffer.extend(cur_episode_length[new_ids][:, 0].cpu().numpy().tolist())
                        cur_reward_sum[new_ids] = 0
                        cur_episode_length[new_ids] = 0

                stop = time.time()
                collection_time = stop - start
                start = stop
                self.alg.compute_returns(critic_obs)

            update_out = self.alg.update()
            if isinstance(update_out, tuple):
                mean_value_loss = update_out[0]
                mean_surrogate_loss = update_out[1]
                mean_entropy = update_out[2] if len(update_out) > 2 else 0.0
                # update_out[3] = mean_rnd_loss (unused)
                mean_sym_loss = update_out[4] if len(update_out) > 4 else None
            else:
                mean_value_loss = update_out
                mean_surrogate_loss = 0.0
                mean_entropy = 0.0
                mean_sym_loss = None
            stop = time.time()
            learn_time = stop - start

            if self.log_dir is not None:
                self.log(locals())
            if it % self.save_interval == 0:
                self.save(os.path.join(self.log_dir, f'model_{it}.pt'), iteration=it)
            stop_training = False
            transitioned_level = False
            if self.infinite_scheduler is not None:
                self.current_learning_iteration = int(it)
                stop_training, transitioned_level = self.infinite_scheduler.handle_iteration(
                    it, ep_infos, self.tot_time, self.tot_timesteps, self.save,
                )
            ep_infos.clear()
            if transitioned_level:
                obs = self.env.get_observations()
                privileged_obs = self.env.get_privileged_observations()
                critic_obs = privileged_obs if privileged_obs is not None else obs
                obs, critic_obs = obs.to(self.device), critic_obs.to(self.device)
            it += 1
            if stop_training:
                break

        self.current_learning_iteration = it
        self.save(os.path.join(self.log_dir, f'model_{self.current_learning_iteration}.pt'), iteration=self.current_learning_iteration)

    def log(self, locs, width=80, pad=35):
        self.tot_timesteps += self.num_steps_per_env * self.env.num_envs
        self.tot_time += locs['collection_time'] + locs['learn_time']
        iteration_time = locs['collection_time'] + locs['learn_time']
        fps = int(self.num_steps_per_env * self.env.num_envs / (locs['collection_time'] + locs['learn_time']))

        # Keep parity with rsl_rl-style episode logging from infos["episode"].
        ep_string = ""
        filtered_ep_string = ""
        if locs.get('ep_infos'):
            for key in locs['ep_infos'][0]:
                infotensor = torch.tensor([], device=self.device)
                for ep_info in locs['ep_infos']:
                    if not isinstance(ep_info[key], torch.Tensor):
                        ep_info[key] = torch.tensor([ep_info[key]], device=self.device, dtype=torch.float)
                    if len(ep_info[key].shape) == 0:
                        ep_info[key] = ep_info[key].unsqueeze(0)
                    infotensor = torch.cat((infotensor, ep_info[key].to(self.device)))
                value = torch.mean(infotensor).item()
                self.writer.add_scalar('Episode/' + key, value, locs['it'])
                line = f"{f'Mean episode {key}:':>{pad}} {value:.4f}\n"
                ep_string += line
                if self.log_episode_keys is None or key in self.log_episode_keys:
                    filtered_ep_string += line

        self.writer.add_scalar('Loss/value_function', locs['mean_value_loss'], locs['it'])
        self.writer.add_scalar('Loss/surrogate', locs['mean_surrogate_loss'], locs['it'])
        self.writer.add_scalar('Loss/entropy', locs['mean_entropy'], locs['it'])
        if locs.get('mean_sym_loss') is not None:
            self.writer.add_scalar('Loss/symmetry', locs['mean_sym_loss'], locs['it'])
        mean_std = self.alg.actor_critic.std.mean()
        self.writer.add_scalar('Policy/mean_noise_std', mean_std.item(), locs['it'])
        self.writer.add_scalar('Perf/total_fps', fps, locs['it'])
        self.writer.add_scalar('Perf/collection time', locs['collection_time'], locs['it'])
        self.writer.add_scalar('Perf/learning_time', locs['learn_time'], locs['it'])
        if len(locs['rewbuffer']) > 0:
            self.writer.add_scalar('Train/mean_reward', statistics.mean(locs['rewbuffer']), locs['it'])
            self.writer.add_scalar('Train/mean_episode_length', statistics.mean(locs['lenbuffer']), locs['it'])
            self.writer.add_scalar('Train/mean_reward/time', statistics.mean(locs['rewbuffer']), self.tot_time)
            self.writer.add_scalar('Train/mean_episode_length/time', statistics.mean(locs['lenbuffer']), self.tot_time)
        if self.debug_reward and isinstance(getattr(self.env, 'reward_debug_info', None), dict):
            for k, v in self.env.reward_debug_info.items():
                self.writer.add_scalar(f'RewardDebug/{k}', float(v), locs['it'])

        total_label = 'inf' if self.infinite_mode else str(self.current_learning_iteration + locs['num_learning_iterations'])
        title = f" \033[1m Learning iteration {locs['it']}/{total_label} \033[0m "
        if len(locs['rewbuffer']) > 0:
            log_string = (
                f"{'#' * width}\n"
                f"{title.center(width, ' ')}\n\n"
                f"{'Computation:':>{pad}} {fps:.0f} steps/s (collection: {locs['collection_time']:.3f}s, learning {locs['learn_time']:.3f}s)\n"
                f"{'Value function loss:':>{pad}} {locs['mean_value_loss']:.4f}\n"
                f"{'Surrogate loss:':>{pad}} {locs['mean_surrogate_loss']:.4f}\n"
                f"{'Mean entropy:':>{pad}} {locs.get('mean_entropy', 0.0):.4f}\n"
                + (f"{'Symmetry loss:':>{pad}} {locs['mean_sym_loss']:.4f}\n" if locs.get('mean_sym_loss') is not None else "")
                + f"{'Mean action noise std:':>{pad}} {mean_std.item():.2f}\n"
                f"{'Mean reward:':>{pad}} {statistics.mean(locs['rewbuffer']):.2f}\n"
                f"{'Mean episode length:':>{pad}} {statistics.mean(locs['lenbuffer']):.2f}\n"
            )
        else:
            log_string = (
                f"{'#' * width}\n"
                f"{title.center(width, ' ')}\n\n"
                f"{'Computation:':>{pad}} {fps:.0f} steps/s (collection: {locs['collection_time']:.3f}s, learning {locs['learn_time']:.3f}s)\n"
                f"{'Value function loss:':>{pad}} {locs['mean_value_loss']:.4f}\n"
                f"{'Surrogate loss:':>{pad}} {locs['mean_surrogate_loss']:.4f}\n"
                f"{'Mean entropy:':>{pad}} {locs.get('mean_entropy', 0.0):.4f}\n"
                + (f"{'Symmetry loss:':>{pad}} {locs['mean_sym_loss']:.4f}\n" if locs.get('mean_sym_loss') is not None else "")
                + f"{'Mean action noise std:':>{pad}} {mean_std.item():.2f}\n"
            )

        if self.debug_reward and isinstance(getattr(self.env, 'reward_debug_info', None), dict):
            dbg_pairs = []
            for k, v in sorted(self.env.reward_debug_info.items()):
                dbg_pairs.append(f"{k}={float(v):.4f}")
            log_string += self._format_debug_lines(dbg_pairs, 'Reward debug:', pad, per_line=3)

        log_string += filtered_ep_string if self.log_episode_keys is not None else ep_string
        log_string += (
            f"{'-' * width}\n"
            f"{'Total timesteps:':>{pad}} {self.tot_timesteps}\n"
            f"{'Iteration time:':>{pad}} {iteration_time:.2f}s\n"
            f"{'Total time:':>{pad}} {self.tot_time:.2f}s\n"
        )
        if self.infinite_mode:
            log_string += f"{'ETA:':>{pad}} n/a\n"
            if self.infinite_scheduler is not None and self.infinite_scheduler.initialized:
                summary = self.infinite_scheduler.level_summary(locs['it'], self.tot_time, self.tot_timesteps)
                log_string += (
                    f"{'Infinite level:':>{pad}} idx={summary['level_index']} "
                    f"height={summary['threshold_height']:.3f} "
                    f"episodes={summary['episodes']} "
                    f"iterations={summary['iterations_spent']}\n"
                )
                log_string += (
                    f"{'Infinite recent:':>{pad}} "
                    f"success={summary['recent_success_rate']:.3f} "
                    f"pass={summary['recent_pass_rate']:.3f} "
                    f"failure={summary['recent_failure_rate']:.3f} "
                    f"episodes={summary['recent_episodes']}\n"
                )
                log_string += (
                    f"{'Infinite cumulative:':>{pad}} "
                    f"success={summary['success_rate']:.3f} "
                    f"pass={summary['pass_rate']:.3f} "
                    f"failure={summary['failure_rate']:.3f} "
                    f"best_rel_x={summary['mean_best_rel_x']:.3f}\n"
                )
        else:
            eta = self.tot_time / (locs['it'] + 1) * (locs['num_learning_iterations'] - locs['it'])
            log_string += f"{'ETA:':>{pad}} {eta:.1f}s\n"
        print(log_string)

    def save(self, path, iteration=None):
        if iteration is None:
            iteration = self.current_learning_iteration
        torch.save(
            {
                'model_state_dict': self.alg.actor_critic.state_dict(),
                'optimizer_state_dict': self.alg.optimizer.state_dict(),
                'iter': int(iteration),
            },
            path,
        )
