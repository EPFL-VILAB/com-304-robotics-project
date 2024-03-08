import collections
from typing import Dict

import torch

from habitat.utils import profiling_wrapper
from habitat_baselines.common.rollout_storage import RolloutStorage
from habitat_baselines.rl.ppo import PPO
from habitat_baselines.rl.ver.ver_rollout_storage import VERRolloutStorage
from habitat_baselines.utils.common import inference_mode


class BasePolicyGradient(PPO):
    def update(
        self,
        rollouts: RolloutStorage,
    ) -> Dict[str, float]:

        advantages = self.get_advantages(rollouts)

        learner_metrics = collections.defaultdict(list)

        def record_min_mean_max(t: torch.Tensor, prefix: str):
            for name, op in (
                ("min", torch.min),
                ("mean", torch.mean),
                ("max", torch.max),
            ):
                learner_metrics[f"{prefix}_{name}"].append(op(t))

        for epoch in range(self.ppo_epoch):
            profiling_wrapper.range_push("PPO.update epoch")
            data_generator = rollouts.recurrent_generator(
                advantages, self.num_mini_batch
            )

            for _bid, batch in enumerate(data_generator):
                self._set_grads_to_none()

                (
                    values,
                    action_log_probs,
                    _,
                    _,
                    aux_loss_res,
                ) = self._evaluate_actions(
                    batch["observations"],
                    batch["recurrent_hidden_states"],
                    batch["prev_actions"],
                    batch["masks"],
                    batch["actions"],
                    batch["rnn_build_seq_info"],
                )

                action_loss = self._compute_action_loss(
                    advantages=batch["advantages"],
                    returns=batch["returns"],
                    action_log_probs=action_log_probs,
                    values=values
                )

                values = values.float()
                orig_values = values

                value_loss = self._compute_value_loss(
                    advantages=batch["advantages"],
                    returns=batch["returns"],
                    action_log_probs=action_log_probs,
                    values=values
                )

                if "is_coeffs" in batch:
                    assert isinstance(batch["is_coeffs"], torch.Tensor)
                    ver_is_coeffs = batch["is_coeffs"].clamp(max=1.0)
                    mean_fn = lambda t: torch.mean(ver_is_coeffs * t)
                else:
                    mean_fn = torch.mean

                action_loss, value_loss = map(
                    mean_fn,
                    (action_loss, value_loss),
                )

                all_losses = [
                    self.value_loss_coef * value_loss,
                    action_loss,
                ]

                all_losses.extend(v["loss"] for v in aux_loss_res.values())

                total_loss = torch.stack(all_losses).sum()

                total_loss = self.before_backward(total_loss)
                total_loss.backward()
                self.after_backward(total_loss)

                grad_norm = self.before_step()
                self.optimizer.step()
                self.after_step()

                with inference_mode():
                    if "is_coeffs" in batch:
                        record_min_mean_max(
                            batch["is_coeffs"], "ver_is_coeffs"
                        )
                    record_min_mean_max(orig_values, "value_pred")

                    learner_metrics["value_loss"].append(value_loss)
                    learner_metrics["action_loss"].append(action_loss)

                    learner_metrics["grad_norm"].append(grad_norm)

                    for name, res in aux_loss_res.items():
                        for k, v in res.items():
                            learner_metrics[f"aux_{name}_{k}"].append(
                                v.detach()
                            )

                    if "is_stale" in batch:
                        assert isinstance(batch["is_stale"], torch.Tensor)
                        learner_metrics["fraction_stale"].append(
                            batch["is_stale"].float().mean()
                        )

                    if isinstance(rollouts, VERRolloutStorage):
                        assert isinstance(
                            batch["policy_version"], torch.Tensor
                        )
                        record_min_mean_max(
                            (
                                rollouts.current_policy_version
                                - batch["policy_version"]
                            ).float(),
                            "policy_version_difference",
                        )

            profiling_wrapper.range_pop()  # PPO.update epoch

        self._set_grads_to_none()

        with inference_mode():
            return {
                k: float(
                    torch.stack(
                        [torch.as_tensor(v, dtype=torch.float32) for v in vs]
                    ).mean()
                )
                for k, vs in learner_metrics.items()
            }

    def _compute_action_loss(
            self,
            advantages: torch.Tensor,  # Advantage, A_t
            returns: torch.Tensor,  # Return, R_t
            action_log_probs: torch.Tensor,  # Log action probability, log(pi(a_t|s_t))
            values: torch.Tensor  # Value prediction, V_t
    ):
        raise NotImplementedError

    def _compute_value_loss(
            self,
            advantages: torch.Tensor,  # Advantage, A_t
            returns: torch.Tensor,  # Return, R_t
            action_log_probs: torch.Tensor,  # Log action probability, log(pi(a_t|s_t))
            values: torch.Tensor  # Value prediction, V_t
    ):
        raise NotImplementedError
