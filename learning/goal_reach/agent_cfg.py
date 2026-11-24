"""goal-reach 과제의 rsl_rl PPO 설정. 값과 이유는 docs/worklog_goal_reach_rl_2026-11-21.md."""

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class GoalReachPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24  # 반복당 env 마다 20 Hz 제어 1.2 s
    max_iterations = 300
    save_interval = 50
    experiment_name = "pode_goal_reach"
    obs_groups = {"actor": ["policy"], "critic": ["policy"]}
    clip_actions = 1.0
    actor = RslRlMLPModelCfg(
        hidden_dims=[128, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.5),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[128, 128], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
