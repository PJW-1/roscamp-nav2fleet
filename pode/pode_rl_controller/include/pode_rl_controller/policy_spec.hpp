#pragma once

#include <string>
#include <vector>

namespace pode_rl_controller
{

// ---- 정책 명세(policy_spec.json, learning/goal_reach/export_policy.py가 씀) ----
struct LidarSpec
{
  int rays;
  double first_ray_deg;
  double ray_step_deg;
  int sectors;
  double min_range;
  double obs_max;
};

struct ActionSpec
{
  double v_min;
  double v_max;
  double w_max;
  double acc_v;
  double acc_w;
};

struct GoalRule
{
  std::string rule;
  double lookahead_m;
  int follow_window;
};

struct PolicySpec
{
  double control_dt;
  int obs_dim;
  int act_dim;
  LidarSpec lidar;
  double goal_obs_max;
  double success_radius;
  GoalRule goal_rule;
  ActionSpec action;
};

std::vector<std::string> expected_obs_layout(int sectors);
PolicySpec load_policy_spec(const std::string & path);

}  // namespace pode_rl_controller
