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

struct PrevActionSpec
{
  double v_scale;
  double w_scale;
};

struct LidarHistorySpec
{
  int frames;
  int stride_steps;
};

struct SpeedCapSpec
{
  double min;
  double max;
  double obs_scale;
};

struct PolicySpec
{
  int format;
  double control_dt;
  int obs_dim;
  int act_dim;
  LidarSpec lidar;
  double goal_obs_max;
  double success_radius;
  GoalRule goal_rule;
  ActionSpec action;
  std::vector<std::string> obs_blocks;
  PrevActionSpec prev_action;
  LidarHistorySpec lidar_history;
  SpeedCapSpec speed_cap;

  bool has_block(const std::string & name) const;
};

std::vector<std::string> expected_obs_layout(int sectors);
std::vector<std::string> expected_obs_layout(const PolicySpec & spec);
PolicySpec load_policy_spec(const std::string & path);

// ---- 주행 최고 속도의 단일 출처(pode_nav_config/params/nav_speed.yaml) ----
double load_nav_speed(const std::string & path);
std::string speed_mismatch(double policy_v_max, double nav_speed);

// ---- 속도 상한 관측(format 3 speed_cap) ----
struct SpeedCapValue
{
  double value;
  std::string error;
};

SpeedCapValue speed_cap_value(double source, const SpeedCapSpec & cap);

}  // namespace pode_rl_controller
