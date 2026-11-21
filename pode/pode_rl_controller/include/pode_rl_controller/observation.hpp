#pragma once

#include <vector>

#include "pode_rl_controller/policy_spec.hpp"

namespace pode_rl_controller
{

// ---- 관측과 행동(task_spec.py와 같은 정의) ----
struct Scan
{
  double angle_min;
  double angle_increment;
  double range_min;
  double range_max;
  std::vector<float> ranges;
};

struct Twist2
{
  double v;
  double w;
};

std::vector<float> scan_to_rays(const Scan & scan, double lidar_yaw, const PolicySpec & spec);
std::vector<float> build_observation(
  const std::vector<float> & rays, double goal_dist, double goal_angle, double v, double w,
  const PolicySpec & spec);
Twist2 action_to_target(float a_v, float a_w, const ActionSpec & lim);
Twist2 rate_limit(const Twist2 & cmd, const Twist2 & target, double dt, const ActionSpec & lim);

}  // namespace pode_rl_controller
