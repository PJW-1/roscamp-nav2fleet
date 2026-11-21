#pragma once

#include <cstddef>
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

struct Point2
{
  double x;
  double y;
};

struct GoalPolar
{
  double dist;
  double angle;
};

// ---- 경로 따라가기(route_spec.follow와 같은 정의) ----
size_t follow_progress(
  const std::vector<Point2> & path, size_t prog, const Point2 & robot, int window);
size_t lookahead_index(const std::vector<Point2> & path, size_t prog, double lookahead);
GoalPolar goal_in_robot_frame(const Point2 & robot, double yaw, const Point2 & goal);
GoalPolar path_goal(
  const std::vector<Point2> & path, size_t & prog, const Point2 & robot, double yaw,
  const GoalRule & rule);

std::vector<float> scan_to_rays(const Scan & scan, double lidar_yaw, const PolicySpec & spec);
std::vector<float> build_observation(
  const std::vector<float> & rays, double goal_dist, double goal_angle, double v, double w,
  const PolicySpec & spec);
Twist2 action_to_target(float a_v, float a_w, const ActionSpec & lim);
Twist2 block_reverse(
  const Twist2 & target, const std::vector<float> & rays, double stop_range, double half_width_deg,
  const PolicySpec & spec);
Twist2 rate_limit(const Twist2 & cmd, const Twist2 & target, double dt, const ActionSpec & lim);

}  // namespace pode_rl_controller
