#include "pode_rl_controller/observation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pode_rl_controller
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
}  // namespace

// ---- 경로 따라가기 ----
size_t follow_progress(
  const std::vector<Point2> & path, size_t prog, const Point2 & robot, int window)
{
  const size_t n = path.size();
  prog = std::min(prog, n - 1);
  const size_t last = std::min(n - 1, prog + static_cast<size_t>(window));
  size_t near = prog;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = prog; i <= last; ++i) {
    const double d = std::hypot(path[i].x - robot.x, path[i].y - robot.y);
    if (d < best) {
      best = d;
      near = i;
    }
  }
  return near;
}

size_t lookahead_index(const std::vector<Point2> & path, size_t prog, double lookahead)
{
  std::vector<double> cum(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    cum[i] = cum[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  const double target = cum[prog] + lookahead;
  const auto it = std::lower_bound(cum.begin(), cum.end(), target);
  return std::min(static_cast<size_t>(it - cum.begin()), path.size() - 1);
}

GoalPolar goal_in_robot_frame(const Point2 & robot, double yaw, const Point2 & goal)
{
  const double dx = goal.x - robot.x;
  const double dy = goal.y - robot.y;
  double ang = std::atan2(dy, dx) - yaw;
  ang = std::atan2(std::sin(ang), std::cos(ang));
  return {std::hypot(dx, dy), ang};
}

GoalPolar path_goal(
  const std::vector<Point2> & path, size_t & prog, const Point2 & robot, double yaw,
  const GoalRule & rule)
{
  if (rule.rule != "lookahead") {
    return goal_in_robot_frame(robot, yaw, path.back());
  }
  prog = follow_progress(path, prog, robot, rule.follow_window);
  return goal_in_robot_frame(robot, yaw, path[lookahead_index(path, prog, rule.lookahead_m)]);
}

// ---- 스캔 -> 학습 광선 360개 ----
std::vector<float> scan_to_rays(const Scan & scan, double lidar_yaw, const PolicySpec & spec)
{
  const auto & lid = spec.lidar;
  const long n = static_cast<long>(scan.ranges.size());
  std::vector<float> rays(lid.rays);
  for (int k = 0; k < lid.rays; ++k) {
    const double theta = (lid.first_ray_deg + k * lid.ray_step_deg) * (kPi / 180.0);
    double rel = std::fmod(theta - lidar_yaw - scan.angle_min, kTwoPi);
    if (rel < 0.0) {
      rel += kTwoPi;
    }
    long i = static_cast<long>(std::floor(rel / scan.angle_increment + 0.5));
    if (i >= n) {
      i = (kTwoPi - rel <= 0.5 * scan.angle_increment) ? 0 : -1;
    }
    double r = i >= 0 ? scan.ranges[i] : std::numeric_limits<double>::infinity();
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
      r = lid.obs_max;
    }
    rays[k] = static_cast<float>(std::min(r, lid.obs_max));
  }
  return rays;
}

// ---- 관측 벡터 ----
std::vector<float> build_observation(
  const std::vector<float> & rays, double goal_dist, double goal_angle, double v, double w,
  const PolicySpec & spec)
{
  const auto & lid = spec.lidar;
  const int per_sector = lid.rays / lid.sectors;
  std::vector<float> obs;
  obs.reserve(spec.obs_dim);
  for (int s = 0; s < lid.sectors; ++s) {
    float m = static_cast<float>(lid.obs_max);
    for (int j = 0; j < per_sector; ++j) {
      const float r = rays[s * per_sector + j];
      m = std::min(m, std::isnan(r) ? static_cast<float>(lid.obs_max) : r);
    }
    obs.push_back(m / static_cast<float>(lid.obs_max));
  }
  obs.push_back(static_cast<float>(std::min(goal_dist, spec.goal_obs_max) / spec.goal_obs_max));
  obs.push_back(static_cast<float>(std::sin(goal_angle)));
  obs.push_back(static_cast<float>(std::cos(goal_angle)));
  obs.push_back(static_cast<float>(v / spec.action.v_max));
  obs.push_back(static_cast<float>(w / spec.action.w_max));
  return obs;
}

// ---- 행동 -> (v, w), 가속 제한 ----
Twist2 action_to_target(float a_v, float a_w, const ActionSpec & lim)
{
  const double av = std::clamp(static_cast<double>(a_v), -1.0, 1.0);
  const double aw = std::clamp(static_cast<double>(a_w), -1.0, 1.0);
  return {lim.v_min + (av + 1.0) * 0.5 * (lim.v_max - lim.v_min), aw * lim.w_max};
}

Twist2 rate_limit(const Twist2 & cmd, const Twist2 & target, double dt, const ActionSpec & lim)
{
  const double sv = lim.acc_v * dt;
  const double sw = lim.acc_w * dt;
  return {cmd.v + std::clamp(target.v - cmd.v, -sv, sv),
    cmd.w + std::clamp(target.w - cmd.w, -sw, sw)};
}

}  // namespace pode_rl_controller
