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
