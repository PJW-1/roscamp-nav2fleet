#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "pode_rl_controller/observation.hpp"
#include "pode_rl_controller/onnx_policy.hpp"
#include "pode_rl_controller/policy_spec.hpp"

using pode_rl_controller::PolicySpec;

namespace
{
constexpr double kObsTol = 1e-6;

PolicySpec spec()
{
  return pode_rl_controller::load_policy_spec(std::string(POLICY_DIR) + "/policy_spec.json");
}

YAML::Node fixture(const std::string & name)
{
  return YAML::LoadFile(std::string(FIXTURE_DIR) + "/" + name);
}

template<typename T>
std::vector<T> vec(const YAML::Node & n)
{
  return n.as<std::vector<T>>();
}
}  // namespace

// ---- 명세 파일 ----
TEST(PolicySpec, ReadsTheExportedTaskSpec)
{
  const PolicySpec s = spec();
  EXPECT_EQ(s.obs_dim, 41);
  EXPECT_EQ(s.lidar.rays, 360);
  EXPECT_EQ(s.lidar.sectors, 36);
  EXPECT_DOUBLE_EQ(s.control_dt, 0.05);
  EXPECT_DOUBLE_EQ(s.action.v_min, -0.05);
  EXPECT_DOUBLE_EQ(s.action.v_max, 0.25);
  EXPECT_DOUBLE_EQ(s.action.w_max, 1.0);
}

// ---- 스캔 -> 광선 -> 관측: task_spec과 같은가 ----
TEST(Observation, MatchesTaskSpec)
{
  const PolicySpec s = spec();
  double worst_rays = 0.0;
  double worst_obs = 0.0;
  for (const auto & c : fixture("obs_cases.json")) {
    pode_rl_controller::Scan scan{c["angle_min"].as<double>(), c["angle_increment"].as<double>(),
      c["range_min"].as<double>(), c["range_max"].as<double>(), vec<float>(c["ranges"])};
    const auto rays = pode_rl_controller::scan_to_rays(scan, c["lidar_yaw"].as<double>(), s);
    const auto want_rays = vec<double>(c["rays"]);
    ASSERT_EQ(rays.size(), want_rays.size());
    for (size_t k = 0; k < rays.size(); ++k) {
      worst_rays = std::max(worst_rays, std::abs(rays[k] - want_rays[k]));
    }
    const auto obs = pode_rl_controller::build_observation(
      rays, c["goal_dist"].as<double>(), c["goal_angle"].as<double>(), c["v"].as<double>(),
      c["w"].as<double>(), s);
    const auto want_obs = vec<double>(c["obs"]);
    ASSERT_EQ(obs.size(), want_obs.size());
    for (size_t k = 0; k < obs.size(); ++k) {
      worst_obs = std::max(worst_obs, std::abs(obs[k] - want_obs[k]));
    }
  }
  std::printf("[parity] rays max |diff| %.3g, obs max |diff| %.3g\n", worst_rays, worst_obs);
  EXPECT_LE(worst_rays, kObsTol);
  EXPECT_LE(worst_obs, kObsTol);
}

TEST(Observation, NoReturnBecomesObsMax)
{
  const PolicySpec s = spec();
  pode_rl_controller::Scan scan{-M_PI, 2 * M_PI / 360, 0.05, 12.0,
    std::vector<float>(360, std::numeric_limits<float>::infinity())};
  scan.ranges[180] = 0.5f;
  const auto rays = pode_rl_controller::scan_to_rays(scan, 0.0, s);
  EXPECT_FLOAT_EQ(rays[180], 0.5f);
  EXPECT_FLOAT_EQ(rays[0], 2.0f);
  const auto rays_turned = pode_rl_controller::scan_to_rays(scan, M_PI, s);
  EXPECT_FLOAT_EQ(rays_turned[0], 0.5f);
}

// ---- 행동 -> (v, w), 가속 제한 ----
TEST(Action, MatchesTaskSpec)
{
  const PolicySpec s = spec();
  double worst = 0.0;
  for (const auto & c : fixture("action_cases.json")) {
    const auto start = vec<double>(c["start"]);
    pode_rl_controller::Twist2 cmd{start[0], start[1]};
    const auto & acts = c["actions"];
    const auto & cmds = c["cmds"];
    for (size_t t = 0; t < acts.size(); ++t) {
      const auto a = vec<float>(acts[t]);
      cmd = pode_rl_controller::rate_limit(
        cmd, pode_rl_controller::action_to_target(a[0], a[1], s.action), s.control_dt, s.action);
      const auto want = vec<double>(cmds[t]);
      worst = std::max({worst, std::abs(cmd.v - want[0]), std::abs(cmd.w - want[1])});
    }
  }
  std::printf("[parity] action -> (v, w) max |diff| %.3g\n", worst);
  EXPECT_LE(worst, kObsTol);
}

// ---- ONNX Runtime C++ 대 PyTorch ----
TEST(Inference, MatchesTorch)
{
  const PolicySpec s = spec();
  pode_rl_controller::OnnxPolicy policy(
    std::string(POLICY_DIR) + "/policy.onnx", s.obs_dim, s.act_dim, 1);
  const auto f = fixture("inference_cases.json");
  const double atol = f["atol"].as<double>();
  const double rtol = f["rtol"].as<double>();
  double worst_torch = 0.0;
  double worst_ort_py = 0.0;
  double worst_excess = -1.0;
  for (size_t i = 0; i < f["obs"].size(); ++i) {
    const auto a = policy.run(vec<float>(f["obs"][i]));
    const auto want = vec<double>(f["actions_torch"][i]);
    const auto ort_py = vec<double>(f["actions_onnxruntime_python"][i]);
    for (int j = 0; j < s.act_dim; ++j) {
      const double d = std::abs(a[j] - want[j]);
      worst_torch = std::max(worst_torch, d);
      worst_ort_py = std::max(worst_ort_py, std::abs(a[j] - ort_py[j]));
      worst_excess = std::max(worst_excess, d - atol - rtol * std::abs(want[j]));
    }
  }
  std::printf(
    "[parity] onnxruntime C++ vs torch max |diff| %.3g (bound margin %.3g), "
    "vs onnxruntime python %.3g\n", worst_torch, -worst_excess, worst_ort_py);
  EXPECT_LE(worst_excess, 0.0);

  const auto obs = vec<float>(f["obs"][0]);
  const int n = 2000;
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) {
    policy.run(obs);
  }
  const double us = std::chrono::duration<double, std::micro>(
    std::chrono::steady_clock::now() - t0).count() / n;
  std::printf("[parity] inference %.1f us per call (mean of %d, 1 thread)\n", us, n);
}
