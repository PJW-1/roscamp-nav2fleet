#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
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

double gap(double got, double want)
{
  const double d = std::abs(got - want);
  return std::isfinite(d) ? d : std::numeric_limits<double>::infinity();
}

double case_cap(const PolicySpec & s, const YAML::Node & c)
{
  return c["speed_cap"] ? c["speed_cap"].as<double>() : s.action.v_max;
}

std::vector<float> with_blocks(const PolicySpec & s, std::vector<float> obs, const YAML::Node & c)
{
  if (s.obs_blocks.empty()) {
    return obs;
  }
  pode_rl_controller::ObsMemory memory(s);
  const std::vector<float> sectors(obs.begin(), obs.begin() + s.lidar.sectors);
  pode_rl_controller::Twist2 prev{0.0, 0.0};
  if (c["prev_target"]) {
    const auto p = vec<double>(c["prev_target"]);
    prev = {p[0], p[1]};
  }
  if (s.has_block("lidar_history")) {
    const auto hist = c["lidar_history"].as<std::vector<std::vector<float>>>();
    const int stride = s.lidar_history.stride_steps;
    for (int k = (s.lidar_history.frames - 1) * stride - 1; k >= 0; --k) {
      memory.step(hist[static_cast<size_t>(k / stride)], prev);
    }
  } else {
    memory.step(sectors, prev);
  }
  const auto extra = memory.blocks(sectors, case_cap(s, c));
  obs.insert(obs.end(), extra.begin(), extra.end());
  return obs;
}
}  // namespace

TEST(Gap, NonFiniteNeverPasses)
{
  EXPECT_GT(gap(std::nan(""), 1.0), 1.0);
  EXPECT_GT(gap(1.0, std::numeric_limits<double>::infinity()), 1.0);
  EXPECT_DOUBLE_EQ(gap(1.5, 1.0), 0.5);
}

// ---- 명세 파일 ----
TEST(PolicySpec, ReadsTheExportedTaskSpec)
{
  const PolicySpec s = spec();
  EXPECT_EQ(s.obs_dim, static_cast<int>(pode_rl_controller::expected_obs_layout(s).size()));
  EXPECT_EQ(s.lidar.rays, 360);
  EXPECT_EQ(s.lidar.sectors, 36);
  EXPECT_DOUBLE_EQ(s.control_dt, 0.05);
  EXPECT_DOUBLE_EQ(s.action.v_min, -0.05);
  EXPECT_DOUBLE_EQ(s.action.v_max, pode_rl_controller::load_nav_speed(NAV_SPEED_FILE));
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
      worst_rays = std::max(worst_rays, gap(rays[k], want_rays[k]));
    }
    const auto obs = with_blocks(s, pode_rl_controller::build_observation(
        rays, c["goal_dist"].as<double>(), c["goal_angle"].as<double>(), c["v"].as<double>(),
        c["w"].as<double>(), s), c);
    const auto want_obs = vec<double>(c["obs"]);
    ASSERT_EQ(obs.size(), want_obs.size());
    for (size_t k = 0; k < obs.size(); ++k) {
      worst_obs = std::max(worst_obs, gap(obs[k], want_obs[k]));
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
        cmd, pode_rl_controller::action_to_target(a[0], a[1], s.action, case_cap(s, c)),
        s.control_dt, s.action);
      const auto want = vec<double>(cmds[t]);
      worst = std::max({worst, gap(cmd.v, want[0]), gap(cmd.w, want[1])});
    }
  }
  std::printf("[parity] action -> (v, w) max |diff| %.3g\n", worst);
  EXPECT_LE(worst, kObsTol);
}

// ---- speed_cap 원천: nav_speed.yaml, 학습 범위 밖이면 잘라서 알린다 ----
TEST(SpeedCap, NavSpeedIsTheSourceAndOutOfRangeIsClamped)
{
  const PolicySpec s = spec();
  if (!s.has_block("speed_cap")) {
    std::printf("[parity] no speed_cap block: v top is action.v_max %.3f\n", s.action.v_max);
    return;
  }
  const double nav = pode_rl_controller::load_nav_speed(NAV_SPEED_FILE);
  const auto in = pode_rl_controller::speed_cap_value(nav, s.speed_cap);
  std::printf("[parity] speed_cap range [%.3f, %.3f], nav speed %.3f -> %.3f\n",
    s.speed_cap.min, s.speed_cap.max, nav, in.value);
  EXPECT_DOUBLE_EQ(in.value, nav);
  EXPECT_TRUE(in.error.empty());
  const auto low = pode_rl_controller::speed_cap_value(s.speed_cap.min - 0.01, s.speed_cap);
  EXPECT_DOUBLE_EQ(low.value, s.speed_cap.min);
  EXPECT_FALSE(low.error.empty());
  const auto high = pode_rl_controller::speed_cap_value(s.speed_cap.max + 0.01, s.speed_cap);
  EXPECT_DOUBLE_EQ(high.value, s.speed_cap.max);
  EXPECT_FALSE(high.error.empty());
}

// ---- ONNX Runtime C++ 대 PyTorch ----
TEST(Inference, MatchesTorch)
{
  // 정책 파일은 학습 PC가 내보낸 뒤 둔다. 저장소에 있어야 하는 후보(4b)는 없으면 실패, 나머지는 건너뛴다
  const std::string onnx = std::string(POLICY_DIR) + "/policy.onnx";
  if (!std::filesystem::exists(onnx)) {
    ASSERT_FALSE(ONNX_REQUIRED) << "missing " << onnx << " (this policy is kept in the repo)";
    GTEST_SKIP() << "no " << onnx << "; export the policy to compare inference";
  }
  const PolicySpec s = spec();
  pode_rl_controller::OnnxPolicy policy(onnx, s.obs_dim, s.act_dim, 1);
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
      const double d = gap(a[j], want[j]);
      worst_torch = std::max(worst_torch, d);
      worst_ort_py = std::max(worst_ort_py, gap(a[j], ort_py[j]));
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
