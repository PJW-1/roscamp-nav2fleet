#include "pode_rl_controller/policy_spec.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace pode_rl_controller
{

namespace
{
const std::vector<std::string> kBlockOrder{"prev_action", "lidar_history", "speed_cap", "intent"};

int block_dim(const std::string & name, const PolicySpec & s)
{
  if (name == "prev_action") {
    return 2;
  }
  if (name == "lidar_history") {
    return s.lidar.sectors * (s.lidar_history.frames - 1);
  }
  return 1;
}

void read_blocks(const YAML::Node & root, const std::string & path, PolicySpec & s)
{
  s.obs_blocks = root["obs_blocks"].as<std::vector<std::string>>();
  size_t at = 0;
  for (const auto & name : s.obs_blocks) {
    const auto it = std::find(kBlockOrder.begin() + at, kBlockOrder.end(), name);
    if (it == kBlockOrder.end()) {
      throw std::runtime_error(
        path + ": obs_blocks must be a subset of prev_action, lidar_history, speed_cap, intent "
        "in that order without repeats (got " + name + ")");
    }
    at = static_cast<size_t>(it - kBlockOrder.begin()) + 1;
  }
  if (s.has_block("intent")) {
    throw std::runtime_error(
      path + ": obs block intent is not supported by this FollowPathRL yet");
  }
  if (s.has_block("prev_action")) {
    const YAML::Node b = root["prev_action"];
    s.prev_action = {b["v_scale"].as<double>(), b["w_scale"].as<double>()};
    if (std::abs(s.prev_action.v_scale - s.action.v_max) > 1e-9 ||
      std::abs(s.prev_action.w_scale - s.action.w_max) > 1e-9)
    {
      throw std::runtime_error(
        path + ": prev_action v_scale and w_scale must equal action v_max and w_max (the base v "
        "and w scaling)");
    }
  }
  if (s.has_block("lidar_history")) {
    const YAML::Node b = root["lidar_history"];
    s.lidar_history = {b["frames"].as<int>(), b["stride_steps"].as<int>()};
    if (s.lidar_history.frames < 2 || s.lidar_history.stride_steps < 1) {
      throw std::runtime_error(path + ": lidar_history needs frames >= 2 and stride_steps >= 1");
    }
  }
  if (s.has_block("speed_cap")) {
    const YAML::Node b = root["speed_cap"];
    s.speed_cap = {b["min"].as<double>(), b["max"].as<double>(), b["obs_scale"].as<double>()};
    if (!(s.speed_cap.min > 0.0) || s.speed_cap.max < s.speed_cap.min) {
      throw std::runtime_error(path + ": speed_cap needs 0 < min <= max");
    }
    if (std::abs(s.speed_cap.obs_scale - s.action.v_max) > 1e-9) {
      throw std::runtime_error(
        path + ": speed_cap obs_scale must equal action v_max (the base v scaling)");
    }
  }
}
}  // namespace

bool PolicySpec::has_block(const std::string & name) const
{
  return std::find(obs_blocks.begin(), obs_blocks.end(), name) != obs_blocks.end();
}

std::vector<std::string> expected_obs_layout(int sectors)
{
  std::vector<std::string> names;
  for (int i = 0; i < sectors; ++i) {
    names.push_back("lidar_sector_" + std::to_string(i));
  }
  for (const char * name : {"goal_dist", "goal_sin", "goal_cos", "v", "w"}) {
    names.emplace_back(name);
  }
  return names;
}

std::vector<std::string> expected_obs_layout(const PolicySpec & spec)
{
  std::vector<std::string> names = expected_obs_layout(spec.lidar.sectors);
  for (const auto & block : spec.obs_blocks) {
    if (block == "prev_action") {
      names.emplace_back("prev_v");
      names.emplace_back("prev_w");
    } else if (block == "lidar_history") {
      for (int f = 1; f < spec.lidar_history.frames; ++f) {
        for (int i = 0; i < spec.lidar.sectors; ++i) {
          names.push_back("lidar_hist" + std::to_string(f) + "_sector_" + std::to_string(i));
        }
      }
    } else {
      names.push_back(block);
    }
  }
  return names;
}

PolicySpec load_policy_spec(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  PolicySpec s{};
  s.format = root["format"].as<int>();
  if (s.format != 2 && s.format != 3) {
    throw std::runtime_error(path + ": policy spec format must be 2 or 3 (export it again)");
  }
  if (s.format == 2 && root["obs_blocks"]) {
    throw std::runtime_error(path + ": obs_blocks needs policy spec format 3");
  }
  s.control_dt = root["control_dt"].as<double>();
  s.obs_dim = root["obs_dim"].as<int>();
  s.act_dim = root["act_dim"].as<int>();
  const YAML::Node l = root["lidar"];
  s.lidar = {l["rays"].as<int>(), l["first_ray_deg"].as<double>(), l["ray_step_deg"].as<double>(),
    l["sectors"].as<int>(), l["min_range"].as<double>(), l["obs_max"].as<double>()};
  s.goal_obs_max = root["goal"]["obs_max"].as<double>();
  s.success_radius = root["goal"]["success_radius"].as<double>();
  const YAML::Node g = root["goal"];
  s.goal_rule = {g["rule"].as<std::string>(), 0.0, 0};
  if (s.goal_rule.rule == "lookahead") {
    s.goal_rule.lookahead_m = g["lookahead_m"].as<double>();
    s.goal_rule.follow_window = g["follow_window"].as<int>();
    if (s.goal_rule.lookahead_m <= 0.0 || s.goal_rule.follow_window <= 0) {
      throw std::runtime_error(path + ": lookahead_m and follow_window must be positive");
    }
  } else if (s.goal_rule.rule != "path_end") {
    throw std::runtime_error(path + ": goal rule must be path_end or lookahead");
  }
  const YAML::Node a = root["action"];
  s.action = {a["v_min"].as<double>(), a["v_max"].as<double>(), a["w_max"].as<double>(),
    a["acc_v"].as<double>(), a["acc_w"].as<double>()};

  if (s.lidar.sectors <= 0 || s.lidar.rays % s.lidar.sectors != 0) {
    throw std::runtime_error(path + ": lidar rays must split evenly into sectors");
  }
  int dim = s.lidar.sectors + 5;
  if (s.format == 3) {
    read_blocks(root, path, s);
    for (const auto & block : s.obs_blocks) {
      dim += block_dim(block, s);
    }
  }
  if (root["obs_layout"].as<std::vector<std::string>>() != expected_obs_layout(s) ||
    s.obs_dim != dim)
  {
    throw std::runtime_error(path + ": observation layout differs from the one this plugin builds");
  }
  if (s.act_dim != 2) {
    throw std::runtime_error(path + ": act_dim must be 2 (v, w)");
  }
  return s;
}

double load_nav_speed(const std::string & path)
{
  const double v = YAML::LoadFile(path)["nav_speed_mps"].as<double>();
  if (!std::isfinite(v) || v <= 0.0) {
    throw std::runtime_error(path + ": nav_speed_mps must be above 0");
  }
  return v;
}

std::string speed_mismatch(double policy_v_max, double nav_speed)
{
  if (std::abs(policy_v_max - nav_speed) <= 1e-9) {
    return "";
  }
  char text[160];
  std::snprintf(
    text, sizeof(text), "policy v_max %.3f m/s differs from the one top speed %.3f m/s "
    "(nav_speed.yaml)", policy_v_max, nav_speed);
  return text;
}

SpeedCapValue speed_cap_value(double source, const SpeedCapSpec & cap)
{
  const double value = std::clamp(source, cap.min, cap.max);
  if (std::abs(value - source) <= 1e-9) {
    return {value, ""};
  }
  char text[200];
  std::snprintf(
    text, sizeof(text), "top speed %.3f m/s is outside the policy's trained speed_cap range "
    "%.3f..%.3f m/s; the policy is told %.3f m/s", source, cap.min, cap.max, value);
  return {value, text};
}

}  // namespace pode_rl_controller
