#include "pode_rl_controller/policy_spec.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace pode_rl_controller
{

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

PolicySpec load_policy_spec(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  if (root["format"].as<int>() != 2) {
    throw std::runtime_error(path + ": policy spec format must be 2 (export it again)");
  }
  PolicySpec s{};
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
  if (root["obs_layout"].as<std::vector<std::string>>() != expected_obs_layout(s.lidar.sectors) ||
    s.obs_dim != s.lidar.sectors + 5)
  {
    throw std::runtime_error(path + ": observation layout differs from the one this plugin builds");
  }
  if (s.act_dim != 2) {
    throw std::runtime_error(path + ": act_dim must be 2 (v, w)");
  }
  return s;
}

}  // namespace pode_rl_controller
