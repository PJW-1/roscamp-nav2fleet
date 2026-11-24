#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include "pode_rl_controller/observation.hpp"
#include "pode_rl_controller/policy_spec.hpp"

using pode_rl_controller::ObsMemory;
using pode_rl_controller::PolicySpec;
using pode_rl_controller::Twist2;

namespace
{
YAML::Node base_spec()
{
  return YAML::LoadFile(std::string(POLICY_DIR) + "/policy_spec.json");
}

YAML::Node format3(const std::vector<std::string> & blocks, int frames = 3, int stride = 4)
{
  YAML::Node s = base_spec();
  s["format"] = 3;
  s["obs_blocks"] = blocks;
  std::vector<std::string> layout = pode_rl_controller::expected_obs_layout(36);
  int dim = 41;
  for (const auto & b : blocks) {
    if (b == "prev_action") {
      s["prev_action"]["v_scale"] = 0.12;
      s["prev_action"]["w_scale"] = 1.0;
      layout.push_back("prev_v");
      layout.push_back("prev_w");
      dim += 2;
    } else if (b == "lidar_history") {
      s["lidar_history"]["frames"] = frames;
      s["lidar_history"]["stride_steps"] = stride;
      for (int f = 1; f < frames; ++f) {
        for (int i = 0; i < 36; ++i) {
          layout.push_back("lidar_hist" + std::to_string(f) + "_sector_" + std::to_string(i));
        }
      }
      dim += 36 * (frames - 1);
    } else if (b == "speed_cap") {
      s["speed_cap"]["min"] = 0.08;
      s["speed_cap"]["max"] = 0.20;
      s["speed_cap"]["obs_scale"] = 0.12;
      layout.push_back("speed_cap");
      dim += 1;
    } else if (b == "intent") {
      layout.push_back("intent");
      dim += 1;
    }
  }
  s["obs_layout"] = layout;
  s["obs_dim"] = dim;
  return s;
}

PolicySpec load(const YAML::Node & s, const std::string & name)
{
  const std::string path = ::testing::TempDir() + "/" + name + ".json";
  std::ofstream(path) << YAML::Dump(s);
  return pode_rl_controller::load_policy_spec(path);
}

std::string refusal(const YAML::Node & s, const std::string & name)
{
  try {
    load(s, name);
  } catch (const std::exception & e) {
    return e.what();
  }
  return "";
}

const std::vector<std::string> kAll{"prev_action", "lidar_history", "speed_cap"};

std::vector<float> frame(float value)
{
  return std::vector<float>(36, value);
}
}  // namespace

TEST(Format3Spec, Format2LoadsUnchangedWithoutBlocks)
{
  const PolicySpec s = pode_rl_controller::load_policy_spec(
    std::string(POLICY_DIR) + "/policy_spec.json");
  EXPECT_EQ(s.format, 2);
  EXPECT_TRUE(s.obs_blocks.empty());
  EXPECT_EQ(s.obs_dim, 41);
  EXPECT_FALSE(s.has_block("prev_action"));
}

TEST(Format3Spec, ReadsTheBlocksInTheirFixedOrder)
{
  const PolicySpec s = load(format3(kAll), "all");
  EXPECT_EQ(s.format, 3);
  EXPECT_EQ(s.obs_blocks, kAll);
  EXPECT_EQ(s.obs_dim, 41 + 2 + 72 + 1);
  EXPECT_DOUBLE_EQ(s.prev_action.v_scale, 0.12);
  EXPECT_DOUBLE_EQ(s.prev_action.w_scale, 1.0);
  EXPECT_EQ(s.lidar_history.frames, 3);
  EXPECT_EQ(s.lidar_history.stride_steps, 4);
  EXPECT_DOUBLE_EQ(s.speed_cap.min, 0.08);
  EXPECT_DOUBLE_EQ(s.speed_cap.max, 0.20);
  EXPECT_DOUBLE_EQ(s.speed_cap.obs_scale, 0.12);
  EXPECT_EQ(pode_rl_controller::expected_obs_layout(s).size(), 116u);
  EXPECT_EQ(load(format3({"speed_cap"}), "cap").obs_dim, 42);
  EXPECT_EQ(load(format3({"prev_action", "speed_cap"}), "prev_cap").obs_dim, 44);
}

TEST(Format3Spec, RefusesWhatThePluginCannotBuild)
{
  const std::vector<std::pair<std::string, std::function<YAML::Node()>>> cases{
    {"intent", [] {return format3({"prev_action", "intent"});}},
    {"order", [] {return format3({"speed_cap", "prev_action"});}},
    {"duplicate", [] {return format3({"prev_action", "prev_action"});}},
    {"unknown", [] {return format3({"odometry"});}},
    {"dim", [] {auto s = format3(kAll); s["obs_dim"] = 115; return s;}},
    {"layout", [] {auto s = format3(kAll); s["obs_layout"][41] = "prev_x"; return s;}},
    {"frames", [] {return format3({"lidar_history"}, 1, 4);}},
    {"stride", [] {return format3({"lidar_history"}, 3, 0);}},
    {"cap_range", [] {auto s = format3({"speed_cap"}); s["speed_cap"]["min"] = 0.3; return s;}},
    {"cap_zero", [] {auto s = format3({"speed_cap"}); s["speed_cap"]["min"] = 0.0; return s;}},
    {"prev_scale", [] {auto s = format3({"prev_action"}); s["prev_action"]["v_scale"] = 0.2;
        return s;}},
    {"cap_scale", [] {auto s = format3({"speed_cap"}); s["speed_cap"]["obs_scale"] = 0.2;
        return s;}},
    {"format2_blocks", [] {auto s = format3({"speed_cap"}); s["format"] = 2; return s;}},
    {"format4", [] {auto s = format3(kAll); s["format"] = 4; return s;}},
  };
  for (const auto & [name, make] : cases) {
    EXPECT_NE(refusal(make(), name), "") << name;
  }
  EXPECT_NE(refusal(format3({"intent"}), "intent_only").find("intent"), std::string::npos);
}

TEST(ObsMemory, EpisodeStartHasZeroPrevActionAndTheCurrentFrameEverywhere)
{
  const PolicySpec s = load(format3(kAll), "start");
  ObsMemory mem(s);
  const auto extra = mem.blocks(frame(0.7f), 0.12);
  ASSERT_EQ(extra.size(), 2u + 72u + 1u);
  EXPECT_FLOAT_EQ(extra[0], 0.0f);
  EXPECT_FLOAT_EQ(extra[1], 0.0f);
  for (size_t i = 2; i < 74; ++i) {
    EXPECT_FLOAT_EQ(extra[i], 0.7f) << i;
  }
  EXPECT_FLOAT_EQ(extra[74], 1.0f);
  EXPECT_FLOAT_EQ(mem.blocks(frame(0.7f), 0.09)[74], 0.75f);
}

TEST(ObsMemory, HistorySlotsAreStrideStepsApartNewestFirst)
{
  const PolicySpec s = load(format3({"lidar_history"}), "stride");
  ObsMemory mem(s);
  for (int t = 0; t < 12; ++t) {
    const auto extra = mem.blocks(frame(static_cast<float>(t)), 0.12);
    ASSERT_EQ(extra.size(), 72u);
    for (int f = 1; f <= 2; ++f) {
      const float want = static_cast<float>(std::max(t - 4 * f, 0));
      EXPECT_FLOAT_EQ(extra[(f - 1) * 36], want) << "t " << t << " slot " << f;
      EXPECT_FLOAT_EQ(extra[(f - 1) * 36 + 35], want) << "t " << t << " slot " << f;
    }
    mem.step(frame(static_cast<float>(t)), {0.0, 0.0});
  }
}

TEST(ObsMemory, PrevActionIsTheScaledTargetBeforeTheRateLimit)
{
  const PolicySpec s = load(format3({"prev_action"}), "prev");
  ObsMemory mem(s);
  mem.step(frame(1.0f), {0.06, -0.5});
  const auto extra = mem.blocks(frame(1.0f), 0.12);
  ASSERT_EQ(extra.size(), 2u);
  EXPECT_FLOAT_EQ(extra[0], 0.5f);
  EXPECT_FLOAT_EQ(extra[1], -0.5f);
  mem.reset();
  EXPECT_FLOAT_EQ(mem.blocks(frame(1.0f), 0.12)[0], 0.0f);
}

TEST(ObsMemory, ResetReturnsToTheEpisodeStart)
{
  const PolicySpec s = load(format3(kAll), "reset");
  ObsMemory mem(s);
  for (int t = 0; t < 9; ++t) {
    mem.step(frame(static_cast<float>(t)), {0.1, 0.3});
  }
  mem.reset();
  ObsMemory fresh(s);
  EXPECT_EQ(mem.blocks(frame(0.4f), 0.12), fresh.blocks(frame(0.4f), 0.12));
}

TEST(ObsMemory, Format2HasNoExtraEntries)
{
  const PolicySpec s = pode_rl_controller::load_policy_spec(
    std::string(POLICY_DIR) + "/policy_spec.json");
  ObsMemory mem(s);
  EXPECT_TRUE(mem.blocks(frame(0.5f), 0.12).empty());
}

TEST(SpeedCap, ClampsTheSourceToTheTrainedRangeAndSaysWhenOutside)
{
  const PolicySpec s = load(format3({"speed_cap"}), "capval");
  const auto in = pode_rl_controller::speed_cap_value(0.12, s.speed_cap);
  EXPECT_DOUBLE_EQ(in.value, 0.12);
  EXPECT_EQ(in.error, "");
  const auto low = pode_rl_controller::speed_cap_value(0.05, s.speed_cap);
  EXPECT_DOUBLE_EQ(low.value, 0.08);
  EXPECT_NE(low.error, "");
  const auto high = pode_rl_controller::speed_cap_value(0.25, s.speed_cap);
  EXPECT_DOUBLE_EQ(high.value, 0.20);
  EXPECT_NE(high.error, "");
}

TEST(SpeedCap, ActionMapsOntoTheCapInPlaceOfVMax)
{
  const auto lim = pode_rl_controller::load_policy_spec(
    std::string(POLICY_DIR) + "/policy_spec.json").action;
  for (const float a : {-1.0f, -0.3f, 0.0f, 0.6f, 1.0f}) {
    const Twist2 same = pode_rl_controller::action_to_target(a, 0.2f, lim, lim.v_max);
    const Twist2 today = pode_rl_controller::action_to_target(a, 0.2f, lim);
    EXPECT_DOUBLE_EQ(same.v, today.v);
    EXPECT_DOUBLE_EQ(same.w, today.w);
  }
  EXPECT_DOUBLE_EQ(pode_rl_controller::action_to_target(1.0f, 0.0f, lim, 0.08).v, 0.08);
  EXPECT_DOUBLE_EQ(pode_rl_controller::action_to_target(-1.0f, 0.0f, lim, 0.08).v, lim.v_min);
  EXPECT_DOUBLE_EQ(
    pode_rl_controller::action_to_target(0.0f, 0.0f, lim, 0.08).v, 0.5 * (lim.v_min + 0.08));
}

TEST(EpisodeRules, MemoryIsStaleWhenThePolicyMissedAStep)
{
  EXPECT_TRUE(pode_rl_controller::memory_stale(false, 0.0, 0.05));
  EXPECT_FALSE(pode_rl_controller::memory_stale(true, 0.05, 0.05));
  EXPECT_FALSE(pode_rl_controller::memory_stale(true, 0.099, 0.05));
  EXPECT_TRUE(pode_rl_controller::memory_stale(true, 0.11, 0.05));
}

TEST(EpisodeRules, ANewEpisodeStartsOnlyWhenTheGoalMoves)
{
  using pode_rl_controller::Point2;
  EXPECT_TRUE(pode_rl_controller::new_episode(false, {0.0, 0.0}, {1.0, 1.0}));
  EXPECT_FALSE(pode_rl_controller::new_episode(true, {1.0, 1.0}, {1.0, 1.0}));
  EXPECT_FALSE(pode_rl_controller::new_episode(true, {1.0, 1.0}, {1.0005, 1.0}));
  EXPECT_TRUE(pode_rl_controller::new_episode(true, {1.0, 1.0}, {1.2, 1.0}));
}

// ---- 레시피(task_spec.py, store_arena_env.lidar_history)와 관측 일치 ----
TEST(Format3Parity, ObservationAndTargetMatchTheRecipe)
{
  const YAML::Node f = YAML::LoadFile(std::string(FORMAT3_FIXTURE));
  const YAML::Node steps = f["steps"];
  ASSERT_EQ(f["sets"].size(), 4u);
  for (const auto & kv : f["sets"]) {
    const std::string name = kv.first.as<std::string>();
    const YAML::Node set = kv.second;
    YAML::Node s = base_spec();
    s["format"] = 3;
    for (const char * key : {"obs_blocks", "prev_action", "lidar_history", "speed_cap",
        "obs_layout", "obs_dim"})
    {
      if (set[key]) {
        s[key] = set[key];
      }
    }
    const PolicySpec spec = load(s, "parity_" + name);
    const bool capped = spec.has_block("speed_cap");
    ObsMemory memory(spec);
    ASSERT_EQ(set["obs"].size(), steps.size());
    for (size_t k = 0; k < steps.size(); ++k) {
      const YAML::Node st = steps[k];
      if (st["reset"].as<bool>()) {
        memory.reset();
      }
      const double cap = capped ? st["speed_cap"].as<double>() : spec.action.v_max;
      std::vector<float> obs = pode_rl_controller::build_observation(
        st["rays"].as<std::vector<float>>(), st["goal_dist"].as<double>(),
        st["goal_angle"].as<double>(), st["v"].as<double>(), st["w"].as<double>(), spec);
      const std::vector<float> sectors(obs.begin(), obs.begin() + 36);
      const std::vector<float> extra = memory.blocks(sectors, cap);
      obs.insert(obs.end(), extra.begin(), extra.end());
      const auto want = set["obs"][k].as<std::vector<double>>();
      ASSERT_EQ(obs.size(), want.size()) << name << " step " << k;
      for (size_t i = 0; i < want.size(); ++i) {
        EXPECT_NEAR(obs[i], want[i], 1e-6) << name << " step " << k << " entry " << i;
      }
      const auto a = st["action"].as<std::vector<float>>();
      const Twist2 target = pode_rl_controller::action_to_target(a[0], a[1], spec.action, cap);
      const auto t = set["target"][k].as<std::vector<double>>();
      EXPECT_NEAR(target.v, t[0], 1e-6) << name << " step " << k;
      EXPECT_NEAR(target.w, t[1], 1e-6) << name << " step " << k;
      memory.step(sectors, target);
    }
  }
}
