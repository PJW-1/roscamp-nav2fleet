#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "pode_rl_controller/observation.hpp"
#include "pode_rl_controller/policy_spec.hpp"

using pode_rl_controller::Point2;
using pode_rl_controller::PolicySpec;

namespace
{
constexpr double kObsTol = 1e-6;
constexpr double kGeomTol = 1e-9;

PolicySpec spec(const std::string & dir = POLICY_DIR)
{
  return pode_rl_controller::load_policy_spec(dir + "/policy_spec.json");
}

YAML::Node follow_fixture()
{
  return YAML::LoadFile(std::string(FIXTURE_DIR) + "/follow_cases.json");
}

std::vector<Point2> path_of(const YAML::Node & c)
{
  std::vector<Point2> path;
  for (const auto & p : c["path"]) {
    path.push_back({p[0].as<double>(), p[1].as<double>()});
  }
  return path;
}

double gap(double got, double want)
{
  const double d = std::abs(got - want);
  return std::isfinite(d) ? d : std::numeric_limits<double>::infinity();
}

double angle_gap(double got, double want)
{
  return gap(std::remainder(got - want, 2.0 * M_PI), 0.0);
}

std::vector<Point2> line(int n, double step)
{
  std::vector<Point2> path;
  for (int i = 0; i < n; ++i) {
    path.push_back({i * step, 0.0});
  }
  return path;
}
}  // namespace

// ---- 명세의 목표점 규칙 ----
TEST(PolicySpec, GoalRuleComesFromTheSpec)
{
  const PolicySpec s = spec();
  EXPECT_EQ(s.goal_rule.rule, "lookahead");
  EXPECT_DOUBLE_EQ(s.goal_rule.lookahead_m, 0.40);
  EXPECT_EQ(s.goal_rule.follow_window, 20);
  const auto f = follow_fixture();
  EXPECT_DOUBLE_EQ(f["lookahead_m"].as<double>(), s.goal_rule.lookahead_m);
  EXPECT_EQ(f["follow_window"].as<int>(), s.goal_rule.follow_window);
  EXPECT_EQ(spec(PATH_END_POLICY_DIR).goal_rule.rule, "path_end");
}

// ---- 경로 위 앞보기 점: route_spec.follow와 같은가 ----
TEST(Follow, MatchesRouteSpec)
{
  const PolicySpec s = spec();
  double worst_point = 0.0;
  int steps = 0;
  for (const auto & c : follow_fixture()["cases"]) {
    const auto path = path_of(c);
    size_t prog = 0;
    for (const auto & st : c["steps"]) {
      const Point2 robot{st["xy"][0].as<double>(), st["xy"][1].as<double>()};
      prog = pode_rl_controller::follow_progress(path, prog, robot, s.goal_rule.follow_window);
      ASSERT_EQ(prog, st["prog"].as<size_t>()) << c["name"].as<std::string>() << " step " << steps;
      const Point2 la =
        path[pode_rl_controller::lookahead_index(path, prog, s.goal_rule.lookahead_m)];
      worst_point = std::max(
        {worst_point, gap(la.x, st["lookahead_xy"][0].as<double>()),
          gap(la.y, st["lookahead_xy"][1].as<double>())});
      ++steps;
    }
  }
  std::printf("[follow] %d steps, lookahead point max |diff| %.3g m\n", steps, worst_point);
  EXPECT_GT(steps, 40);
  EXPECT_LE(worst_point, kGeomTol);
}

TEST(Follow, PathGoalMatchesTaskSpecInThePlanFrame)
{
  const PolicySpec s = spec();
  double worst = 0.0;
  int steps = 0;
  for (const auto & c : follow_fixture()["cases"]) {
    const auto path = path_of(c);
    size_t prog = 0;
    for (const auto & st : c["steps"]) {
      const Point2 robot{st["xy"][0].as<double>(), st["xy"][1].as<double>()};
      const auto g =
        pode_rl_controller::path_goal(path, prog, robot, st["yaw"].as<double>(), s.goal_rule);
      ASSERT_EQ(prog, st["prog"].as<size_t>());
      worst = std::max(
        {worst, gap(g.dist, st["goal_dist"].as<double>()),
          angle_gap(g.angle, st["goal_angle"].as<double>())});
      ++steps;
    }
  }
  std::printf("[follow] path_goal over %d steps, goal max |diff| %.3g\n", steps, worst);
  EXPECT_GT(steps, 40);
  EXPECT_LE(worst, kGeomTol);
}

TEST(Follow, PathEndRuleTakesTheLastPoint)
{
  const auto path = line(81, 0.025);
  size_t prog = 0;
  const auto g = pode_rl_controller::path_goal(
    path, prog, {0.0, 1.0}, 0.0, pode_rl_controller::GoalRule{"path_end", 0.0, 0});
  EXPECT_NEAR(g.dist, std::hypot(2.0, 1.0), kGeomTol);
  EXPECT_NEAR(g.angle, std::atan2(-1.0, 2.0), kGeomTol);
  EXPECT_EQ(prog, 0u);
}

TEST(Follow, PathShorterThanLookaheadGivesItsEnd)
{
  const auto path = line(9, 0.025);
  EXPECT_EQ(pode_rl_controller::lookahead_index(path, 0, 0.40), 8u);
  EXPECT_EQ(pode_rl_controller::lookahead_index(path, 4, 0.40), 8u);
}

TEST(Follow, EndOfPathGivesItsEnd)
{
  const auto path = line(81, 0.025);
  EXPECT_EQ(pode_rl_controller::lookahead_index(path, 0, 0.40), 16u);
  EXPECT_EQ(pode_rl_controller::lookahead_index(path, 70, 0.40), 80u);
  EXPECT_EQ(pode_rl_controller::lookahead_index(path, 80, 0.40), 80u);
  EXPECT_EQ(pode_rl_controller::follow_progress(path, 80, {5.0, 0.0}, 20), 80u);
}

TEST(Follow, ProgressMovesAtMostTheWindowAndNeverBack)
{
  const auto path = line(81, 0.025);
  EXPECT_EQ(pode_rl_controller::follow_progress(path, 10, {1.5, 0.0}, 20), 30u);
  EXPECT_EQ(pode_rl_controller::follow_progress(path, 30, {0.0, 0.0}, 20), 30u);
}

// ---- 목표와 속도 관측: task_spec과 같은가 ----
TEST(GoalAndVelocity, MatchTaskSpec)
{
  const PolicySpec s = spec();
  const auto rays = std::vector<float>(s.lidar.rays, static_cast<float>(s.lidar.obs_max));
  double worst_polar = 0.0;
  double worst_obs = 0.0;
  int steps = 0;
  for (const auto & c : follow_fixture()["cases"]) {
    for (const auto & st : c["steps"]) {
      ++steps;
      const Point2 robot{st["xy"][0].as<double>(), st["xy"][1].as<double>()};
      const Point2 la{st["lookahead_xy"][0].as<double>(), st["lookahead_xy"][1].as<double>()};
      const auto g = pode_rl_controller::goal_in_robot_frame(robot, st["yaw"].as<double>(), la);
      worst_polar = std::max(
        {worst_polar, gap(g.dist, st["goal_dist"].as<double>()),
          angle_gap(g.angle, st["goal_angle"].as<double>())});
      const auto obs = pode_rl_controller::build_observation(
        rays, g.dist, g.angle, st["v"].as<double>(), st["w"].as<double>(), s);
      const auto want = st["obs_goal_vel"].as<std::vector<double>>();
      ASSERT_EQ(want.size(), 5u);
      for (size_t k = 0; k < want.size(); ++k) {
        worst_obs = std::max(worst_obs, gap(obs[s.lidar.sectors + k], want[k]));
      }
    }
  }
  std::printf("[goal] polar max |diff| %.3g, goal and velocity obs max |diff| %.3g\n",
    worst_polar, worst_obs);
  EXPECT_GT(steps, 40);
  EXPECT_LE(worst_polar, kGeomTol);
  EXPECT_LE(worst_obs, kObsTol);
}

// ---- 라이다가 yaw pi로 달린 Gazebo 배치 ----
TEST(Lidar, YawPiMountPutsTheLidarBackAtRayZero)
{
  const PolicySpec s = spec();
  const int n = 640;
  pode_rl_controller::Scan scan{-M_PI, 2.0 * M_PI / (n - 1), 0.05, 12.0,
    std::vector<float>(n, std::numeric_limits<float>::infinity())};
  scan.ranges[0] = 0.5f;
  scan.ranges[160] = 0.8f;
  scan.ranges[320] = 1.2f;
  const auto rays = pode_rl_controller::scan_to_rays(scan, M_PI, s);
  EXPECT_FLOAT_EQ(rays[180], 0.5f);
  EXPECT_FLOAT_EQ(rays[270], 0.8f);
  EXPECT_FLOAT_EQ(rays[0], 1.2f);
  EXPECT_FLOAT_EQ(rays[90], 2.0f);
  const auto obs = pode_rl_controller::build_observation(rays, 1.0, 0.0, 0.0, 0.0, s);
  EXPECT_FLOAT_EQ(obs[18], 0.25f);
  EXPECT_FLOAT_EQ(obs[27], 0.4f);
  EXPECT_FLOAT_EQ(obs[0], 0.6f);
  EXPECT_FLOAT_EQ(obs[9], 1.0f);
  const auto unturned = pode_rl_controller::scan_to_rays(scan, 0.0, s);
  EXPECT_FLOAT_EQ(unturned[180], 1.2f);
  EXPECT_FLOAT_EQ(unturned[0], 0.5f);
}
