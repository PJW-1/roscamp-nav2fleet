#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pode_rl_controller/observation.hpp"
#include "pode_rl_controller/onnx_policy.hpp"
#include "pode_rl_controller/policy_spec.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace pode_rl_controller
{

// ---- Nav2 controller 플러그인 FollowPathRL ----
class FollowPathRL : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

private:
  double lidar_yaw(const std::string & frame);
  void goals(GoalPolar & goal, GoalPolar & end, double & yaw_end);

  std::string name_;
  std::string base_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("FollowPathRL")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  PolicySpec spec_{};
  std::unique_ptr<OnnxPolicy> policy_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  std::string scan_topic_;
  std::string lidar_frame_;
  double lidar_yaw_{0.0};
  nav_msgs::msg::Path plan_;
  std::vector<Point2> path_;
  size_t prog_{0};
  GoalRule rule_{};
  double scan_timeout_{0.5};
  double transform_tolerance_{0.2};
  double rotate_gain_{1.5};
  double log_period_{1.0};
  double rear_stop_range_{0.0};
  double rear_half_width_deg_{30.0};
  int rear_blocks_{0};
  double speed_limit_{0.0};
  Twist2 cmd_{0.0, 0.0};
  rclcpp::Time last_call_{0, 0, RCL_ROS_TIME};
  bool have_last_call_{false};
  rclcpp::Time last_log_{0, 0, RCL_ROS_TIME};
  int calls_{0};
  double infer_sum_us_{0.0};
  double infer_max_us_{0.0};
  double call_sum_us_{0.0};
  double call_max_us_{0.0};
};

}  // namespace pode_rl_controller
