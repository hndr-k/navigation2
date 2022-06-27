#include "nav2_behavior_tree/plugins/action/mapf_call_service.hpp"
#include <memory>
#include <string>

namespace nav2_behavior_tree {
MapfCallService::MapfCallService(const std::string &service_node_name,
                                 const BT::NodeConfiguration &conf)
    : BtServiceNode<mapf_actions::srv::Mapf>(service_node_name, conf) {
  RCLCPP_INFO(rclcpp::get_logger("MAPF"), "MAPF call instantiated");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->declare_parameter("robotino_frame", std::string("base_link"));
  node->declare_parameter("robotino_id", 2);
}

void MapfCallService::on_tick() {
  getInput("goal", request_->goal);
  getInput("global_frame", global_frame_);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("robotino_frame", robot_base_frame_);
  node->get_parameter("robotino_id", request_->robotino_id);
  RCLCPP_INFO(rclcpp::get_logger("MAPF"), "base_frame: %s ",
              robot_base_frame_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("MAPF"), "robotino id : %d",
              request_->robotino_id);
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_,
                                 robot_base_frame_, 0.2)) {
    RCLCPP_INFO(node->get_logger(),
                "MAPF call unsuccesfull no localization possible");
  }
  request_->start = current_pose;
}
BT::NodeStatus MapfCallService::on_completion() {
  geometry_msgs::msg::PoseStamped current_mapf_;
  if (future_result_.get()->path.poses.size() > 1) {
    current_mapf_ = future_result_.get()->path.poses[1];
    setOutput("use_mapf", true);
  } else if (future_result_.get()->path.poses.size() == 1) {
    current_mapf_ = future_result_.get()->path.poses[0];
    setOutput("use_mapf", true);
  } else {
    setOutput("use_mapf", false);
    return BT::NodeStatus::FAILURE;
  }
  current_mapf_.header.frame_id = "map";
  RCLCPP_INFO(rclcpp::get_logger("MAPF"), "X: %4.2f, Y: %4.2f",
              current_mapf_.pose.position.x, current_mapf_.pose.position.y);

  setOutput("mapf_goal", current_mapf_);
  setOutput("mapf_poses", future_result_.get()->path.poses);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MapfCallService::check_future() {
  auto node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  auto elapsed =
      (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
  auto remaining = server_timeout_ - elapsed;

  if (remaining > std::chrono::milliseconds(0)) {
    auto timeout =
        remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;

    rclcpp::FutureReturnCode rc;
    rc = callback_group_executor_.spin_until_future_complete(future_result_,
                                                             server_timeout_);
    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      request_sent_ = false;
      BT::NodeStatus status = on_completion();
      return status;
    }

    if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
      on_wait_for_result();
      elapsed =
          (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
      if (elapsed < server_timeout_) {
        return BT::NodeStatus::RUNNING;
      }
    }
  }

  RCLCPP_WARN(node_->get_logger(),
              "Node timed out while executing service call to %s.",
              service_name_.c_str());
  request_sent_ = false;
  setOutput("use_mapf", false);
  return BT::NodeStatus::FAILURE;
}
} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::MapfCallService>(
      "MapfCallService");
}
