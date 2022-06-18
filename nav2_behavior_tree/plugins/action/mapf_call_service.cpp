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
  /*robot_base_frame_ = "robotino";
  robot_base_frame_.append(std::to_string(request_->robotino_id + 1));
  robot_base_frame_.append("base_link");
  RCLCPP_INFO(rclcpp::get_logger("MAPF"), "base frame new: %s",
              robot_base_frame_.c_str());*/
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
  current_mapf_ = future_result_.get()->path.poses[0];
  if (future_result_.get()->path.poses.size() > 1) {
    current_mapf_ = future_result_.get()->path.poses[1];
  }
  current_mapf_.header.frame_id = "map";
  RCLCPP_INFO(rclcpp::get_logger("MAPF"), "X: %4.2f, Y: %4.2f",
              current_mapf_.pose.position.x, current_mapf_.pose.position.y);
  setOutput("mapf_goal", current_mapf_);
  return BT::NodeStatus::SUCCESS;
}
} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::MapfCallService>(
      "MapfCallService");
}
