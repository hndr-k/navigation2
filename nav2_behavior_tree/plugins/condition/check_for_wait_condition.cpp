// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

#include "nav2_behavior_tree/plugins/condition/check_for_wait_condition.hpp"

namespace nav2_behavior_tree {

CheckForWait::CheckForWait(const std::string &condition_name,
                           const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf) {

  // node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus CheckForWait::tick() {
  // node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // RCLCPP_INFO(rclcpp::get_logger("Wait Check"), "checking for wait");

  getInput("mapf_poses", goals_);

  if (goals_.size() > 2) {
    if (posesEqual(goals_[0], goals_[1])) {
      RCLCPP_INFO(rclcpp::get_logger("Wait Check"), "waiting required");

      return BT::NodeStatus::FAILURE;
    }
  }
  // RCLCPP_INFO(rclcpp::get_logger("Wait Check"), "waiting not required");

  return BT::NodeStatus::SUCCESS;
}

bool CheckForWait::posesEqual(geometry_msgs::msg::PoseStamped pose_zero,
                              geometry_msgs::msg::PoseStamped pose_one) {
  if ((pose_zero.pose.position.x == pose_one.pose.position.x) &&
      (pose_zero.pose.position.y == pose_one.pose.position.y)) {
    return true;
  }

  return false;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::CheckForWait>("CheckForWait");
}
