// Copyright (c) 2019 Samsung Research America
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

#include "nav2_behavior_tree/plugins/action/call_mapf_service.hpp"

namespace nav2_behavior_tree {

CallMapfService::CallMapfService(const std::string &xml_tag_name,
                                 const std::string &action_name,
                                 const BT::NodeConfiguration &conf)
    : BtActionNode<mapf_actions::action::Mapf>(xml_tag_name, action_name,
                                               conf) {
      getInput("global_frame", global_frame_);
      getInput("robot_base_frame", robot_base_frame_);
      tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
      auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      node->get_parameter("transform_tolerance", transform_tolerance_);             
     }

void CallMapfService::on_tick() {
  int id;
  geometry_msgs::msg::PoseStamped goal;
  geometry_msgs::msg::PoseStamped start;
  nav2_util::getCurrentPose(
      start, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_);
  getInput("rob_id", id);
  getInput("goal", goal);

  goal_.robotino_id = id;
  goal_.goal = goal;
  goal_.start = start;
  /*getInput("ID", request_->robotino_id);
  getInput("goal", request_->goal);
  if(request_sent_)
  {
       setOutput("goals", future_result_.get()->path);
  }*/
  increment_recovery_count();
}
BT::NodeStatus CallMapfService::on_success()
{
  setOutput("path", result_.result->path);
  return BT::NodeStatus::SUCCESS;
}
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<nav2_behavior_tree::CallMapfService>(
        name, "CallMapfService", config);
  };

  factory.registerBuilder<nav2_behavior_tree::CallMapfService>(
      "CallMapfService", builder);
}
