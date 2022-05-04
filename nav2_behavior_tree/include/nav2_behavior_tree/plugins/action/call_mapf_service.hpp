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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CALL_MAPF_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CALL_MAPF_SERVICE_HPP_

#include <string>


#include "nav2_behavior_tree/bt_service_node.hpp"
#include "mapf_actions/srv/mapf.hpp"
#include "mapf_actions/action/mapf.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_bt_navigator/navigator.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps mapf_actions::srv::Mapf
 */
class CallMapfService : public BtActionNode<mapf_actions::action::Mapf>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::CallMapfService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  CallMapfService(
    const std::string & service_node_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;
  BT::NodeStatus on_success();
  BT::NodeStatus on_completion();
  rclcpp::Node::SharedPtr node_;

  double viapoint_achieved_radius_;
  std::string robot_base_frame_, global_frame_;
  double transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
    static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<int>("rob_id"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose", "Origin"),
        BT::OutputPort<nav_msgs::msg::Path>("goals", "Destination to plan to"),
        BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
        BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      });
  }
};


}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_SERVICE_HPP_