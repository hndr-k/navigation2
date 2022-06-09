#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TIME_EXPIRED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TIME_EXPIRED_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
namespace nav2_behavior_tree {

/**
 * @brief A BT::ConditionNode that returns SUCCESS if two poses are the same
 * within the goal array
 */
class CheckForWait : public BT::ConditionNode {
public:
  /**
   * @brief A constructor for nav2_behavior_tree::CheckForWait
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  CheckForWait(const std::string &condition_name,
               const BT::NodeConfiguration &conf);

  CheckForWait() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  bool posesEqual(geometry_msgs::msg::PoseStamped pose_zero,
                  geometry_msgs::msg::PoseStamped pose_one);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "mapf_goals", "incoming mapf goals")};
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TIME_EXPIRED_CONDITION_HPP_