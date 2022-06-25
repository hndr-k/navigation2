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
class UseMapf : public BT::ConditionNode {
public:
  /**
   * @brief A constructor for nav2_behavior_tree::UseMapf
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  UseMapf(const std::string &condition_name, const BT::NodeConfiguration &conf);

  UseMapf() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() {
    return {BT::InputPort<bool>("use_mapf", "mapf plan used")};
  }

private:
  bool use_mapf;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TIME_EXPIRED_CONDITION_HPP_