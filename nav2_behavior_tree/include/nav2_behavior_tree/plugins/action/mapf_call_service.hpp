#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "mapf_actions/srv/mapf.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree {
class MapfCallService : public BtServiceNode<mapf_actions::srv::Mapf> {
public:
  MapfCallService(const std::string &service_node_name,
                  const BT::NodeConfiguration &conf);

  void on_tick() override;
  BT::NodeStatus on_completion() override;
  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::InputPort<int>("identifier",
                           "idenftifier of robototino for MAPF request"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
            "goal", "Destination to plan to"),
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
            "mapf_goals", "Destinations to plan through"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("mapf_goal",
                                                        "Current Mapf Goal"),
        BT::InputPort<std::string>("global_frame", std::string("map"),
                                   "Global frame"),
        BT::InputPort<std::string>("robot_base_frame", std::string("base_link"),
                                   "Robot base frame"),

    });
  }

  std::string robot_base_frame_, global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  geometry_msgs::msg::PoseStamped goal_pose_;
};
} // namespace nav2_behavior_tree