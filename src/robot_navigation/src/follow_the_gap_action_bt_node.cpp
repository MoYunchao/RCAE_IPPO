#include "robot_navigation/follow_the_gap_action_bt_node.hpp"

namespace nav2_behavior_tree
{

FollowTheGapAction::FollowTheGapAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<follow_the_gap_recovery::action::FollowTheGap>(xml_tag_name, action_name, conf)
{
}

void FollowTheGapAction::on_tick()
{
  double time_allowance;
  getInput("time_allowance", time_allowance);
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"

// Manual registration to ensure symbol export
extern "C" {
    __attribute__((visibility("default")))
    void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
    {
        BT::NodeBuilder builder =
            [](const std::string & name, const BT::NodeConfiguration & config)
            {
            return std::make_unique<nav2_behavior_tree::FollowTheGapAction>(
                name, "follow_the_gap", config);
            };

        factory.registerBuilder<nav2_behavior_tree::FollowTheGapAction>("FollowTheGap", builder);
    }
}
