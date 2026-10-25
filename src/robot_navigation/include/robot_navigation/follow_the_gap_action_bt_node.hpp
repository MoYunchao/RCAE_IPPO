#ifndef FOLLOW_THE_GAP_ACTION_BT_NODE_HPP_
#define FOLLOW_THE_GAP_ACTION_BT_NODE_HPP_

#include <string>
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "follow_the_gap_recovery/action/follow_the_gap.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps follow_the_gap_recovery::action::FollowTheGap
 */
class FollowTheGapAction : public BtActionNode<follow_the_gap_recovery::action::FollowTheGap>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::FollowTheGapAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  FollowTheGapAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for recovery")
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // FOLLOW_THE_GAP_ACTION_BT_NODE_HPP_
