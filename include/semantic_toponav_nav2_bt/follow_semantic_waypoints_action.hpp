// Copyright 2026 semantic-toponav contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
// implied. See the License for the specific language governing
// permissions and limitations under the License.

#ifndef SEMANTIC_TOPONAV_NAV2_BT__FOLLOW_SEMANTIC_WAYPOINTS_ACTION_HPP_
#define SEMANTIC_TOPONAV_NAV2_BT__FOLLOW_SEMANTIC_WAYPOINTS_ACTION_HPP_

#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "semantic_toponav_msgs/msg/semantic_waypoint_array.hpp"

namespace semantic_toponav_nav2_bt
{

/// @brief BT action node that bridges a v1.0 SemanticWaypointArray to
///        Nav2's NavigateThroughPoses action.
///
/// Inherits from nav2_behavior_tree::BtActionNode so it integrates with
/// the same action-client lifecycle the rest of the Nav2 BT primitives
/// use (cancel-on-halt, feedback ports, server-side timeouts).
///
/// Blackboard inputs:
///   - "waypoints" (semantic_toponav_msgs::msg::SemanticWaypointArray):
///     The semantic plan to follow. Required.
///   - "server_name" (std::string, default "navigate_through_poses"):
///     The action server to dispatch to. Override when running multiple
///     navigation stacks side-by-side.
///   - "server_timeout" (std::chrono::milliseconds, default 1000):
///     Time to wait for the action server to come up before failing.
///
/// Blackboard outputs:
///   - "n_poses_dispatched" (int): How many poses the BT actually sent
///     to NavigateThroughPoses (waypoints with has_pose=false are
///     skipped). Set on SUCCEEDED.
///   - "current_waypoint_index" (int): Zero-based index of the
///     dispatched pose Nav2 is currently progressing toward, derived
///     from feedback as
///     n_poses_dispatched - number_of_poses_remaining. -1 before the
///     first feedback arrives or when the BT cannot infer it. Updated
///     on every feedback tick — upper BTs can branch on progress
///     under a ReactiveSequence / ReactiveFallback.
///   - "number_of_poses_remaining" (int): Nav2 NavigateThroughPoses
///     feedback, forwarded verbatim. -1 before the first feedback.
///   - "distance_remaining" (double): Nav2 feedback, meters. NaN
///     before the first feedback.
///   - "number_of_recoveries" (int): Nav2 feedback. -1 before the
///     first feedback.
class FollowSemanticWaypointsAction
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateThroughPoses>
{
public:
  using Action = nav2_msgs::action::NavigateThroughPoses;
  using WaypointArray = semantic_toponav_msgs::msg::SemanticWaypointArray;

  FollowSemanticWaypointsAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /// Populate the NavigateThroughPoses goal from the blackboard's
  /// SemanticWaypointArray input. Called by the parent class on each
  /// activation, before the action client sends the goal.
  void on_tick() override;

  /// Forward the latest NavigateThroughPoses feedback to the
  /// blackboard outputs (current_waypoint_index, distance_remaining,
  /// number_of_poses_remaining, number_of_recoveries) so upper BTs
  /// can branch on progress while the action is RUNNING. Called by
  /// the parent class each time the action server publishes feedback,
  /// with that feedback handed in as the argument.
  void on_wait_for_result(
    std::shared_ptr<const Action::Feedback> feedback) override;

  /// Surface the dispatched-pose count on the blackboard when the
  /// action server returns SUCCEEDED.
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<WaypointArray>(
          "waypoints",
          "Semantic plan produced by the semantic-toponav planner."),
        BT::OutputPort<int>(
          "n_poses_dispatched",
          "Count of poses actually dispatched to NavigateThroughPoses."),
        BT::OutputPort<int>(
          "current_waypoint_index",
          "Zero-based index of the dispatched pose Nav2 is currently "
          "progressing toward (derived from feedback). -1 before the "
          "first feedback."),
        BT::OutputPort<int>(
          "number_of_poses_remaining",
          "NavigateThroughPoses feedback, forwarded verbatim. -1 "
          "before the first feedback."),
        BT::OutputPort<double>(
          "distance_remaining",
          "Distance to the current goal in meters, from feedback. "
          "NaN before the first feedback."),
        BT::OutputPort<int>(
          "number_of_recoveries",
          "Nav2 feedback, forwarded verbatim. -1 before the first "
          "feedback."),
      });
  }

private:
  /// Convert one SemanticWaypoint to a PoseStamped. Returns std::nullopt
  /// when the waypoint has no usable pose (has_pose=false or empty
  /// frame_id).
  std::optional<geometry_msgs::msg::PoseStamped>
  to_pose_stamped(
    const semantic_toponav_msgs::msg::SemanticWaypoint & wp,
    const std_msgs::msg::Header & header) const;

  int n_poses_dispatched_ = 0;
};

}  // namespace semantic_toponav_nav2_bt

#endif  // SEMANTIC_TOPONAV_NAV2_BT__FOLLOW_SEMANTIC_WAYPOINTS_ACTION_HPP_
