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

#include "semantic_toponav_nav2_bt/follow_semantic_waypoints_action.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace semantic_toponav_nav2_bt
{

FollowSemanticWaypointsAction::FollowSemanticWaypointsAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void FollowSemanticWaypointsAction::on_tick()
{
  WaypointArray waypoints;
  if (!getInput("waypoints", waypoints)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "%s: missing required blackboard input 'waypoints'",
      name().c_str());
    // Empty goal makes the action server reject immediately, which
    // propagates as FAILURE up the BT.
    goal_.poses.clear();
    n_poses_dispatched_ = 0;
    return;
  }

  goal_.poses.clear();
  goal_.poses.reserve(waypoints.waypoints.size());
  for (const auto & wp : waypoints.waypoints) {
    auto pose = to_pose_stamped(wp, waypoints.header);
    if (pose.has_value()) {
      goal_.poses.push_back(*pose);
    }
  }
  n_poses_dispatched_ = static_cast<int>(goal_.poses.size());

  // Seed the progress outputs with sentinel values so upper BTs that
  // peek at them before the first feedback arrives see a defined
  // "no progress yet" state instead of stale blackboard contents
  // from a prior tick.
  setOutput("current_waypoint_index", -1);
  setOutput("number_of_poses_remaining", -1);
  setOutput("distance_remaining", std::numeric_limits<double>::quiet_NaN());
  setOutput("number_of_recoveries", -1);

  RCLCPP_INFO(
    node_->get_logger(),
    "%s: dispatching %d of %zu semantic waypoints to NavigateThroughPoses",
    name().c_str(),
    n_poses_dispatched_,
    waypoints.waypoints.size());
}

void FollowSemanticWaypointsAction::on_wait_for_result(
  std::shared_ptr<const Action::Feedback> feedback)
{
  // The base class hands the latest feedback in directly; guard
  // against the pre-first-feedback / spurious-wakeup case where no
  // feedback has been published yet.
  if (feedback == nullptr) {
    return;
  }

  const int remaining = static_cast<int>(feedback->number_of_poses_remaining);
  // current_waypoint_index = (dispatched count) - (poses remaining).
  // When Nav2 hasn't yet started counting down or has progressed
  // beyond the dispatched list (recovery / retry), clamp to a
  // legal in-range index so downstream BTs that read this as an
  // array index don't fault.
  int current_index = n_poses_dispatched_ - remaining;
  if (current_index < 0) {
    current_index = 0;
  }
  if (n_poses_dispatched_ > 0 && current_index >= n_poses_dispatched_) {
    current_index = n_poses_dispatched_ - 1;
  }

  setOutput("current_waypoint_index", current_index);
  setOutput("number_of_poses_remaining", remaining);
  setOutput("distance_remaining", static_cast<double>(feedback->distance_remaining));
  setOutput("number_of_recoveries", static_cast<int>(feedback->number_of_recoveries));
}

BT::NodeStatus FollowSemanticWaypointsAction::on_success()
{
  setOutput("n_poses_dispatched", n_poses_dispatched_);
  return BT::NodeStatus::SUCCESS;
}

std::optional<geometry_msgs::msg::PoseStamped>
FollowSemanticWaypointsAction::to_pose_stamped(
  const semantic_toponav_msgs::msg::SemanticWaypoint & wp,
  const std_msgs::msg::Header & header) const
{
  if (!wp.has_pose) {
    return std::nullopt;
  }

  geometry_msgs::msg::PoseStamped ps;
  ps.header = header;
  if (!wp.frame_id.empty()) {
    ps.header.frame_id = wp.frame_id;
  }
  ps.pose.position.x = wp.pose.x;
  ps.pose.position.y = wp.pose.y;
  ps.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, wp.pose.theta);
  ps.pose.orientation = tf2::toMsg(q);

  return ps;
}

namespace
{
std::unique_ptr<BT::TreeNode> create_follow_semantic_waypoints_node(
  const std::string & name, const BT::NodeConfiguration & config)
{
  return std::make_unique<FollowSemanticWaypointsAction>(
    name, "navigate_through_poses", config);
}
}  // namespace

}  // namespace semantic_toponav_nav2_bt

BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<semantic_toponav_nav2_bt::FollowSemanticWaypointsAction>(
    "FollowSemanticWaypoints",
    semantic_toponav_nav2_bt::create_follow_semantic_waypoints_node);
}
