// Copyright 2026 semantic-toponav contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "semantic_toponav_nav2_bt/follow_semantic_waypoints_action.hpp"

#include <optional>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

  RCLCPP_INFO(
    node_->get_logger(),
    "%s: dispatching %d of %zu semantic waypoints to NavigateThroughPoses",
    name().c_str(),
    n_poses_dispatched_,
    waypoints.waypoints.size());
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

}  // namespace semantic_toponav_nav2_bt

#include <behaviortree_cpp_v3/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](
    const std::string & name,
    const BT::NodeConfiguration & config)
  {
    return std::make_unique<
      semantic_toponav_nav2_bt::FollowSemanticWaypointsAction>(
      name, "navigate_through_poses", config);
  };

  factory.registerBuilder<
    semantic_toponav_nav2_bt::FollowSemanticWaypointsAction>(
    "FollowSemanticWaypoints", builder);
}
