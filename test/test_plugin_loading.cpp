// Copyright 2026 semantic-toponav contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <memory>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "semantic_toponav_nav2_bt/follow_semantic_waypoints_action.hpp"

namespace stb = semantic_toponav_nav2_bt;

// Smoke test: confirm the registered BT node type loads through the BT
// factory and reports the ports we documented in providedPorts().
//
// This deliberately doesn't dispatch a real NavigateThroughPoses goal —
// that needs a running action server and is exercised in the
// integration tests under tests/integration/ instead.
TEST(FollowSemanticWaypointsAction, RegistersAndExposesPorts)
{
  BT::BehaviorTreeFactory factory;

  BT::NodeBuilder builder = [](
    const std::string & name,
    const BT::NodeConfiguration & config)
  {
    return std::make_unique<stb::FollowSemanticWaypointsAction>(
      name, "navigate_through_poses", config);
  };
  factory.registerBuilder<stb::FollowSemanticWaypointsAction>(
    "FollowSemanticWaypoints", builder);

  const auto & manifests = factory.manifests();
  const auto it = std::find_if(
    manifests.begin(), manifests.end(),
    [](const auto & m) {
      return m.registration_ID == "FollowSemanticWaypoints";
    });
  ASSERT_NE(it, manifests.end())
    << "FollowSemanticWaypoints not registered with the factory";

  const auto & ports = it->ports;
  EXPECT_TRUE(ports.count("waypoints"))
    << "missing 'waypoints' blackboard input port";
  EXPECT_TRUE(ports.count("n_poses_dispatched"))
    << "missing 'n_poses_dispatched' blackboard output port";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
