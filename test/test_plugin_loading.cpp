// Copyright 2026 semantic-toponav contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <memory>
#include <string>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "semantic_toponav_nav2_bt/follow_semantic_waypoints_action.hpp"

namespace stb = semantic_toponav_nav2_bt;

// Smoke test: confirm the FollowSemanticWaypoints node can be
// registered with a BT.CPP factory under its documented name.
//
// We deliberately keep the assertion surface narrow — listing ports
// or instantiating the node would couple this test to BT.CPP v3
// internal map layouts and to a running rclcpp context. Both end-to-
// end interactions (port wiring, NavigateThroughPoses dispatch) are
// exercised in the integration tests under tests/integration/ instead.
TEST(FollowSemanticWaypointsAction, RegistersUnderDocumentedName)
{
  BT::BehaviorTreeFactory factory;

  BT::NodeBuilder builder = [](
    const std::string & name,
    const BT::NodeConfiguration & config)
  {
    return std::make_unique<stb::FollowSemanticWaypointsAction>(
      name, "navigate_through_poses", config);
  };

  EXPECT_NO_THROW({
    factory.registerBuilder<stb::FollowSemanticWaypointsAction>(
      "FollowSemanticWaypoints", builder);
  });

  // registeredBuilders() returns a map<string, NodeBuilder>; using
  // .count() avoids touching iterator-pair internals that vary
  // between BT.CPP v3 patch releases.
  EXPECT_EQ(factory.builders().count("FollowSemanticWaypoints"), 1U)
    << "FollowSemanticWaypoints was not registered with the factory";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
