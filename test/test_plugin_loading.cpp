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

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "semantic_toponav_nav2_bt/follow_semantic_waypoints_action.hpp"

namespace stb = semantic_toponav_nav2_bt;

namespace
{
std::unique_ptr<BT::TreeNode> build_test_node(
  const std::string & name, const BT::NodeConfiguration & config)
{
  return std::make_unique<stb::FollowSemanticWaypointsAction>(
    name, "navigate_through_poses", config);
}
}  // namespace

// Smoke test: confirm the FollowSemanticWaypoints node can be
// registered with a BT.CPP factory under its documented name.
//
// We deliberately keep the assertion surface narrow — listing ports
// or instantiating the node would couple this test to BT.CPP v3
// internal map layouts and to a running rclcpp context. Both end-to-
// end interactions (port wiring, NavigateThroughPoses dispatch) are
// exercised against a live action server in
// test/test_integration_navigate_through_poses.cpp instead.
TEST(FollowSemanticWaypointsAction, RegistersUnderDocumentedName)
{
  BT::BehaviorTreeFactory factory;

  EXPECT_NO_THROW(
    factory.registerBuilder<stb::FollowSemanticWaypointsAction>(
      "FollowSemanticWaypoints", build_test_node));

  // registeredBuilders() returns a map<string, NodeBuilder>; using
  // .count() avoids touching iterator-pair internals that vary
  // between BT.CPP v3 patch releases.
  EXPECT_EQ(factory.builders().count("FollowSemanticWaypoints"), 1U)
    << "FollowSemanticWaypoints was not registered with the factory";
}

// providedPorts() is a static contract surface — we can assert the
// documented ports exist without a running rclcpp context or a live
// node instance, so this stays a pure unit check. It guards the v0.2
// feedback-port additions against accidental removal / rename.
TEST(FollowSemanticWaypointsAction, ExposesDocumentedPorts)
{
  const auto ports = stb::FollowSemanticWaypointsAction::providedPorts();

  EXPECT_EQ(ports.count("waypoints"), 1U) << "missing input port 'waypoints'";
  EXPECT_EQ(ports.count("n_poses_dispatched"), 1U)
    << "missing output port 'n_poses_dispatched'";

  // v0.2 NavigateThroughPoses feedback ports.
  EXPECT_EQ(ports.count("current_waypoint_index"), 1U)
    << "missing feedback port 'current_waypoint_index'";
  EXPECT_EQ(ports.count("number_of_poses_remaining"), 1U)
    << "missing feedback port 'number_of_poses_remaining'";
  EXPECT_EQ(ports.count("distance_remaining"), 1U)
    << "missing feedback port 'distance_remaining'";
  EXPECT_EQ(ports.count("number_of_recoveries"), 1U)
    << "missing feedback port 'number_of_recoveries'";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
