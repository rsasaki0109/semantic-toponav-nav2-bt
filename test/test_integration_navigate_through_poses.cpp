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

// End-to-end integration test for the FollowSemanticWaypoints BT node.
//
// Rather than stand up a full Nav2 stack (bt_navigator + planner +
// controller + localization + a simulator), which is heavy and
// non-deterministic in CI, this test exercises the node against an
// in-process NavigateThroughPoses action server that stands in for the
// real one. That covers the entire contract the node actually depends
// on: the BtActionNode lifecycle (goal translation, dispatch, feedback
// forwarding, result handling). Driving the real planner / controller
// requires a simulator and is out of scope for unit CI.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "semantic_toponav_msgs/msg/semantic_waypoint_array.hpp"
#include "semantic_toponav_nav2_bt/follow_semantic_waypoints_action.hpp"

namespace stb = semantic_toponav_nav2_bt;

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

namespace
{

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateThroughPoses>;

// A minimal stand-in for the Nav2 NavigateThroughPoses action server.
// It records how many poses the goal carried, streams a countdown of
// feedback messages so the BT node has something to forward, and then
// reports SUCCEEDED.
class FakeNavigateThroughPosesServer
{
public:
  explicit FakeNavigateThroughPosesServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    action_server_ = rclcpp_action::create_server<NavigateThroughPoses>(
      node_,
      "navigate_through_poses",
      std::bind(&FakeNavigateThroughPosesServer::handle_goal, this, _1, _2),
      std::bind(&FakeNavigateThroughPosesServer::handle_cancel, this, _1),
      std::bind(&FakeNavigateThroughPosesServer::handle_accepted, this, _1));
  }

  int received_pose_count() const
  {
    return received_pose_count_.load();
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateThroughPoses::Goal> goal)
  {
    received_pose_count_.store(static_cast<int>(goal->poses.size()));
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread(
      [this, goal_handle]() {
        this->execute(goal_handle);
      }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const int total = static_cast<int>(goal->poses.size());
    auto feedback = std::make_shared<NavigateThroughPoses::Feedback>();
    // Count remaining poses down to zero so the BT node observes the
    // index advancing across several feedback ticks.
    for (int remaining = total; remaining >= 0; --remaining) {
      feedback->number_of_poses_remaining = static_cast<int16_t>(remaining);
      feedback->distance_remaining = static_cast<float>(remaining) * 2.0f;
      feedback->number_of_recoveries = 0;
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(40ms);
    }
    auto result = std::make_shared<NavigateThroughPoses::Result>();
    goal_handle->succeed(result);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<NavigateThroughPoses>::SharedPtr action_server_;
  std::atomic<int> received_pose_count_{-1};
};

// Build the SemanticWaypointArray the node will translate: three
// pose-bearing waypoints interleaved with one pose-less graph node
// (an entrance / virtual junction) that must be filtered out.
stb::FollowSemanticWaypointsAction::WaypointArray make_plan()
{
  stb::FollowSemanticWaypointsAction::WaypointArray plan;
  plan.header.frame_id = "map";

  for (int i = 0; i < 3; ++i) {
    semantic_toponav_msgs::msg::SemanticWaypoint wp;
    wp.node_id = "room_" + std::to_string(i);
    wp.has_pose = true;
    wp.frame_id = "map";
    wp.pose.x = static_cast<double>(i);
    wp.pose.y = static_cast<double>(i) * 0.5;
    wp.pose.theta = 0.0;
    plan.waypoints.push_back(wp);

    if (i == 1) {
      // A pose-less junction between rooms 1 and 2; must be skipped.
      semantic_toponav_msgs::msg::SemanticWaypoint junction;
      junction.node_id = "junction";
      junction.has_pose = false;
      plan.waypoints.push_back(junction);
    }
  }
  return plan;
}

// FollowSemanticWaypointsAction's constructor takes an explicit action
// name, so it cannot be registered via the default registerNodeType<>;
// supply the same builder the BT_REGISTER_NODES glue uses in production.
std::unique_ptr<BT::TreeNode> build_node(
  const std::string & name, const BT::NodeConfiguration & config)
{
  return std::make_unique<stb::FollowSemanticWaypointsAction>(
    name, "navigate_through_poses", config);
}

const char * const kTreeXml =
  R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <FollowSemanticWaypoints
      waypoints="{waypoints}"
      server_name="navigate_through_poses"
      n_poses_dispatched="{n_poses_dispatched}"
      current_waypoint_index="{current_waypoint_index}"
      number_of_poses_remaining="{number_of_poses_remaining}"
      distance_remaining="{distance_remaining}"
      number_of_recoveries="{number_of_recoveries}"/>
  </BehaviorTree>
</root>)";

}  // namespace

// Drives the node against the fake server and asserts that:
//   - pose-less waypoints are filtered before dispatch,
//   - the goal reaches the action server with the right pose count,
//   - live feedback is forwarded to the blackboard mid-flight,
//   - the action resolves to SUCCESS with n_poses_dispatched set.
TEST(FollowSemanticWaypointsIntegration, DispatchesAndForwardsFeedback)
{
  auto server_node = std::make_shared<rclcpp::Node>("fake_ntp_server");
  FakeNavigateThroughPosesServer server(server_node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  std::atomic<bool> spinning{true};
  std::thread spin_thread(
    [&executor, &spinning]() {
      while (spinning.load() && rclcpp::ok()) {
        executor.spin_some();
        std::this_thread::sleep_for(2ms);
      }
    });

  auto bt_node = std::make_shared<rclcpp::Node>("integration_test_bt");
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", bt_node);
  blackboard->set<std::chrono::milliseconds>("server_timeout", 20000ms);
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
  blackboard->set<std::chrono::milliseconds>(
    "wait_for_service_timeout", 20000ms);
  blackboard->set("waypoints", make_plan());

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<stb::FollowSemanticWaypointsAction>(
    "FollowSemanticWaypoints", build_node);
  auto tree = factory.createTreeFromText(kTreeXml, blackboard);

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  bool saw_feedback = false;
  int observed_index = -1;
  double observed_distance = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 4000; ++i) {
    status = tree.tickRoot();
    int remaining = -1;
    if (blackboard->get("number_of_poses_remaining", remaining) &&
      remaining != -1)
    {
      saw_feedback = true;
      blackboard->get("current_waypoint_index", observed_index);
      blackboard->get("distance_remaining", observed_distance);
    }
    if (status == BT::NodeStatus::SUCCESS ||
      status == BT::NodeStatus::FAILURE)
    {
      break;
    }
    std::this_thread::sleep_for(5ms);
  }

  spinning.store(false);
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS)
    << "the action did not resolve to SUCCESS";
  EXPECT_EQ(server.received_pose_count(), 3)
    << "the pose-less junction should have been filtered out";

  int dispatched = -1;
  ASSERT_TRUE(blackboard->get("n_poses_dispatched", dispatched));
  EXPECT_EQ(dispatched, 3);

  EXPECT_TRUE(saw_feedback)
    << "no NavigateThroughPoses feedback reached the blackboard";
  EXPECT_GE(observed_index, 0);
  EXPECT_LT(observed_index, 3);
  EXPECT_FALSE(std::isnan(observed_distance))
    << "distance_remaining was never forwarded from feedback";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
