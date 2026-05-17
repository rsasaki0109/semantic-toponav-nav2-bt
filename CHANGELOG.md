# Changelog

All notable changes to `semantic_toponav_nav2_bt` are recorded in this
file. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and the package uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Initial scaffold landing. No tagged release yet.

### Added

- `FollowSemanticWaypoints` BT action node — reads a
  `semantic_toponav_msgs/SemanticWaypointArray` from a blackboard
  input, converts each entry with `has_pose=true` to a
  `geometry_msgs/PoseStamped`, and dispatches the resulting list to
  Nav2's `navigate_through_poses` action. Inherits from
  `nav2_behavior_tree::BtActionNode` so it picks up cancel-on-halt,
  feedback ports, and server-timeout handling for free.
- Sample behavior tree under `behavior_trees/follow_semantic_waypoints.xml`.
- gtest smoke test verifying the BT factory registers the node type
  and exposes the documented blackboard ports.
- CI: GitHub Actions matrix on ROS 2 Humble running `colcon build`
  + `colcon test` against an apt-installed Nav2.
- Apache-2.0 LICENSE, `.gitignore` for colcon / cmake / editor cruft,
  `CHANGELOG.md` placeholder.

### Deferred

- `ament_lint_auto` / `ament_lint_common` (uncrustify / cpplint /
  copyright / xmllint / pep257 / lint_cmake) intentionally not wired
  at the v0.1.0 scaffold stage — bringing the code in line with the
  ament canonical style needs a focused follow-up PR. Tracked as the
  first issue against this repo.

### Depends on

- `semantic_toponav_msgs` v0.1.0+ from the
  [`semantic-toponav`](https://github.com/rsasaki0109/semantic-toponav)
  core repo (vendored as a ROS 2 package under `ros2/`).
- Schema contract: `SemanticWaypointArray` v1.0
  ([upstream JSON Schema](https://github.com/rsasaki0109/semantic-toponav/blob/main/schemas/semantic_waypoint_array.schema.json)).
  Compatible with upstream tags `>= v1.0.0`; a v2 schema bump in the
  upstream wire format would require a matching major bump here.
