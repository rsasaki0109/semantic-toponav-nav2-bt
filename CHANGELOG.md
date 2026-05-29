# Changelog

All notable changes to `semantic_toponav_nav2_bt` are recorded in this
file. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and the package uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] — 2026-05-29

### Added

- `FollowSemanticWaypoints` now forwards live `NavigateThroughPoses`
  feedback to the blackboard so upper behavior trees can branch on
  navigation progress while the action is RUNNING. Four new output
  ports, refreshed on every feedback tick via the `on_wait_for_result()`
  hook:
  - `current_waypoint_index` (int) — zero-based index of the dispatched
    pose Nav2 is progressing toward, derived as
    `n_poses_dispatched - number_of_poses_remaining` and clamped into
    `[0, n_poses_dispatched - 1]`.
  - `number_of_poses_remaining` (int) — Nav2 feedback, forwarded verbatim.
  - `distance_remaining` (double, meters) — Nav2 feedback, forwarded
    verbatim.
  - `number_of_recoveries` (int) — Nav2 feedback, forwarded verbatim.
  All four are seeded with sentinel values (`-1` / `NaN`) on activation
  so a BT that peeks before the first feedback sees a defined
  "no progress yet" state rather than stale contents from a prior tick.
  To observe them mid-flight, place the node under a `ReactiveSequence`
  / `ReactiveFallback`; the sample tree in
  `behavior_trees/follow_semantic_waypoints.xml` shows the port wiring.
- gtest `ExposesDocumentedPorts` case asserting the static
  `providedPorts()` contract surface includes all input/output ports,
  guarding the new feedback ports against accidental rename / removal.

### Changed

- `ament_lint_auto` + `ament_lint_common` re-enabled in the build
  (`CMakeLists.txt`) and as `test_depend` entries in `package.xml`.
  cpplint / uncrustify / xmllint / lint_cmake now run under
  `colcon test` alongside the gtest smoke check. C++ sources rewritten
  to satisfy them: full Apache-2.0 boilerplate (license + warranty
  paragraph) on each file, includes rewritten in double-quote form
  with canonical ordering (C / C++ stdlib → third-party headers →
  local headers), `<gtest/gtest.h>` placed in the C-system block per
  cpplint, `<optional>` declared at the top of the header that uses
  it, the BT factory glue (`behaviortree_cpp_v3/bt_factory.h`) lifted
  out of mid-file into the upper include block, and the BT node
  builder lambdas in `BT_REGISTER_NODES` + the unit test refactored
  into anonymous-namespace functions so the uncrustify formatting
  rules apply cleanly.
- `ament_copyright` intentionally **excluded** via
  `AMENT_LINT_AUTO_EXCLUDE` — it expects a `CONTRIBUTING.md` at the
  package root and treats the standard Apache-2.0 LICENSE text as
  "unknown" because it has no project-line copyright header. cpplint's
  `legal/copyright` check still verifies the per-source headers.
  Documented in `CMakeLists.txt`. Closes #1.

## [0.1.0] — 2026-05-17

Initial scaffold landing.

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

### Deferred (resolved post-0.1.0)

- `ament_lint_auto` / `ament_lint_common` (uncrustify / cpplint /
  copyright / xmllint / pep257 / lint_cmake) intentionally not wired
  at the v0.1.0 scaffold stage — bringing the code in line with the
  ament canonical style was a focused follow-up; resolved in the next
  Unreleased entry above.

### Depends on

- `semantic_toponav_msgs` v0.1.0+ from the
  [`semantic-toponav`](https://github.com/rsasaki0109/semantic-toponav)
  core repo (vendored as a ROS 2 package under `ros2/`).
- Schema contract: `SemanticWaypointArray` v1.0
  ([upstream JSON Schema](https://github.com/rsasaki0109/semantic-toponav/blob/main/schemas/semantic_waypoint_array.schema.json)).
  Compatible with upstream tags `>= v1.0.0`; a v2 schema bump in the
  upstream wire format would require a matching major bump here.
