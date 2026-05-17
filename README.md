# semantic-toponav-nav2-bt

Nav2 behavior-tree plugins that bridge the
[`semantic-toponav`](https://github.com/rsasaki0109/semantic-toponav)
planning layer to Nav2's motion stack.

The upstream project produces v1.0-locked
[`SemanticWaypointArray`](https://github.com/rsasaki0109/semantic-toponav/blob/main/schemas/semantic_waypoint_array.schema.json)
plans in Python. This package consumes them in C++ from inside a Nav2
behavior tree — no Python at runtime — and dispatches them to the
existing Nav2 action stack (`NavigateThroughPoses`).

> **Status: 0.1.0 scaffold.** The `FollowSemanticWaypoints` BT node
> compiles and registers but has not yet been exercised against a
> running Nav2 stack. Treat this as the contract surface, not a
> production drop-in. The integration test against an apt-installed
> Nav2 lands in a follow-up PR.

## What's in the package

| File | Purpose |
|---|---|
| `include/semantic_toponav_nav2_bt/follow_semantic_waypoints_action.hpp` | BT action-node class header |
| `src/follow_semantic_waypoints_action.cpp` | Implementation + `BT_REGISTER_NODES` glue |
| `plugins.xml` | Nav2 BT plugin manifest the BT factory loads |
| `behavior_trees/follow_semantic_waypoints.xml` | Sample BT tree showing the node in use |
| `test/test_plugin_loading.cpp` | gtest smoke check that the factory registers the node + exposes ports |
| `.github/workflows/colcon-ci.yml` | ROS 2 Humble matrix CI (build + test) |

## How the bridge works

```
┌──────────────────────────┐    SemanticWaypointArray
│  semantic-toponav (PY)   │   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ▶  ┌───────────────────────────┐
│  plan_with_scheduler /   │   ROS 2 topic / blackboard │  FollowSemanticWaypoints  │
│  waypoint_publisher_node │   (semantic_toponav_msgs)  │  (this package, C++)      │
└──────────────────────────┘                            └───────────────────────────┘
                                                                     │
                                                                     │ NavigateThroughPoses
                                                                     ▼
                                                       ┌───────────────────────────┐
                                                       │  Nav2 motion stack         │
                                                       │  (bt_navigator, planner,   │
                                                       │   controller, recovery)    │
                                                       └───────────────────────────┘
```

The BT node:

1. Reads `SemanticWaypointArray` from the blackboard input `{waypoints}`.
2. Filters out waypoints with `has_pose=false` (graph nodes without a
   2D pose — entrances, virtual junctions).
3. Converts each remaining `Pose2D` to a `geometry_msgs/PoseStamped`,
   stamping `frame_id` from the waypoint (falling back to the
   `SemanticWaypointArray` header when empty).
4. Builds a `nav2_msgs/action/NavigateThroughPoses.Goal` and dispatches
   it via the action-client lifecycle the parent `BtActionNode`
   provides (cancel-on-halt, feedback ports, server-side timeouts all
   come for free).
5. On `SUCCEEDED`, writes the dispatched-pose count to the blackboard
   output `{n_poses_dispatched}`.

The instruction / action / label fields from each `SemanticWaypoint`
are **not** consumed in this 0.1.0 scaffold — they're available on the
blackboard for downstream BT nodes (text-to-speech, UI overlays) to
pick up if the integrator wires them through.

## Build

This is a stock ROS 2 ament package. Drop it into a colcon workspace
alongside the
[`semantic_toponav_msgs`](https://github.com/rsasaki0109/semantic-toponav/tree/main/ros2/semantic_toponav_msgs)
package from the upstream repo:

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/rsasaki0109/semantic-toponav-nav2-bt.git
git clone https://github.com/rsasaki0109/semantic-toponav.git
# semantic_toponav_msgs lives under semantic-toponav/ros2/
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to semantic_toponav_nav2_bt
colcon test --packages-select semantic_toponav_nav2_bt
```

Tested ROS 2 distro: **Humble** (Nav2 `1.1.x` / BT.CPP v3).
Iron / Jazzy support is on the roadmap once the BT.CPP v4 migration
in Nav2 lands across the matrix.

## Schema compatibility

This package's wire-format contract is pinned to upstream
`SemanticWaypointArray` **v1.0** (locked in
[`docs/schema_v1.md`](https://github.com/rsasaki0109/semantic-toponav/blob/main/docs/schema_v1.md)
of the core repo). The freeze policy there means:

- Adding a new optional field in upstream → safe, this package
  continues to compile.
- Adding a *required* field, renaming, changing a type, or shifting
  an enum value in upstream → v2 wire bump → matching major bump
  here.

The `tests/test_schema_v1_lock.py` suite in the upstream repo blocks
those changes at upstream CI before they ever reach a release.

## Why this lives in its own repo

Documented under upstream decision
[D-16](https://github.com/rsasaki0109/semantic-toponav/blob/main/docs/decisions.md#d-16-heavy-adapters-live-out-of-repo):
heavy adapters (C++ ROS plugins, TypeScript Foxglove panels, torch
Mast3R weights) live out-of-core to keep the readable-Python-core
narrative intact. The core depends on **no** ROS / C++ / torch; this
package depends on the core's locked schemas and Protocols, not on
its internals.

## License

Apache-2.0, matching the upstream
[`semantic-toponav`](https://github.com/rsasaki0109/semantic-toponav)
project.
