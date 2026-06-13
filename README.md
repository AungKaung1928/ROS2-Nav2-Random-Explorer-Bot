# random_explorer_bot

Autonomous exploration of an unknown indoor world: a TurtleBot3 Burger drives itself to randomly sampled, collision-checked goals while SLAM builds a map of everything it sees — fully simulated in **Gazebo Harmonic**, no hardware required.

---

## 1. What it does

A single C++ node (`exploration_controller`) picks random reachable goals inside the
known map, sends them to Nav2 as `NavigateToPose` actions, and lets `slam_toolbox`
map the world as the robot drives. Goals that occupy or sit too close to an obstacle
are rejected before they are ever sent; goals Nav2 cannot reach within a timeout are
cancelled and replaced. The robot explores indefinitely with no human input.

## 2. Node graph

```
                Gazebo Harmonic (gz-sim8)
                ┌────────────────────────┐
                │  explore_world.sdf      │
                │  TB3 Burger: diff-drive │
                │  + gpu_lidar + imu      │
                └───────────┬─────────────┘
                            │ gz-transport
                   ┌────────┴──────────┐
                   │   ros_gz_bridge   │  /clock /scan /odom /tf /imu  (GZ→ROS)
                   │ (parameter_bridge)│  /cmd_vel                     (ROS→GZ)
                   └────────┬──────────┘
            /scan /tf  ┌────┴────────────────────────────┐  /tf (map→odom)
                  ┌────▼───────┐                    ┌─────▼──────────────────┐
                  │ slam_toolbox│  /map ─────────►  │ exploration_controller │
                  │   (sync)    │                   │  (this package, C++)   │
                  └────┬────────┘                   │  • RandomGoalGenerator │
            /map(costmap)│                          │  • MapValidator        │
                  ┌─────▼───────────────────────┐   │  • tf2 map→base_link   │
                  │            Nav2              │   └───────┬────────────────┘
                  │ planner → controller →       │  navigate_to_pose (action)
                  │ velocity_smoother → /cmd_vel │◄─────────┘
                  │ (lifecycle: autostart)       │
                  └──────────────────────────────┘
                         │ /exploration_goal_marker → RViz2
```

tf chain: `map`→`odom` (slam_toolbox) → `base_footprint` (gz diff-drive) →
`base_link`/`base_scan` (robot_state_publisher).
cmd_vel chain: `controller_server` → `/cmd_vel_nav` → `velocity_smoother` →
`/cmd_vel` → `ros_gz_bridge` → DiffDrive plugin.

## 3. Key decisions

- **Gazebo Harmonic (gz-sim8), not Classic.** Gazebo Classic is EOL and is not installed
  on Humble here (`turtlebot3_gazebo` is absent). The entire sim — world, diff-drive,
  gpu_lidar, imu, and the ros_gz bridge — is **self-contained in this package**
  (`worlds/`, `models/`, `description/`, `config/gz_bridge.yaml`), so it runs from a
  clean checkout with no external sim package.
- **Pose from the SLAM tf tree, not `/amcl_pose`.** The map is unknown at start, so
  localization comes from SLAM's `map`→`odom` tf — there is no AMCL. Pose is read via a
  `tf2` `map`→`base_link` lookup. An earlier version subscribed to `/amcl_pose`, which
  never publishes under SLAM, so every goal distance was measured from `(0,0)`.
- **Goal logic in C++, not Python.** Goal sampling + occupancy-grid clearance checks run
  in the planning hot path; C++ keeps `MapValidator`'s per-cell circular clearance scan
  cheap enough to run every cycle without stalling callbacks.
- **`slam_toolbox` + Nav2 `navigation_launch.py` (no AMCL/map_server).** SLAM provides
  both the map and the `map`→`odom` transform.
- **Pre-send validity filtering.** Goals are clearance-checked (0.35 m circular) against
  the live occupancy grid *before* dispatch, so Nav2 is never handed goals inside the
  obstacles — cutting wasted planning cycles.
- **Timed staged bringup.** Sim starts first; the nav stack waits 8 s and the explorer a
  further 10 s, so `/scan`, `/odom`, and tf are flowing before Nav2 initializes (avoids
  the lifecycle-not-active and tf-startup-race footguns).

## 4. Dependencies

**Build:** `ament_cmake`, C++17.
**ROS 2 Humble packages:** `rclcpp`, `rclcpp_action`, `nav2_msgs`, `geometry_msgs`,
`nav_msgs`, `sensor_msgs`, `action_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`,
`visualization_msgs`.
**Runtime stack:** `nav2_bringup`, `slam_toolbox`, `robot_state_publisher`, `xacro`,
`ros_gz_sim`, `ros_gz_bridge`, `turtlebot3_description`, `rviz2`.
**Simulator:** Gazebo Harmonic (gz-sim8).

## 5. Build & launch

```bash
cd ~/nav2_explorer_ws
colcon build --packages-select random_explorer_bot --symlink-install
source install/setup.bash

# One-shot demo: Gazebo Harmonic + bridge + SLAM + Nav2 + RViz + explorer
ros2 launch random_explorer_bot explore_world.launch.py
```

Bring up only the simulator + ros_gz bridge (sensors, no nav stack):

```bash
ros2 launch random_explorer_bot turtlebot3_gz.launch.py
# then: ros2 topic hz /scan   # ~5 Hz   ros2 topic hz /odom   # ~30 Hz
```

> **Operational footgun:** always `pkill -9 -f "gz sim"` before relaunching. A leftover
> headless `gz sim` server shares the global gz-transport bus and silently corrupts
> `/clock`, `/odom`, and `/tf`. After launch, sanity-check: exactly one `gz sim server`,
> `/clock` ≈ wall time, odom near the spawn at `(-2.0, -0.5)`.

Save the live SLAM map at any time:

```bash
ros2 run nav2_map_server map_saver_cli -f explored_map
```

## 6. Demo

Best capture is RViz2 (occupancy grid growing + colored goal-arrow markers) over a live
session. Record with the windows foregrounded, then commit `docs/demo.gif`:

```bash
ffmpeg -f x11grab -framerate 15 -video_size <W>x<H> -i :0.0+<X>,<Y> -t 12 /tmp/demo.mp4
ffmpeg -i /tmp/demo.mp4 -vf "fps=10,scale=600:-1:flags=lanczos,palettegen" /tmp/p.png
ffmpeg -i /tmp/demo.mp4 -i /tmp/p.png -lavfi "fps=10,scale=600:-1[x];[x][1:v]paletteuse" docs/demo.gif
```

## 7. Verified results

Migration smoke-test on this machine (ROS 2 Humble, Gazebo Sim 8.12.0), from
`turtlebot3_gz.launch.py`:

| Check | Result |
|---|---|
| `gz sim` loads `explore_world.sdf` | ✅ |
| `/scan` (gpu_lidar → bridge) | ✅ ~4.8 Hz |
| `/odom` (diff-drive → bridge) | ✅ ~28.7 Hz |
| `/clock` (sim time → bridge) | ✅ advancing |
| tf `odom`→`base_footprint` (diff-drive) | ✅ |
| tf `base_link`→`base_scan` (robot_state_publisher) | ✅ |
| `colcon build` | ✅ clean |

Exploration-run metrics (goals dispatched/reached, success rate, coverage) are
reproducible from a full `explore_world.launch.py` session plus `map_saver_cli`; they
are intentionally not pre-filled here — record them from your own run.

## 8. Planned improvements

- **Frontier-based exploration.** `RandomGoalGenerator::generateFrontierBiasedGoal()`
  already exists but is unused — the controller calls pure-random `generateGoal()`. Wire
  it in (drive toward known/unknown map boundaries) to raise success rate and cut aborts
  in obstacle-dense regions.
- **Coverage-aware termination.** Stop or report "done" when the explored free-cell area
  stops growing, instead of exploring forever.
- **Replace the wall-clock goal timeout** with a Nav2-progress check (cancel on *stalled*
  progress, not just elapsed time) so slow-but-valid goals aren't dropped.
- **Launch-time cleanup of stray gz servers** so the operational footgun in §5 can't bite
  on relaunch.
