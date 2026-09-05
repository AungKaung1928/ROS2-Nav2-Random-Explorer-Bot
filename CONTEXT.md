# CONTEXT.md — ROS2-Nav2-Random-Explorer-Bot (canonical; ros2-explorer-bot deleted 2026-09-05)

## Current
WORKING END-TO-END. Robot autonomously explores: random goals generated, Nav2
drives to them, SLAM maps as it goes. Verified: Goals #1-5 reached, /cmd_vel @20Hz,
RTF~1, single gz instance. Nothing pending — project functional.
OPERATIONAL FOOTGUN: always `pkill -9 -f "gz sim"` before relaunching. A stray
headless `gz sim -s` from a prior session shares the gz-transport bus and silently
corrupts /clock /odom /tf. Launch has no auto-cleanup of leftover servers.

## Solved
- 2026-09-05 CONSOLIDATION: this repo (3 stars, descriptive name) kept as canonical; content of the
  duplicate ros2-explorer-bot (measured results, docs/ map, LICENSE, .gitignore, blocking
  rclcpp::spin main) synced in byte-for-byte; ros2-explorer-bot deleted from GitHub.
- PORTFOLIO PASS: README.md written (8-section standard) + docs/explored_map.png|pgm|yaml
  saved live. Measured run: 66 goals, 38 reached / 26 aborted = 59% success, mean 12.2s
  to goal, 2.11m mean hop, full arena mapped @0.05m/cell. Honest weakness logged: random
  sampling -> 41% aborts in cylinder-dense arena -> roadmap = wire up the already-present
  generateFrontierBiasedGoal(). Live demo GIF deferred (gz/RViz buried behind terminals,
  no wmctrl/xdotool); record recipe is commented in README section 6.
- STATIC ROBOT ROOT CAUSE: a zombie headless `gz sim -s explore_world.sdf` (and a
  stray gz from moveit_project_ws) left running from the prior "verified headless"
  session. Two sim servers on one gz bus -> /clock raced to ~2260s (RTF~120x),
  odom drifted to (250,-184), SLAM map + costmaps followed -> all goals invalid.
  Fix: kill all stray gz/ros procs, relaunch clean. Robot then drives normally.
- Explorer pose bug: subscribed /amcl_pose (never published under SLAM) -> perpetual
  "Waiting for robot pose", goal distances measured from (0,0). Fixed: now does a
  tf2 lookup map->base_link in explorationLoop (tf2_ros added to CMake/package.xml).
- Switched world: explore_world.sdf now <include> model://turtlebot3_world;
  GZ_SIM_RESOURCE_PATH set to pkg models/ in turtlebot3_gz.launch.py. Model copied
  to models/turtlebot3_world (meshes hexagon.dae + wall.dae load fine on Harmonic).
- Spawn moved to (-2.0, -0.5) — origin has a cylinder in turtlebot3_world.
- Exploration bounds tightened to +/-2.0 (was +/-4.5), min_goal_distance 1.5->1.0
  to fit the ~4.6m turtlebot3_world arena.
- Harmonic TB3: description/turtlebot3_burger.urdf.xacro (gz-sim diff-drive +
  gpu_lidar + imu), config/gz_bridge.yaml bridges /scan /odom /tf /joint_states
  /imu /clock /cmd_vel. Verified headless earlier.
- Package symlinked into ~/ros2_ws/src; build with colcon, run explore_world.launch.py.
- Migrated off Classic (EOL, no binaries on this machine) -> Harmonic gz-sim8.
