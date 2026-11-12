# New PC Quickstart Checklist (ROS 2 Jazzy)

This page is a command-first checklist for bringing up the autonomous navigation stack on a new PC.

## 0) Preconditions

- Ubuntu with ROS 2 Jazzy installed
- Git available
- Robot hardware connected (Livox MID-360 + OM motor driver)
- Serial device path confirmed (example: `/dev/ttyUSB0`)
- Network configured for Livox device communication

## 1) Workspace setup

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b develop https://github.com/megane312/amr_navigator_autonomy.git amr_navigator
```

## 2) Bring required dependent repositories into `src`

Add the repositories used by your robot stack (examples in this project context):

- livox_ros_driver2
- livox_mid360_mapping
- FAST_LIO
- om_modbus_master
- om_msgs

(Use your team's validated repository URLs/branches.)

## 3) Install dependencies and build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## 4) Record waypoints (manual drive)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch amr_navigator record_waypoints.launch.py
```

Expected during operation:

- `waypoint_recorder` starts
- `Saved waypoint #...` logs appear after movement

## 5) Replay waypoints (visual check)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch amr_navigator replay_waypoints.launch.py
```

Expected:

- `/waypoints` path appears in RViz

## 6) Autonomous follow with safety stop/resume

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch amr_navigator follow_waypoints.launch.py
```

Expected:

- Robot tracks waypoint sequence
- Obstacle monitor asserts stop and follower pauses
- Follower resumes same route after obstacle clears

## 7) Fast validation commands

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep -E '/odom|/pose|/waypoints|/waypoint_safety/stop'
ros2 topic echo /waypoint_safety/stop --once
```

## 8) Common issues

1. No waypoints saved

- Confirm `om_odom_tf` is running
- Confirm `/odom` and `/pose` are being published
- Confirm `recorded_waypoints.txt` path is writable

2. Odom rejected by ID mismatch

- Ensure launch parameters `topic_id` and `global_id` match hardware setup
- Ensure `om_odom_tf` expected slave ID aligns with driver response IDs

3. RViz plugin class load errors

- Use the RViz config files shipped in this repository
- Rebuild and re-source after config changes

## 9) Shutdown

Use Ctrl-C. Some nodes may print shutdown tracebacks (`rcl_shutdown already called`) during teardown; verify that core waypoint/odom behavior was correct during runtime.
