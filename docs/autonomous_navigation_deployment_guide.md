# Autonomous Navigation Deployment Guide

## Purpose

This document summarizes the development history and provides a reproducible procedure so another engineer can set up the same autonomous navigation functionality on a different PC.

The implementation target is a ROS 2 Jazzy based robot stack with:

- Livox MID-360 LiDAR
- FAST_LIO localization/mapping
- OM Modbus motor driver (ID Share mode)
- Waypoint record/replay/follow workflow
- Obstacle-based stop/resume safety layer

## Development Summary

The autonomous navigation workflow was built in phases:

1. Waypoint recording
- Added `record_waypoints.launch.py`
- Added/updated `waypoint_recorder` to save pose samples into CSV waypoints
- Added marker publication so recorded points are visible in RViz

2. Waypoint replay visualization
- Added `replay_waypoints.launch.py`
- Added `waypoint_mission.py` publish-only mode to publish `nav_msgs/Path` from CSV
- Cleaned RViz config for ROS 2 plugin compatibility

3. Minimal autonomous following
- Added `waypoint_follower.py`
- Follower reads CSV, subscribes to odometry, outputs OM query commands
- Added status and target marker outputs for observability

4. Safety stop/resume
- Added `obstacle_stop_monitor.py`
- Monitor inspects `/cloud_registered_body`, publishes stop flag and diagnostics
- Follower pauses without advancing waypoint index and resumes same target when clear

5. Launch and dependency cleanup
- Added `follow_waypoints.launch.py` with parameterized safety/follower settings
- Added missing package dependency (`sensor_msgs_py`)
- Added runtime diagnostics to identify odom/pose/recording flow issues

6. Odom alignment fix for recording
- Updated `om_odom_tf.py` and launch parameter wiring to support expected slave ID matching
- Added forwarding diagnostics in `odom_to_pose` and save diagnostics in `waypoint_recorder`

## Repository Layout (Key Files)

- Launch
  - `launch/record_waypoints.launch.py`
  - `launch/replay_waypoints.launch.py`
  - `launch/follow_waypoints.launch.py`
  - `launch/map_livox_fastlio_gmap.launch.py`

- Scripts
  - `scripts/waypoint_mission.py`
  - `scripts/waypoint_follower.py`
  - `scripts/obstacle_stop_monitor.py`
  - `scripts/joy_idshare_drive.py`
  - `scripts/om_odom_tf.py`

- C++ Nodes
  - `src/waypoint_recorder.cpp`
  - `src/odom_to_pose.cpp`

- RViz
  - `rviz/path.rviz`
  - `rviz/livox.rviz`

## Environment Requirements

- Ubuntu (validated on Linux)
- ROS 2 Jazzy
- Colcon build tools
- LiDAR and motor dependencies used by existing workspace

## Setup on Another PC

### 1) Clone source

Clone your workspace and this repository into `src/` as usual.

### 2) Install ROS and dependencies

Install ROS 2 Jazzy and all package dependencies required by your workspace.

### 3) Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select amr_navigator --symlink-install
source ~/ros2_ws/install/setup.bash
```

### 4) Verify package installation

```bash
ros2 pkg executables amr_navigator
```

Ensure executables include waypoint and safety scripts/nodes.

## Operation Workflow

### A. Record waypoints (manual drive)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch amr_navigator record_waypoints.launch.py
```

Expected recorder logs:

- `Ready to record the waypoints...`
- `Saved waypoint #...` after movement

Default output file:

- `install/amr_navigator/share/amr_navigator/path/recorded_waypoints.txt`

### B. Replay recorded waypoints (visualize)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch amr_navigator replay_waypoints.launch.py
```

Expected:

- `/waypoints` Path published
- Path visible in RViz

### C. Follow waypoints autonomously

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch amr_navigator follow_waypoints.launch.py
```

Expected:

- Follower drives toward recorded waypoints
- Target marker and status topic updates
- Safety stop/resume works from obstacle monitor

## Validation Checklist

1. File creation
- `recorded_waypoints.txt` exists and is not empty

2. Topic flow
- `/odom` is published
- `/pose` is forwarded by `odom_to_pose`
- `/waypoints` is published in replay/follow mode

3. Safety
- `/waypoint_safety/stop` toggles when obstacle count crosses thresholds
- Follower pauses without skipping waypoint index

## Known Runtime Notes

- `KeyboardInterrupt` and `rcl_shutdown already called` traces on Ctrl-C may appear in some Python nodes during shutdown. These are shutdown-sequence artifacts and not necessarily runtime failures.
- If no waypoints are written, check:
  - `om_odom_tf.py` started successfully
  - expected slave ID matches your driver behavior
  - `/odom` and `/pose` are active
- If replay warns that `recorded_waypoints.txt` is missing, fallback to `waypoints.txt` can be used for visualization validation.

## Recommended Bringup Commands

See `docs/manual_drive_setup.md` for detailed command variants and parameter tuning.

## Handover Notes

For deployment handoff, share:

- This document
- `docs/manual_drive_setup.md`
- Sample waypoint files under `path/`
- The exact launch command set used on target hardware
