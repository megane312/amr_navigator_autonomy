# Manual Drive / ID-Share Setup

This document describes how to start the OM Modbus driver and the manual (joy) ID-Share drive, and how to collect logs/debug a crash in `om_modbus_master`.

## Cheat Sheet

### Navigation only

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch amr_navigator nav2_bringup.launch.py bringup_preset:=nav_only
```

### Manual drive only

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export AMENT_PREFIX_PATH=/home/ubuntu/ros2_ws/install/amr_navigator:$AMENT_PREFIX_PATH
ros2 launch amr_navigator nav2_bringup.launch.py bringup_preset:=manual_drive
```

### Navigation + manual drive

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export AMENT_PREFIX_PATH=/home/ubuntu/ros2_ws/install/amr_navigator:$AMENT_PREFIX_PATH
ros2 launch amr_navigator nav2_bringup.launch.py bringup_preset:=nav_manual
```

### Livox MID360 + FAST_LIO + manual drive

This launch starts the Livox MID360 driver, FAST_LIO mapping, map saving, and joystick-based motor control together.

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch amr_navigator map_livox_fastlio_gmap.launch.py
```

You can still tune the controller mapping and speed at launch time:

```bash
ros2 launch amr_navigator map_livox_fastlio_gmap.launch.py \
  forward_axis:=1 turn_axis:=0 deadman_button:=4 speed_scale:=2.0
```

### Record waypoints while driving

This launch records the `/pose` stream into a CSV file while manual drive is active.

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch amr_navigator record_waypoints.launch.py
```

### Replay recorded waypoints

This launch publishes the recorded CSV as `/waypoints` and shows it in RViz.

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch amr_navigator replay_waypoints.launch.py
```

### Autonomous waypoint following with obstacle stop/resume

This launch brings up FAST_LIO, OM Modbus, the obstacle monitor, and the waypoint follower.

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch amr_navigator follow_waypoints.launch.py
```

The default safety topics are:

- `/waypoint_safety/stop`
- `/waypoint_safety/obstacle_count`
- `/waypoint_safety_marker`
- `/waypoint_target_marker`

The default obstacle monitor cloud input is `/cloud_registered_body` from FAST_LIO, and the follower pauses on stop messages without advancing the current waypoint index.

RViz uses the `livox.rviz` config, which is set up to show the FAST_LIO point clouds on `/cloud_registered_body` and `/cloud_registered`, along with `/map` and TF. If no point cloud appears, check whether FAST_LIO is publishing those topics and whether the LiDAR/IMU are actually streaming data.

### Speed tuning

Manual drive speed is controlled by these launch parameters:

- `max_forward_rpm` and `max_turn_rpm` set the base joystick-to-RPM range
- `speed_scale` multiplies the final command RPM, and defaults to `2.0`

Examples:

```bash
# Make manual drive slower
ros2 launch amr_navigator joy_manual_drive.launch.py speed_scale:=1.0

# Increase only forward range
ros2 launch amr_navigator joy_manual_drive.launch.py max_forward_rpm:=500.0
```

When `joy_idshare_drive` is running, it now logs joystick axes, configured limits, and the final RPM command whenever the command changes. The log shows `left=... rpm right=... rpm` so you can see whether the command is limited by the joystick input, the configured max RPM, or the speed scale.

### Controller mapping

The current joystick-to-motor mapping is summarized below.

| Item | Launch parameter | Default | Meaning |
| --- | --- | --- | --- |
| Forward / backward axis | `forward_axis` | `1` | Axis used for straight motion |
| Left / right turn axis | `turn_axis` | `0` | Axis used for rotation |
| Enable button | `deadman_button` | `4` | Must be pressed to allow motion |
| Forward speed limit | `max_forward_rpm` | `360.0` | Base forward/backward RPM range |
| Turn speed limit | `max_turn_rpm` | `180.0` | Base turn RPM range |
| Final speed multiplier | `speed_scale` | `2.0` | Multiplies the final motor RPM command |

The motor command is computed as:

```text
right_rpm = (forward + turn) * speed_scale
left_rpm  = (forward - turn) * speed_scale
```

where:

- `forward = joystick_forward_axis_value * max_forward_rpm`
- `turn = joystick_turn_axis_value * max_turn_rpm`

So if you change only one value, `speed_scale` is the simplest knob for overall speed, and `forward_axis` / `turn_axis` are the knobs for changing which stick direction controls which motion.

### `om_modbus_master` only

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch om_modbus_master om_modbus_master_launch.py \
  com:=/dev/ttyUSB0 topicID:=1 baudrate:=230400 updateRate:=1000 \
  secondGen:="1,2," globalID:=10 axisNum:=2
```

## Quick start (recommended)

1. Source ROS2 and installed workspace packages:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
```

2. Start `om_modbus_master` with correct `secondGen` formatting (trailing comma is required):

```bash
ros2 launch om_modbus_master om_modbus_master_launch.py \
  com:=/dev/ttyUSB0 topicID:=1 baudrate:=230400 updateRate:=1000 \
  secondGen:="1,2," globalID:=10 axisNum:=2
```

Notes:
- If you start `om_modbus_master` from the `nav2_bringup.launch.py`, the bundle includes a copy; for production we recommend starting the driver separately as above.
- The `secondGen` parameter must be a comma-separated string with a trailing comma, e.g. `"1,2,"`. A malformed value (missing comma) causes the node to exit with `Error: Comma does not exist`.

3. In another terminal start the nav bringup and enable the manual drive:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

# If you started om_modbus_master manually, disable the bundled start
ros2 launch amr_navigator nav2_bringup.launch.py \
  start_om_modbus_master:=false start_manual_drive:=true
```

You can also use the preset selector instead of setting both flags manually:

```bash
# Navigation only
ros2 launch amr_navigator nav2_bringup.launch.py bringup_preset:=nav_only

# Manual drive only
ros2 launch amr_navigator nav2_bringup.launch.py bringup_preset:=manual_drive

# Navigation + manual drive (default)
ros2 launch amr_navigator nav2_bringup.launch.py bringup_preset:=nav_manual

# Manual drive only means no Nav2 stack and no static map->odom bridge
ros2 launch amr_navigator nav2_bringup.launch.py bringup_preset:=manual_drive
```

## Real Robot Start Patterns

Use these as the standard execution patterns on the real robot.

### 1. Navigation only

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

ros2 launch amr_navigator nav2_bringup.launch.py \
  bringup_preset:=nav_only
```

### 2. Manual drive only

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

export AMENT_PREFIX_PATH=/home/ubuntu/ros2_ws/install/amr_navigator:$AMENT_PREFIX_PATH
ros2 launch amr_navigator nav2_bringup.launch.py \
  bringup_preset:=manual_drive
```

### 3. Navigation + manual drive

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

export AMENT_PREFIX_PATH=/home/ubuntu/ros2_ws/install/amr_navigator:$AMENT_PREFIX_PATH
ros2 launch amr_navigator nav2_bringup.launch.py \
  bringup_preset:=nav_manual
```

If you want to start `om_modbus_master` separately, keep the driver in a dedicated terminal and use:

```bash
ros2 launch om_modbus_master om_modbus_master_launch.py \
  com:=/dev/ttyUSB0 topicID:=1 baudrate:=230400 updateRate:=1000 \
  secondGen:="1,2," globalID:=10 axisNum:=2
```

Then start the combined Nav2/manual bringup in another terminal with `bringup_preset:=nav_manual` or `bringup_preset:=manual_drive` depending on whether you need Nav2.

## Flags exposed by `nav2_bringup.launch.py`

- `start_om_modbus_master` (default: `true`) — start the bundled `om_modbus_master` instance.
- `start_manual_drive` (default: `false`) — include `joy_manual_drive.launch.py` (manual ID-Share drive).
- `start_nav_stack` (default: `true` for `nav_only` and `nav_manual`, `false` for `manual_drive`) — start Nav2 localization/navigation.
- `bringup_preset` (default: `nav_manual`) — one of `nav_only`, `manual_drive`, `nav_manual`.

## How to reproduce and capture a crash (om_modbus_master)

1. Enable core dumps in the shell before launching:

```bash
ulimit -c unlimited
```

2. Launch the node in a terminal and keep the terminal open so stdout/stderr are captured (or redirect to a log file):

```bash
ros2 launch om_modbus_master om_modbus_master_launch.py ... 2>&1 | tee om_modbus_master.log
```

3. When the process crashes, collect:

- the `om_modbus_master.log` file
- any `core.*` file in the working directory or `/var/lib/systemd/coredump` depending on system config

4. If a core dump is present, get a backtrace with gdb:

```bash
gdb --batch -ex "thread apply all bt full" -ex quit /path/to/om_modbus_master <core-file> > om_modbus_master_bt.txt
```

Alternatively attach gdb to a running process (useful when reproducing interactively):

```bash
pid=$(pgrep -f om_modbus_master)
sudo gdb -p $pid
# then in gdb: thread apply all bt
```

5. For memory corruption (double free) or heap issues consider AddressSanitizer build:

```bash
# Rebuild om_modbus_master with ASAN
colcon build --packages-select om_modbus_master \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address"

# Then run the same launch and collect ASAN output (it prints to stderr/stdout)
```

6. If reproducing under Valgrind is acceptable (slower):

```bash
valgrind --leak-check=full --show-leak-kinds=all --log-file=valgrind_om.log \
  ros2 launch om_modbus_master om_modbus_master_launch.py ...
```

## Collecting ROS-level traces

- Capture topics (queries/responses) while running a test to correlate crash time with modbus frames:

```bash
ros2 topic echo /om_query1 --once > om_query_sample.txt
ros2 topic echo /om_response1 --once > om_response_sample.txt
```

## Notes / Observations

- In previous testing the driver sometimes exits with `double free or corruption (out)` under heavy/debug logging. The most common user-level cause found earlier was malformed launch args for `secondGen`.
- Before performing deep memory-debugging runs, reproduce with normal logging and gather the log and the most recent `om_response` messages.

---
If you want, I can now (A) run a short reproduce attempt here and collect logs, or (B) proceed to prepare a sanitized build (ASAN) and then run to capture a sanitizer report — which do you prefer? Reply with `A` or `B` (or `both`).
