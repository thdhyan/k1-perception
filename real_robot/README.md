# real_robot — recording and sync for the Booster K1 on hardware

Everything needed to capture real-robot data while the robot is moving, and to
move the resulting bags between the robot and a workstation. Kept separate from
`isaac_tasks/` (simulation) because this directory talks to a physical machine
and has different safety and failure modes.

## Why this exists

Two separate jobs, often confused:

1. **Record** what the robot actually did — TF, IMU, camera, odometry, joints.
   This is the ground truth we currently have none of. Every sim2real claim so
   far rests on actuator models we *inferred*, not measured.
2. **Sync** the bags. The robot is a network client with finite storage; the
   workstation is where the data gets processed. Bags must move both ways.

## Topic contract

The K1 SDK speaks FastDDS (`rt/low_state`, `rt/odometer_state`). ROS 2 also runs
over DDS, but **there is no bridge between them yet** — see `sdk_bridge_node`
in `src/k1_control/k1_control/` (the Python version is a stub; the C++ one is
the intended path). Until that bridge exists, the topics below are what a
bridged rig must publish for these scripts to be useful.

| Topic | Type | Notes |
|---|---|---|
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Base → link tree. Built from joint states. **Without TF there is no map.** |
| `/imu/data` | `sensor_msgs/Imu` | Orientation as a **quaternion**. See the warning below. |
| `/odom` | `nav_msgs/Odometry` | Base linear/angular velocity. Gives *achieved* speed, not commanded. |
| `/<cam>/color/image_raw` | `sensor_msgs/Image` | RGB |
| `/<cam>/color/camera_info` | `sensor_msgs/CameraInfo` | **Required.** cuVSLAM rejects uncalibrated frames. |
| `/<cam>/depth/image_raw` | `sensor_msgs/Image` | For NVBloX reconstruction |
| `/joint_states` | `sensor_msgs/JointState` | 22 joints |

### IMU orientation warning

`booster_ros2_interface/msg/ImuState` is **not** a `sensor_msgs/Imu`:

```
float32[3] rpy     # roll, pitch, yaw   <-- EULER, no quaternion
float32[3] gyro
float32[3] acc
```

Any bridge that copies rpy straight into an `Imu.orientation` field produces a
silently wrong rotation and a garbage map. Convert properly. This is a real bug
in our own `locomotion_node.py` per `docs/sdk_ros2_audit.md`.

## Usage

```bash
# 1. discover / check the robot is publishing
source config/env.sh                      # or: source /opt/ros/humble/setup.bash
./check_topics.sh                         # what is alive, rates, TF tree

# 2. record a session (stops cleanly on Ctrl-C, then pulls to ./bags)
./record_session.sh walk_fast --duration 120

# 3. or drive it manually
./rosbag_record.sh --out bags/walk_fast --duration 120 --topics-file config/topics_lidar_walk.yaml
```

## Files

| File | Purpose |
|---|---|
| `check_topics.sh` | Health check: topic rates, TF tree, calibration presence |
| `rosbag_record.sh` | The recorder. Chunked, compressed, size-capped, clean shutdown |
| `record_session.sh` | Orchestrates: pre-flight → record → verify → sftp pull → report |
| `sftp_sync.sh` | Batch pull/push against the robot with resume + checksums |
| `verify_bag.py` | Post-hoc check: duration, topic coverage, message counts, NaN scan |
| `config/` | Topic lists, env template |
| `bags/` | Local bag storage (gitignored) |

## Action labels for behaviour cloning

The recorder captures observation and **action** separately, because BC needs
both: `dq/ddq/tau_est` + IMU + odometry is the observation, and the commanded
`(vx, vy, vyaw)` is the action label. That is the same interface a VLM emits,
so the same dataset serves both a BC policy and a navigation policy.

What is available, verified against the SDK:

| Label | Source | Status |
|---|---|---|
| Commanded velocity | `B1LocoClient.Move(vx, vy, vyaw)` | **Logged.** Our own commands |
| Robot mode / behaviour mask | `rt/robot_states` | **Logged** (`current_mode`, `current_body_control`, `current_actions` bitmask) |
| Fall / recovery | `rt/fall_down` | **Logged** — lets you drop or label falls |
| Soccer behaviour | `rt/robocup_behavior_status` | **Logged** |
| Joystick axes | `RemoteControllerState` exists in the SDK but has **no `rt/` topic constant** | **Not loggable yet** — see below |
| Factory walker's joint targets | firmware-internal | **Not available, ever** |

### The one that does not exist

The factory walker's **joint-space actions cannot be captured.** `rt/joint_ctrl`
(`LowCmd`) is an *input* to the robot's low-level controller; the firmware never
echoes its internal commands, and there is no `ChannelSubscriber<LowCmd>`
anywhere in the SDK. So joint-level imitation by cloning the factory policy's
actions is off the table.

Use the recorded joint trajectory as a **reference motion** instead — that is
motion tracking (BeyondMimic / ProtoMotions style), not BC, and it is the right
tool for the human-like-gait goal anyway.

### Worth doing if you can

If a person drives the K1 with the handheld remote while recording, that is a
**human demonstration** — the highest-value dataset available, and directly on
target for "human-like gait". The message type exists
(`RemoteControllerState`: `lx, ly, rx, ry`, hat switches, buttons) but no topic
constant is published, so find the topic at runtime first:

```bash
# on the robot's network, list what actually has a publisher
python3 scripts/rosbag_record.sh --out bags/probe --duration 15 \
  --topics-file /dev/null 2>/dev/null     # then: ros2 topic list -t | grep -i remote
```

Then add it to `config/topics_walk.yaml`. If the remote does not publish over
DDS, log your own `Move()` calls instead — which the recorder already does.

## Full K1 topic inventory (from the SDK)

Useful beyond the recorder — several are already ROS-compatible:

| Topic | Note |
|---|---|
| `rt/low_state` | joints + IMU + fall state |
| `rt/odometer_state` | `x, y, theta` |
| `rt/joint_ctrl` | LowCmd — **we publish here**; firmware does not echo |
| `rt/fall_down` | fall / recovery-available |
| `rt/robot_states` | mode, body control, action mask |
| `rt/robocup_behavior_status` | soccer behaviour (Track A relevant) |
| `rt/kick_ball` | kick reference (Track A P4 relevant) |
| `rt/imu/data` | **ROS-compatible** — may be a real `sensor_msgs/Imu` |
| `rt/odom` | **ROS-compatible** |
| `rt/joint_states` | **ROS-compatible** |
| `rt/tf` | **ROS-compatible** |

If `rt/imu/data` really is `sensor_msgs/Imu`, the FastDDS-to-ROS 2 bridge is far
smaller than assumed: a topic remap may be enough. Verify at runtime before
building the bridge.

## Storage reality

Bags are large — a 120 s RGB-D + IMU + TF session is easily 2–10 GB. That is why:

- `bags/` is gitignored. **Never commit bag data.**
- Recording is chunked (default 512 MB) so a crash loses at most one chunk.
- Sync is resumable and checksummed; a partial transfer is detected, not
  silently accepted.

## Safety

- The recorder is read-only. It never publishes commands.
- `record_session.sh` has a pre-flight that refuses to start if TF or IMU is
  missing, because a bag without them is not worth the disk.
- Recording on a robot that is falling over is still fine; do not add compute
  load to a machine that is mid-fall.

---
> **Network note (2026-10-27):** the robotics router (192.168.50.1) has no internet uplink. The laptop shares its eduroam (`wlo1`) internet to the robots by NAT: `enp2s0` = 192.168.50.250/24 is the robots' gateway, with `ip_forward=1` and nft table `ip k1nat` (masquerade out `wlo1`, source 192.168.50.0/24). It is lost on laptop reboot; robots use DNS 8.8.8.8/1.1.1.1 (router DNS is wrong). Re-create steps: `docs/network_robotics_5G_2026-10-26.md` ("Robot internet via laptop").
