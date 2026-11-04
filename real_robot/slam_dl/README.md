# cuVSLAM + NVBloX on dl — why a container

## Host reality (checked, not assumed)

| | dl | laptop |
|---|---|---|
| OS | Ubuntu 22.04.5 x86_64 | — |
| GPU | 4× RTX 6000 Ada, 48 GB each | RTX 4060 Laptop, 8 GB |
| Driver | 590.48.01 (CUDA 13 capable) | 580.173.02 |
| CUDA toolkit | **11.7 — too old** | 12.0 |
| Python | 3.10.12 | 3.12 |
| ROS 2 | Humble | Jazzy |
| Free disk | 366 GB | **16 GB** |

Three consequences:

1. **cuVSLAM needs CUDA 12.6.3 or 13.2.0**; dl's host toolkit is 11.7. The
   driver (590) is fine — CUDA is backward compatible via the runtime — so the
   fix is a container, not a driver upgrade.
2. **Python 3.10 on dl matches cuVSLAM's `cp310` wheel tag exactly.** That is a
   prebuilt wheel, no compile.
3. **The laptop is not viable for this.** 8 GB VRAM, 16 GB free disk, and
   `AGENTS.md` says the laptop is a client only. cuVSLAM's prebuilt archive plus
   an nvblox build does not fit in 16 GB alongside the existing 130 GB of images.

## Plan

Run the SLAM stack in a CUDA 12/13 container on dl, with ROS 2 Humble from the
host mounted in.

```
Booster K1 ──DDS──> [laptop: recorder + rviz]      (rosbag, sftp sync)
                 └─> [dl: cuVSLAM -> nvblox]      (map: TSDF/ESDF/occupancy)
```

The laptop records and gives the operator view; dl does the GPU work. Bags move
via the `real_robot/sftp_sync.sh` path already committed.

## cuVSLAM x86 support (verified from the README, not assumed)

Official prebuilt archives exist for desktop/server:

| Target | Ubuntu | CUDA | Archive |
|---|---|---|---|
| Desktop/server | 22.04 / 24.04 | 12.6.3 or 13.2.0 | `x86_64-cuda<ver>-ubuntu<ver>` |

The archive contains `bin/libcuvslam.so`, so no `nvcc` is needed. Source build
(`cmake -S . -B build`) is the fallback and does require CUDA 12/13.

Odometry modes available: multisensor RGB-D + IMU (needs a cuNLS build), plus
single-camera modes that work in the default build. Pinhole cameras only.

## NVBloX

Builds against CUDA and ROS 2; produces TSDF, ESDF, occupancy and costmap
layers, and subsumes the "build a map" requirement. Needs the same CUDA 12/13
container.

## What is not built yet

This directory documents the deployment; the container and the SLAM nodes are
not yet built. Blocking prerequisite, in order:

1. **ROS 2 bridge from the K1's FastDDS to ROS 2.** The K1 publishes
   `rt/low_state`, `rt/odometer_state`, `rt/fall_down`; cuVSLAM wants
   `sensor_msgs/Imu` + `CameraInfo`, nvblox wants RGB-D + TF. Without this there
   is nothing to fuse. `src/k1_control/k1_control/sdk_bridge_node.cpp` is the
   intended path; the Python one is a stub.
2. Camera calibration for whichever K1 camera is mounted (ZED class intrinsics
   exist in the P3 head config, but must be confirmed against the real sensor).
3. IMU rpy→quaternion conversion. `ImuState` is Euler; copying it into
   `sensor_msgs/Imu.orientation` gives a silently wrong rotation and a garbage
   map.

## Honest limits

- Nothing here has been run against a real K1 yet. The map pipeline is designed,
  not validated.
- cuVSLAM + nvblox on 4× RTX 6000 Ada is comfortable; expect the GPUs to be
  underused for a single rig. That is fine — headroom for multiple robots later.
- 3 m/s walking while building a live map is a much harder regime than either
  separately. Treat the first map as a slow-walk artifact.

---
> **Network note (2026-10-27):** the robotics router (192.168.50.1) has no internet uplink. The laptop shares its eduroam (`wlo1`) internet to the robots by NAT: `enp2s0` = 192.168.50.250/24 is the robots' gateway, with `ip_forward=1` and nft table `ip k1nat` (masquerade out `wlo1`, source 192.168.50.0/24). It is lost on laptop reboot; robots use DNS 8.8.8.8/1.1.1.1 (router DNS is wrong). Re-create steps: `docs/network_robotics_5G_2026-10-26.md` ("Robot internet via laptop").
