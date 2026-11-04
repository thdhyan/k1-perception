# real_robot/slam_onboard

Part of [k1-perception](https://github.com/thdhyan/k1-perception).

| File | What it does |
|---|---|
| `k1_cuvslam_node.py` | Run ON a K1 (humble, ~/k1_slam/venv with PyCuVSLAM 17.0.1+cu12). cuVSLAM RGB-D SLAM from the head camera. |
| `k1_grid_map.py` | Laptop: 2-D occupancy map, costmap and collision marker from the K1 SLAM stream (stand-in until nvblox). |
| `k1_nvblox_rx.py` | Laptop (inside the nvblox container): receive k1_nvblox_stream.py (K1, TCP 7448) and republish depth, colour, camera_info and the head camer |
| `k1_nvblox_stream.py` | Run ON a K1 next to run_slam.sh: streams nvblox inputs to the laptop (k1_nvblox_rx.py) over TCP 7448. |
| `k1_slam_relay.py` | Run ON a K1 next to run_slam.sh: streams the SLAM visualisation topics to the laptop (k1_slam_rx.py) over TCP. |
| `k1_slam_rx.py` | Laptop: receive the K1 SLAM viz stream (k1_slam_relay.py on the robot, TCP 7447) and republish it on the local ROS domain for RViz. Reconnec |
| `laptop_nvblox.sh` | Laptop: nvblox (mesh, 2D occupancy, voxels) from a K1's SLAM, in docker (docker/Dockerfile.nvblox). |
| `laptop_rviz.sh` | Laptop: RViz on the SLAM running on a K1 (run_slam.sh). k1_slam_rx.py pulls the K1 relay's TCP stream and |
| `run_slam.sh` | Run ON the K1: cuVSLAM RGB-D node + viz relay (+ nvblox if NVBLOX=1). Needs k1-tf.service active. Ctrl-C stops all. |
