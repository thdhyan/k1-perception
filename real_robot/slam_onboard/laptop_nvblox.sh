#!/usr/bin/env bash
# Laptop: nvblox (mesh, 2D occupancy, voxels) from a K1's SLAM, in docker (docker/Dockerfile.nvblox).
# Needs run_slam.sh on the K1 and laptop_rviz.sh here (it supplies map->trunk and the robot TF on domain 42).
# Starts k1_nvblox_stream.py on the K1 over ssh (stopped on exit); no DDS crosses Wi-Fi.
#   real_robot/slam_onboard/laptop_nvblox.sh [robot_ip] [ssh_host]     (defaults: A1, boosterk1.a1)
set -e
D=$(dirname "$(readlink -f "$0")")
REPO=$(readlink -f $D/../..)
ROBOT_IP=${1:-192.168.50.82}
K1=${2:-boosterk1.a1}
IMG=k1-nvblox:4.5
docker image inspect $IMG >/dev/null 2>&1 || docker build -t $IMG -f $REPO/docker/Dockerfile.nvblox $REPO/docker

scp -q $D/k1_nvblox_stream.py $K1:k1_slam/slam_onboard/
# Same DDS profile as run_slam.sh; killed first by the OOM killer, like the SLAM node.
ssh $K1 'source /opt/ros/humble/setup.bash; export FASTRTPS_DEFAULT_PROFILES_FILE=~/k1_slam/slam_onboard/fastdds_slam.xml
  echo 1000 > /proc/self/oom_score_adj; pkill -f "[k]1_nvblox_stream" || true
  setsid nohup nice -n 5 python3 ~/k1_slam/slam_onboard/k1_nvblox_stream.py ${ROBOT_NS:-k1_0} > ~/k1_slam/nvblox_stream.log 2>&1 < /dev/null &'
trap "ssh $K1 'pkill -f \"[k]1_nvblox_stream\"'" EXIT

xhost +local: >/dev/null 2>&1 || true
NS=${ROBOT_NS:-k1_0}
# Container talks to the host's domain-42 nodes over UDP loopback only (no shared memory across the boundary).
docker run --rm -it --gpus all --net=host --ipc=host \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $D:/k1:ro \
  -e ROS_DOMAIN_ID=${K1_VIZ_DOMAIN:-42} -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  $IMG bash -c "source /opt/ros/jazzy/setup.bash
    python3 /k1/k1_nvblox_rx.py $ROBOT_IP &
    ros2 run nvblox_ros nvblox_node --ros-args --params-file /k1/nvblox_laptop.yaml \
      -r camera_0/depth/image:=/$NS/slam/depth/image -r camera_0/depth/camera_info:=/$NS/slam/depth/camera_info \
      -r camera_0/color/image:=/$NS/slam/color/image -r camera_0/color/camera_info:=/$NS/slam/color/camera_info &
    rviz2 -d /k1/k1_nvblox.rviz
    kill \$(jobs -p)"
