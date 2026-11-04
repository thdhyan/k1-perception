#!/usr/bin/env bash
# Run ON the K1: cuVSLAM RGB-D node + viz relay (+ nvblox if NVBLOX=1). Needs k1-tf.service active. Ctrl-C stops all.
#   scp -r real_robot/slam_onboard boosterk1.a1:~/k1_slam/ && ssh -t boosterk1.a1 ~/k1_slam/slam_onboard/run_slam.sh
set -e
source /opt/ros/humble/setup.bash
D=$(dirname "$(readlink -f "$0")")
NS=${ROBOT_NS:-k1_0}
LOG=~/k1_slam/logs/$(date +%Y%m%d_%H%M%S); mkdir -p $LOG
systemctl is-active --quiet k1-tf || { echo "k1-tf.service not active"; exit 1; }
# Camera arrives over UDP; let fastdds_slam.xml ask for 8 MB socket buffers (runtime only, lost on reboot).
[ "$(sysctl -n net.core.rmem_max)" -ge 8388608 ] || sudo -n sysctl -q -w net.core.rmem_max=8388608 net.core.wmem_max=8388608
export FASTRTPS_DEFAULT_PROFILES_FILE=$D/fastdds_slam.xml
# GPU memory is system RAM on Jetson: if it runs out, the kernel must kill SLAM, never the walking stack.
echo 1000 > /proc/self/oom_score_adj
nice -n 5 ~/k1_slam/venv/bin/python $D/k1_cuvslam_node.py --ros-args -p robot_ns:=$NS 2>&1 | tee $LOG/cuvslam.log &
# nvblox on the K1 costs ~2 GB of RAM and most of the GPU; off unless NVBLOX=1.
[ "${NVBLOX:-0}" = 1 ] && nice -n 5 ros2 run nvblox_ros nvblox_node --ros-args --params-file $D/nvblox_k1.yaml \
  -r camera_0/depth/image:=/$NS/slam/depth/image -r camera_0/depth/camera_info:=/$NS/slam/depth/camera_info \
  -r camera_0/color/image:=/$NS/slam/color/image -r camera_0/color/camera_info:=/$NS/slam/color/camera_info \
  > $LOG/nvblox.log 2>&1 &
# Laptop view (laptop_rviz.sh) reads only this relay's domain, never the SLAM publishers directly.
nice -n 5 python3 $D/k1_slam_relay.py $NS > $LOG/relay.log 2>&1 &
trap 'kill $(jobs -p) 2>/dev/null; wait' INT TERM EXIT
echo "logs: $LOG"
wait
