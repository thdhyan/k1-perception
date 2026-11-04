#!/usr/bin/env bash
# Laptop: RViz on the SLAM running on a K1 (run_slam.sh). k1_slam_rx.py pulls the K1 relay's TCP stream and
# republishes it on a localhost-only ROS domain (no DDS over Wi-Fi: Humble<->Jazzy FastDDS leaked GBs on the K1).
# Robot model: real_robot/urdf/k1_robot_description.urdf (same link and joint names as the K1's /joint_states)
# through a local robot_state_publisher.
#   real_robot/slam_onboard/laptop_rviz.sh [robot_ip]     (default: A1)
D=$(dirname "$(readlink -f "$0")")
REPO=$(readlink -f $D/../..)
ROBOT_IP=${1:-192.168.50.82}
TMP=${XDG_RUNTIME_DIR:-/tmp}
# URDF meshes point at the robot's /opt/booster copy; use the booster_assets submodule instead.
sed "s|file:///opt/booster/Gait/configs/booster_assets/robots/K1/meshes/|file://$REPO/src/k1_description/assets/robots/K1/meshes/|" \
  $REPO/real_robot/urdf/k1_robot_description.urdf | sed '/<\/robot>/q' > $TMP/k1_rviz.urdf  # file ends with a stray "---"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${K1_VIZ_DOMAIN:-42} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 daemon stop >/dev/null 2>&1
# URDF comments/newlines break a -p value; pass it via a params file.
python3 -c "import json,sys; print('robot_state_publisher:\n  ros__parameters:\n    robot_description: '+json.dumps(open(sys.argv[1]).read()))" \
  $TMP/k1_rviz.urdf > $TMP/k1_rviz_rsp.yaml
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file $TMP/k1_rviz_rsp.yaml &
python3 $D/k1_slam_rx.py $ROBOT_IP &
python3 $D/k1_grid_map.py &
# Every session is recorded (REC=0 to skip): the relay stream. Map/costmap are left out (650 kB each per second);
# k1_grid_map.py rebuilds them from depth_points + odometry on bag replay.
if [ "${REC:-1}" = 1 ]; then
  ros2 bag record -o $REPO/real_robot/bags/a1_slam_$(date +%Y%m%d_%H%M%S) /k1_0/slam/odometry /k1_0/slam/path \
    /k1_0/slam/landmarks/global /k1_0/slam/landmarks/local /k1_0/slam/features/image/compressed /k1_0/slam/status \
    /k1_0/slam/depth_points /k1_0/collision /tf /tf_static /joint_states /robot_description \
    > /dev/null 2>&1 &
fi
trap 'kill -INT $(jobs -p) 2>/dev/null; wait' EXIT
rviz2 -d $D/k1_slam.rviz
