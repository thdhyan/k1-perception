#!/bin/bash
# Build + run the cuVSLAM/NVBloX container on dl.
#
#   ./run_slam_container.sh build          # build image (no GPU needed)
#   ./run_slam_container.sh check          # link check only, no GPU work
#   ./run_slam_container.sh bash           # interactive shell with ROS 2 mounted
#   ./run_slam_container.sh gpu 0          # pin to one GPU
#
# Deliberately does NOT run anything on a GPU by default. dl is shared and
# currently hosts Isaac Sim training and LLM evals; SLAM work is deferred until
# a GPU is genuinely free. `--gpus all` is opt-in per invocation.
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REMOTE_ROOT="${REMOTE_ROOT:-$HOME/k1_slam}"
IMG="${IMG:-k1-slam:cuvslam12}"
ACTION="${1:-check}"
GPU="${2:-}"

ssh_guard() { echo "[slam] NOTE: $1"; }

run() {
  ssh -o ClearAllForwardings=yes -o ConnectTimeout=20 dl "$@"
}

echo "[slam] action=$ACTION  remote=$REMOTE_ROOT  image=$IMG"

case "$ACTION" in
  build)
    # Plain names, not -R: an absolute $HERE with --relative would recreate the
    # whole local path tree on the remote and the build context would be empty.
    ( cd "$HERE" && rsync -az -e 'ssh -o ClearAllForwardings=yes' \
        Dockerfile check_cuvslam.sh dl:~/k1_slam/build/ )
    run "cd $REMOTE_ROOT/build && docker build -t $IMG . && echo '[slam] build OK'"
    ;;

  check)
    # No --gpus: a pure ldd check. Safe while the GPUs are busy.
    ( cd "$HERE" && rsync -az -e 'ssh -o ClearAllForwardings=yes' \
        check_cuvslam.sh dl:~/k1_slam/ )
    run "chmod +x $REMOTE_ROOT/check_cuvslam.sh && docker run --rm \
         -v $REMOTE_ROOT/cuvslam:/opt/cuvslam:ro \
         -v $REMOTE_ROOT/check_cuvslam.sh:/usr/local/bin/check_cuvslam.sh:ro \
         $IMG /usr/local/bin/check_cuvslam.sh /opt/cuvslam/bin/libcuvslam.so"
    ;;

  bash|gpu)
    GPUFLAG="--gpus all"
    [ "$ACTION" = "gpu" ] && [ -n "$GPU" ] && GPUFLAG="--gpus device=$GPU"
    ssh_guard "starting an interactive shell with $GPUFLAG"
    run "docker run --rm -it $GPUFLAG \
         --network host \
         --ipc host --shm-size=8g \
         -e DISPLAY=${DISPLAY:-} \
         -v /opt/ros/humble:/opt/ros/humble:ro \
         -v $REMOTE_ROOT:/k1_slam \
         -v $REMOTE_ROOT/cuvslam:/opt/cuvslam:ro \
         $IMG bash"
    ;;

  *)
    sed -n '2,10p' "$0"; exit 2 ;;
esac
