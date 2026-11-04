#!/bin/bash
# Verify libcuvslam.so's CUDA dependencies resolve, without touching a GPU.
# `ldd` is a pure loader/ELF check, so this is safe to run while other jobs
# occupy the GPUs.
set -uo pipefail
SO="${1:-/opt/cuvslam/bin/libcuvslam.so}"
rc=0

echo "[check] $SO"
if [ ! -e "$SO" ]; then echo "[check] FAIL: not found"; exit 1; fi

echo "[check] unresolved shared libraries:"
missing=$(ldd "$SO" 2>/dev/null | grep "not found" || true)
if [ -n "$missing" ]; then
  echo "$missing" | sed 's/^/    /'
  echo "[check] FAIL -- this is the reason the container is required"
  rc=1
else
  echo "    (none)"
fi

echo "[check] CUDA libraries resolved:"
for lib in cudart cublas cusparse cusolver; do
  line=$(ldd "$SO" 2>/dev/null | grep -oE "lib${lib}\.so\.[0-9]+[^ ]*" | head -1)
  printf '    %-10s %s\n' "$lib" "${line:-MISSING}"
  [ -z "$line" ] && rc=1
done

echo "[check] host driver / GPU visibility (informational):"
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -2 | sed 's/^/    /' \
  || echo "    (no GPU visible to this container -- pass --gpus all)"

exit $rc
