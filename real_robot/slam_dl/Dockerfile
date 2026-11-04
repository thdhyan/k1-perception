# cuVSLAM + NVBloX runtime for the K1 map pipeline (dl: Ubuntu 22.04, x86_64).
#
# Why a container at all: the prebuilt libcuvslam.so needs libcudart.so.12,
# libcublas.so.12, libcusparse.so.12 and libcusolver.so.11, while this host only
# has CUDA 11.7 (libcudart.so.11). The *driver* (590.48.01) already supports
# CUDA 13, so only the runtime is missing -- which is exactly what a container
# supplies. Installing CUDA 12 on the host would disturb every other job on this
# shared machine.
#
# ROS 2 Humble is NOT baked in: it is bind-mounted from the host so there is one
# copy of it and no version skew with the rest of the fleet.

FROM nvidia/cuda:12.6.3-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      libgomp1 \
      python3 python3-dev python3-pip \
      curl ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# cuVSLAM's prebuilt x86_64 archive (copied in by the run script, not baked in,
# so the 112 MB blob is not duplicated in every layer).
RUN mkdir -p /opt/cuvslam /k1_slam
WORKDIR /k1_slam

# Sanity: the loader must resolve every CUDA symbol libcuvslam.so needs.
COPY check_cuvslam.sh /usr/local/bin/check_cuvslam.sh
RUN chmod +x /usr/local/bin/check_cuvslam.sh

CMD ["/bin/bash"]
