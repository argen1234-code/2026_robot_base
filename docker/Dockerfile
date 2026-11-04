# 从已固化的快照镜像派生，只补 entrypoint / 标签 / 环境变量。
#
# 基础镜像 ros_humble_ai:v1.0 是历史导入镜像，无法从零复现；
# 可复现边界 = 基础镜像 + docker commit 快照 + base-image-manifest.txt。
ARG BASE_IMAGE=yahboom/ros-humble-ai:snapshot-20261104
FROM ${BASE_IMAGE}

LABEL org.opencontainers.image.title="2026 robot AI stack"
LABEL org.opencontainers.image.description="ROS 2 Humble + ASR/TTS/LLM + Orbbec runtime"

ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV MODELS_DIR=/root/yahboom_ws/src/largemodel/MODELS

COPY entrypoint-ai.sh /usr/local/bin/entrypoint-ai.sh
RUN chmod 0755 /usr/local/bin/entrypoint-ai.sh

WORKDIR /root
CMD ["sleep", "infinity"]
