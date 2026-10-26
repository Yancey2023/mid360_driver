ARG ROS_DISTRO=humble
ARG ROS_WORKSPACE=/opt/ros_ws

FROM ros:${ROS_DISTRO}-ros-base AS builder
ARG ROS_DISTRO
ARG ROS_WORKSPACE

RUN --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends --no-install-suggests libasio-dev

WORKDIR ${ROS_WORKSPACE}
COPY . .
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    rosdep -y --ignore-src install --from-paths .  && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM ros:${ROS_DISTRO}-ros-core AS runner
ARG ROS_DISTRO
ARG ROS_WORKSPACE
ENV ROS_WORKSPACE=${ROS_WORKSPACE}

COPY --from=builder ${ROS_WORKSPACE}/install/ ${ROS_WORKSPACE}/install/

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source $ROS_WORKSPACE/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "mid360_driver", "mid360_driver.launch.py"]