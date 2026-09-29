FROM osrf/ros:humble-desktop

# 安装基础编译与调试工具
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    ros-humble-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# 设置工作空间目录
ENV WS_DIR=/root/ros2_ws
WORKDIR ${WS_DIR}

# 默认加载 ROS 2 环境
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 拷贝代码到工作空间
COPY . ${WS_DIR}/src/sheep_patrol

# 编译项目
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
RUN echo "source ${WS_DIR}/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
