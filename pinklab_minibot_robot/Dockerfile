# Use the official ROS 2 Humble base image
FROM ros:humble

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install ultralytics (YOLOv5)
RUN pip3 install ultralytics

# Copy the ROS 2 package to the Docker image
COPY ai_server /ros2_ws/src/ai_server

# Install ROS 2 dependencies
RUN source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# Source the ROS 2 setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint
# ENTRYPOINT ["ros2", "launch", "robot_manage_pkg", "detection_launch.py"]
