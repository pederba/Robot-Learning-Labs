# Use the official ROS base image
FROM ros:noetic

# Set the working directory
WORKDIR /workspace

# Copy your ROS project into the container
COPY . .

# Install any additional dependencies
# For example, if you have a ROS package with dependencies, install them here

# Install build tools
RUN apt-get update && \
    apt-get install -y build-essential

# Install rqt and its plugins
RUN apt-get install -y ros-noetic-rqt ros-noetic-rqt-common-plugins

# Set up display for graphical tools
ENV DISPLAY=:0
RUN apt-get install -y x11-apps


# Source ROS setup.bash
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
