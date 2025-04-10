FROM ros:noetic

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-catkin-tools \
    python3-rospy \
    python3-sensor-msgs \
    python3-tf2-ros \
    python3-tf \
    ros-noetic-pcl-ros \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-image-transport-codecs \
    ros-noetic-image-transport-dbgsym \
    ros-noetic-image-transport-codecs-dbgsym \
    ros-noetic-image-transport-plugins \
    ros-noetic-grid-map \
    ros-noetic-velodyne \
    ros-noetic-tf2-ros \
    ros-noetic-nodelet \
    ros-noetic-velodyne-pointcloud \
    ros-noetic-grid-map-core \
    ros-noetic-grid-map-ros \
    ros-noetic-grid-map-cv \
    ros-noetic-grid-map-loader \
    ros-noetic-grid-map-msgs \
    ros-noetic-grid-map-rviz-plugin \
    ros-noetic-grid-map-visualization \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies
COPY groundgrid/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Set up catkin workspace
RUN mkdir -p /home/workspace/src
WORKDIR /home/workspace/

# Source the workspace
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /home/workspace/devel/setup.bash" >> ~/.bashrc

# Set the default command
CMD ["bash"]
