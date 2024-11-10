FROM ros:$ROS_DISTRO-ros-base

RUN apt-get update \
    && apt-get install -q -y --no-install-recommends \
    tmux \
    nano \
    ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir setuptools pip packaging -U

COPY ros2_ws /home/ros2_ws
RUN cd /home/ros2_ws/ \
    && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build --symlink-install \
    && ros2 run mavros install_geographiclib_datasets.sh \
    && echo "source /home/ros2_ws/install/setup.sh " >> ~/.bashrc

# Add docker configuration
LABEL version="0.0.1"
LABEL permissions='{\
  "HostConfig": {\
    "Binds": [\
      "/dev:/dev:rw",
      "ros2_ws/src:/home/ros2_ws/src:rw",
    ],\
    "Privileged": true,\
    "NetworkMode": "host"\
  }\
}'
LABEL authors='[\
  {\
    "name": "Kalvik Jakkala",\
    "email": "itskalvik@gmail.com"\
  }\
]'
LABEL company='{\
  "about": "",\
  "name": "itskalvik",\
  "email": "itskalvik@gmail.com"\
}'
LABEL readme="https://raw.githubusercontent.com/itskalvik/blueos-ros2/master/README.md"
LABEL type="other"
LABEL tags='[\
  "ros2",\
  "robot"\
]'

ENTRYPOINT [ "/ros_entrypoint.sh" ]