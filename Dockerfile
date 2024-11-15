ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO-ros-base

RUN rm /var/lib/dpkg/info/libc-bin.* \
    && apt-get clean \
    && apt-get update \
    && apt-get -y install libc-bin \
    && apt-get install -q -y --no-install-recommends \
    tmux nano nginx wget netcat \
    ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs \
    python3-dev python3-pip \
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

# Setup ttyd
ADD files/install-ttyd.sh /install-ttyd.sh
RUN bash /install-ttyd.sh && rm /install-ttyd.sh
COPY files/tmux.conf /etc/tmux.conf

RUN mkdir -p /site
COPY files/register_service /site/register_service
COPY files/nginx.conf /etc/nginx/nginx.conf

ADD files/start.sh /start.sh

# Add docker configuration
LABEL version="v0.0.1"
LABEL permissions='{\
  "HostConfig": {\
    "Binds": [\
      "/dev:/dev:rw",\
      "ros2_ws/src:/home/ros2_ws/src:rw"\
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