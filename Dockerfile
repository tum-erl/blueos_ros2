ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO-ros-base

RUN apt-get update \
    && apt-get install -q -y --no-install-recommends \
    tmux nano nginx wget netcat \
    ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-dev python3-pip \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir setuptools pip packaging -U

# Set up ROS2 environment
COPY cyclonedds.xml /root/cyclonedds.xml

ENV CYCLONEDDS_URI=file:///root/cyclonedds.xml \
    ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && ros2 run mavros install_geographiclib_datasets.sh \
    && echo "source /ros_entrypoint.sh" >> ~/.bashrc

# Setup ttyd for web terminal interface
ADD files/install-ttyd.sh /install-ttyd.sh
RUN bash /install-ttyd.sh && rm /install-ttyd.sh
COPY files/tmux.conf /etc/tmux.conf

RUN mkdir -p /site
COPY files/register_service /site/register_service
COPY files/nginx.conf /etc/nginx/nginx.conf

ADD files/start.sh /start.sh

# Add docker configuration
LABEL version="0.0.2"
LABEL permissions='{\
  "NetworkMode": "host",\
  "HostConfig": {\
    "Binds": [\
      "/dev:/dev:rw",\
      "/usr/blueos/extensions/ros2/:/home/persistent_ws/:rw"\
    ],\
    "Privileged": true,\
    "NetworkMode": "host"\
  },\
  "Env": [\
    "NAVIGATION_TYPE=0", \
    "FOXGLOVE=True" \
  ]\
}'
LABEL authors='[\
  {\
    "name": "Marko Alten",\
    "email": "marko.alten@tum.de"\
  }\
]'
LABEL company='{\
  "about": "",\
  "name": "Technical University of Munich | Chair of Robotics and System Intelligence",\
  "email": ""\
}'
LABEL readme="https://raw.githubusercontent.com/tum-erl/blueos-ros2/refs/heads/main/README.md"
LABEL type="other"
LABEL tags='[\
  "ros2",\
  "robot"\
]'

# Keep bash alive even if there is an error
RUN echo "set +e" >> ~/.bashrc
ENTRYPOINT [ "/start.sh" ]