# BlueOS ROS2 Extension

This extension enables controlling an ArduPilot-based vehicle (BlueBoat, BlueROV) via [ROS2](https://github.com/ros2)

It is a modified version from this one which adapts to the needs of the Environmental Robotics Lab of the Chair of Robotics and System Intelligence at TUM.

The extension includes the following packages:

- [MAVROS](https://github.com/mavlink/mavros): Includes a launch file to communicate with the flight controller (vehicle)

- [Foxglove ](https://docs.foxglove.dev/docs): Web-based RViz for visualizing ROS2 data

The extension also mounts the following folder on the computer running [BlueOS](https://blueos.cloud/) to the home directory folder in the extension's Docker container, which can be used to store files that need to persist across reboots, such as a ROS2 workspace with custom code:

```/usr/blueos/extensions/ros2/``` -> ```/home/persistent_ws/```

## Setup
This extension works only on 64-bit version of [BlueOS](https://github.com/bluerobotics/BlueOS). You can get the 64-bit image of BlueOS for Raspberry Pi from [here](https://github.com/bluerobotics/BlueOS/releases). Make sure to select the Raspberry Pi 5 version which uses 64-bit for the operating system.

Right now this extension needs to be installed manually in the extension store.

## Prerequisites
The ROS2 environment of this container uses following settings:
- ROS_DOMAIN_ID: 42

- RMW_IMPLEMENTATION: rmw_cyclonedds_cpp

- custom cyclonedds.xml to enable communication with other ROS2 nodes in the BaseStation network.


## Building the Docker Container Locally
First, setup buildx to build the containers for both arm64 and amd64 platforms: 

```bash
docker buildx create --name multi-arch \
  --platform "linux/arm64,linux/amd64" \
  --driver "docker-container"
docker buildx use multi-arch
```

Next, clone the repo and build the container, replace ```<tag>``` with your own tag:

```bash
git clone --recurse-submodules https://github.com/itskalvik/blueos-ros2
cd blueos-ros2
docker buildx build --platform linux/amd64,linux/arm64 -t <your_docker_hub_name>/blueos_ros2:<version> . --push
```

## FAQ
To control the vehicle using RC-control, please ensure that the ```SYSID_MYGCS``` parameter is set to ```1``` through the BlueOS Autopilot Parameters Tab.
You can also update ```SYSID_MYGCS``` from the ROS2 extension's terminal by runnning the following command:
```
ros2 param set /mavros/param SYSID_MYGCS 1
```

Note that once the ```SYSID_MYGCS``` is changed, you won't be able to control the vehicle using Cockpit anymore until you reset the parameter to its default value: ```255```

## BlueOS Settings
Extension Identifier: TUM.ERL</br>
Extension Name: ROS2</br>
Docker image: 0401marko/blueos_ros2</br>
Docker tag: 0.0.1 (make sure to use the latest)</br>

{
  "NetworkMode": "host",
  "HostConfig": {
    "Binds": [
      "/dev:/dev:rw",
      "/usr/blueos/extensions/ros2/:/home/persistent_ws/:rw"
    ],
    "Privileged": true,
    "NetworkMode": "host"
  }
}
