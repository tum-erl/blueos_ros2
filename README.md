# BlueOS ROS2 Extension

This extension makes it possible to communicate/control a  BlueROV2 or a BlueBoat via [ROS2](https://github.com/ros2)

The extension includes the following packages:
- [MAVROS](https://github.com/mavlink/mavros): Includes a launch file to communicate with the vehicle
- [mavros_control](https://github.com/itskalvik/mavros_control): Includes a python script to control the vehicle using GPS-based waypoints (BlueBoat) and RC-control (BlueROV2) via mavros
- [Foxglove ](https://docs.foxglove.dev/docs): Web-based RViz for visualizing ROS2 data

The extension also mounts the following folder on the computer running BlueOS to the ```/home/``` directory in the extension's Docker container, which can be used to store files that need to persist across reboots, such as a ROS2 workspace with custom code:
```
/usr/blueos/extensions/ros2/
```

## Parameters
You can control the following extension parameters by running the following command in the terminal provided by the ros2 extension:

```
export <parameter_name>=<parameter_value>
```

The parameters reset to their default values after rebooting. They can be made permanent by configuring the parameters using the app environment variables on the BlueOS extensions page in pirate mode.

### Available Parameters: 

* ```NAVIGATION_TYPE``` (```default: 0```):
    - If ```0```: Uses global position based navigation (assumes access to GPS)
    - If ```1```: Uses raw rc controls for navigation

* ```FOXGLOVE``` (```default: True```):
    - Starts Foxglove bridge when set to ```True```
    - You can access it from a web browser at https://app.foxglove.dev/. Use the open connection feature and change the address from localhost to the IP address of the vehicle

## FAQ
To control the vehicle using RC-control, please ensure that the ```SYSID_MYGCS``` parameter is set to ```1``` using BlueOS. Note that once the ```SYSID_MYGCS``` is cnahged, you won't be able to control the vehicle using Cockpit anymore until you reset the parameter to its default value: ```255```.