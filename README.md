# K3 Pro thermometer ROS2 Adapter
This is a ros2 adapter for the K3 Pro Non-Contact Auto Infrared Forehead Thermometer which is developed by Centre for Healthcare Assistive & Robotics Technology ([CHART]). The adapter extracts temperature information via low-level USB Serial and publishes the relevant information in the form of [RoMi-Health Messages]. The temperature reading will be published as part of the VSM message of RoMi-Health standardised messages.

## Pre-requisites
The following are packages/messages required by the adapter
- [RoMi-Health Messages](https://github.com/sharp-rmf/romi_health_msgs.git) 

This package has been tested to be working on:
- Ubuntu 20.04
- [ROS2 Galactic]


## Usage Instructions

### Port Access
Access to the port `/dev/ttyUSBX` must be given the user for the wrapper to work. `X` is the USB port number assigned to the thermometer. Run the following line in the terminal to provide the user access rights to the relevant USB port.
```
sudo chown <username> -R /dev/ttyUSBX
``` 

### Adapter Execution
To run the adapter, run the following:
```
ros2 run k3_thermometer_ros2 temperature_reader_script
```


[CHART]: <https://www.cgh.com.sg/CHART>
[RoMi-H Messages]: https://github.com/sharp-rmf/romi_health_msgs
[romi_health_msgs.msg]: https://github.com/sharp-rmf/romi_health_msgs
[ROS2 Galactic]: <https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html>
