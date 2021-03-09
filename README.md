# K3 Pro thermometer ROS2 Wrapper
This is a ros2 repository for the K3 Pro Non-Contact Auto Infrared Forehead Thermometer

## Dependencies

[RoMi-Health Messages](https://github.com/sharp-rmf/romi_health_msgs.git) 

For this ROS2 Wrapper, the temperature reading is published as part of the VSM message of RoMi-Health standardised messages. Therefore, the above repo is needed for this repo to work.

## Usage Instructions
Access to the port `/dev/ttyUSBX` must be given the user for the wrapper to work. `X` is the USB port number assigned to the thermometer. Run the following line in the terminal to provide the user access rights to the relevant USB port.
`sudo chown <username> -R /dev/ttyUSBX` 

To run the wrapper, run the following:
`ros2 run k3_thermometer_ros2 temperature_reader_script`
