#!/usr/bin/env/python3
######## THIS IS A REST and/or ROS2 ADAPTER FOR THE K3 Pro Thermometer #######
## NOTE : K3 Thermometer has a heartbeat ambient temperature checker of 10s ##

import rclpy
from rclpy.node import Node
import serial
import time
import requests
import re
from std_msgs.msg import String

class TemperatureAdapter(Node):
    def __init__(self):
        super().__init__('mock_temperature_adapter_node')
        self.publisher = self.create_publisher(String, 'temperature', 10)

        try:
            self.reading_check()
        except Exception as e:
            print(e)
            return

    def reading_check(self):
        while True:
            try:
                temp = input('Enter your temperature: ')
                self.ros_pub(temp)
                time.sleep(1)
            except Exception as e:
                print(e)
                break
            except KeyboardInterrupt:
                print("Keyboard Interrupt registered.")
                break
                
    def ros_pub(self, temp):
        ## This is ROS2 publishing function to publish the temperature readings as part of the RoMi-H VSM message
        try:
            msg = String()
            msg.data = temp
            self.publisher.publish(msg)
            print('Publishing Recorded Temperature, Please Hold ...')
        except Exception as e:
            print(e)
            return

def main(args=None):
    rclpy.init(args=args)
    k3 = TemperatureAdapter()
    rclpy.spin(k3)
    k3.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
