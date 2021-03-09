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
from romi_health_msgs.msg import VSM

class TemperatureAdapter(Node):
    def __init__(self):
        super().__init__('temperature_adapter_node')
        self.publisher = self.create_publisher(VSM, 'patient_vital_signs', 10)
        ser = serial.Serial()
        ser.port = '/dev/ttyUSB0'
        ser.baudrate = 115200
        ser.timeout = 0
        self.key = 'ambience compensate\r\nT body = '
        self.key_len = len(self.key)
        self.temp_db_url = '' ## <REST API get URL>

        try:
            self.initialise(ser)
            self.reading_check(ser)
        except (serial.SerialException, serial.SerialTimeoutException) as e:
            print("Serial Port could not be opened!")
            print(e)
        finally:
            ser.close()


    def initialise(self, ser):
        try:
            print("Attempting to open and read K3 Pro Thermometer ...")
            ser.close()
            ser.open()
        except Exception as e:
            print(e)
            return

    def reading_check(self, ser):
        while True:
            try:
                data = ser.read(9999)
                # print(data)
                if len(data) > 1:
                    body_temp_index = int(data.find(self.key.encode()))
                    if body_temp_index > 0:
                        byte_temp = data[body_temp_index + self.key_len : body_temp_index + self.key_len + 6] 
                        temp_full = float(byte_temp.decode())
                        temp = round(temp_full, 1)

                        ## Here the user can choose to upload the temperature 
                        ## through REST APIs or as a ROS2 topic  
                        # self.upload(temp)
                        self.ros_pub(temp)

                time.sleep(1)
            except Exception as e:
                print(e)
                break
            except KeyboardInterrupt:
                print("Keyboard Interrupt registered.")
                break

    def upload(self, temp):
        ## This is a REST get function to upload the temperature readings
        try:
            print("Uploading Recorded Temperature, Please Hold ...")
            msg = str(temp)
            print(msg)
            response = requests.get(self.temp_db_url + str(temp))
                   
        except Exception as e:
            print(e)
            return
 
    def ros_pub(self, temp):
        ## This is ROS2 publishing function to publish the temperature readings as part of the RoMi-H VSM message
        try:
            msg = VSM()
            msg.update_time = self.get_clock().now().to_msg()
            msg.temperature = temp
            # print(temp)
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