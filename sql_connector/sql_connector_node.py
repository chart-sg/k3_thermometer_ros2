import mysql.connector 
import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from romi_health_msgs.msg import VSM

class sql_connector_node(Node):
    def __init__(self):
        super().__init__('k3_thermometer_sql_connector_node')

        # Subscriber
        self.temp_status_sub = self.create_subscription(String,
                                            '/temperature',
                                            self.temp_status_cb,
                                            10)
        self.cnx = mysql.connector.connect(host='localhost',
                                      user='root',
                                      password='chart123', 
                                      database='Medical_Device',
                                      port=3306,
                                      auth_plugin='mysql_native_password')
        print("Initialized.")


    def temp_status_cb(self, msg):
        
        print("[Callback] sending data.")
        therm_id = 'k3_pro'
        reading = float(msg.data)
        if (reading > 37.5):
            status = 'high'
        else:
            status = 'normal'

        try:
            cursor = self.cnx.cursor(prepared=True)
            add_temp_reading = ("INSERT INTO Temperature "
                "(therm_id, reading, status) " 
                "VALUES (%s, %s, %s)")
            data = (
                therm_id,
                reading,
                status)
            cursor.execute(add_temp_reading, data)
            self.cnx.commit()
            cursor.close()
        except Exception as e:
            print(e)

    def close(self):
        self.cnx.close()
    
    def start(self):
        try:
            while(rclpy.ok()):
                rclpy.spin_once(self)
        except KeyboardInterrupt:
            return


def main(args=None):
    rclpy.init(args=args)
    node = sql_connector_node()
    node.start()
    node.close()


if __name__ == "__main__":
    main()
