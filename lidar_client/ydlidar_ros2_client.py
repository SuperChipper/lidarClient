import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from math import pi
#import numpy as np
def RAD2DEG(x):
    return int(x * 180.0 / pi)

class YDLidarROS2DriverClient(Node):

    def __init__(self):
        super().__init__('client')
        self.lidar_info_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_cb,
            rclpy.qos.qos_profile_sensor_data
        )

    def scan_cb(self, scan):
        count = int(scan.scan_time / scan.time_increment)
        #self.get_logger().info(f'I heard a laser scan {scan.header.frame_id}[{count}]:')
        #self.get_logger().info(f'angle_range : [{RAD2DEG(scan.angle_min)}, {RAD2DEG(scan.angle_max)}]')

        
        print('\r'+str(RAD2DEG(scan.angle_min + scan.angle_increment * 180))+','+str(scan.ranges[180])+','+str(len(scan.ranges))+','+str(scan.angle_increment),end='')
            #self.get_logger().info(f'angle-distance : [{degree}, {scan.ranges[i]}]')

def main(args=None):
    rclpy.init(args=args)
    node = YDLidarROS2DriverClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
