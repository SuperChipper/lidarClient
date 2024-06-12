import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from math import pi
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion
import numpy as np

real_pose_2d=[0.,0.,0.]

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

        pick_num=int(len(scan.ranges)*180/360)#%len(scan.ranges)RAD2DEG(scan.angle_min)
        
        if pick_num<4 or pick_num>len(scan.ranges)-4:
            window=np.array([n for n in (scan.ranges[pick_num-2:]+scan.ranges[0:(pick_num+2)%len(scan.ranges)]) if n!=0])
        else:
            #window=np.array(scan.ranges[pick_num-2:pick_num+2])
            window=np.array([scan.ranges[i] for i in range(pick_num-2,pick_num+2) if scan.ranges[i]!=0])

        print('\r'+str(pick_num)+','+str(scan.ranges[pick_num])+','+str(len(scan.ranges))+','+str(np.mean(window)),end='')

class Subscriber_RealPath(Node):
    def __init__(self):
        super().__init__('Subscriber_RealPath')
        self.subscription = self.create_subscription(Path, 'r1_py_run_pkg/real', self.listener_callback, 10, callback_group=ReentrantCallbackGroup())
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global real_pose_2d
        real_pose_2d[0]=float(msg.poses[-1].pose.position.x)
        real_pose_2d[1]=float(msg.poses[-1].pose.position.y)
        q = msg.poses[-1].pose.orientation
        # _, _, real_location_list[2] = tf2_geometry_msgs.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        (roll, pitch, yaw_sub) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        real_pose_2d[2] = int(np.rad2deg(yaw_sub))            #self.get_logger().info(f'angle-distance : [{degree}, {scan.ranges[i]}]')

def main(args=None):
    rclpy.init(args=args)
    node = YDLidarROS2DriverClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
