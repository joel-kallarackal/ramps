#!/usr/bin/python3


import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

import numpy as np


import open3d as o3d
from sklearn.cluster import DBSCAN

from pcl_pub import PointCloudPublisher
from waypoint_pub import WaypointPublisher

from ctypes import *


class RampDetector(Node):

    def __init__(self):
        super().__init__('ramp_detector')
        self.imu_subscription = self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data',
            self.imu_callback,
            10)
        self.imu_subscription 
        self.add_on_set_parameters_callback(self.on_params_changed)
        self.on_ramp = False
        self.change_param = True

    def imu_callback(self,data: Imu):
        q = data.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x,q.y,q.z,q.w])
        
        
        
        if np.rad2deg(pitch) > 15 or np.rad2deg(pitch) < -15 and self.change_param:
            self.on_ramp = True
            self.change_param = False
            # set params for ramps
            param_str = Parameter('my_str', Parameter.Type.STRING, 'Set from code')
            param_int = Parameter('my_int', Parameter.Type.INTEGER, 12)
            param_double_array = Parameter('my_double_array', Parameter.Type.DOUBLE_ARRAY, [1.1, 2.2])
            self.set_parameters([param_str, param_int, param_double_array])
        
        else:
            self.on_ramp = False
            
        if not self.on_ramp and not self.change_param:
            # Set params back to normal
            param_str = Parameter('my_str', Parameter.Type.STRING, 'Set from code')
            param_int = Parameter('my_int', Parameter.Type.INTEGER, 12)
            param_double_array = Parameter('my_double_array', Parameter.Type.DOUBLE_ARRAY, [1.1, 2.2])
            self.set_parameters([param_str, param_int, param_double_array])
        
            
            
   
def main(args=None):
    rclpy.init(args=args)

    ramps = RampDetector()

    rclpy.spin(ramps)
    ramps.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()