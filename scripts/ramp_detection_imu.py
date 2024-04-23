#!/usr/bin/python3


import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

import numpy as np
import subprocess
import threading
import signal
import sys
import os


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
        # self.add_on_set_parameters_callback(self.on_params_changed)
        self.on_ramp = False
        self.change_param = True
        
    def imu_callback(self,data: Imu):
        q = data.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x,q.y,q.z,q.w])

        if np.rad2deg(pitch) < 7 or np.rad2deg(pitch) > 97:
            if self.change_param:
                self.change_param = False
                # set params for ramps
                self.start_subprocess("ros2 param set /twist_mux topics.lanefollowing.topic meaw!")
        else:
            if not self.change_param:
                # Set params back to normal
                self.start_subprocess("ros2 param set /twist_mux topics.lanefollowing.topic /lane/cmd_vel")
                self.change_param = True

        
    def start_subprocess(self,command) -> None:
        global subprocesses
        process = subprocess.Popen(command, shell=True)
        print(f"\033[92m Launching: {''.join(command)}\033[0m") 
        
    def stop_subprocesses(self) -> None:
        global subprocesses
        for process in subprocesses:
            process.send_signal(signal.SIGINT)
        for process in subprocesses:
            process.wait()
        print("\033[92m Stopping processes\033[0m")  
            
   
def main(args=None):
    rclpy.init(args=args)

    ramps = RampDetector()

    rclpy.spin(ramps)
    ramps.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()