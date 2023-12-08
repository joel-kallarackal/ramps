#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped,Pose,Point
from std_msgs.msg import Header

import numpy as np


class WaypointPublisher(Node):
    def __init__(self,topic = "/goal_pose",node_name="waypoint_publisher"):
        super().__init__(node_name)
        self.waypoint_publisher = self.create_publisher(PoseStamped, topic, 10)
    def publish_waypoint(self,x,y,z,parent_frame="map"):
        header = Header()
        header.frame_id=parent_frame
        pose = Pose()
        pose.position = Point(x=x,y=y,z=z)
        pose_stamped = PoseStamped(header=header,pose=pose)
        self.waypoint_publisher.publish(pose_stamped)