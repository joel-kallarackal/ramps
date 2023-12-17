#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Imu, CameraInfo

import numpy as np
import ros2_numpy as rnp

import open3d as o3d
from sklearn.cluster import DBSCAN

from pcl_pub import PointCloudPublisher

from sensor_msgs_py import point_cloud2 as pc2
from ctypes import *


class PointCloudSubscriber(Node):

    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.pc_subscription = self.create_subscription(
            PointCloud2,
            '/zed/zed_node/point_cloud/cloud_registered',
            self.pc_listener_callback,
            10)
        self.pc_subscription  # prevent unused variable warning
        self.point_cloud_pub = PointCloudPublisher("cluster_point_cloud")

    def pc_listener_callback(self, msg: PointCloud2):
        '''
        Callback function of the subscriber which listens to the point cloud published.
        The point cloud is downsampled, it's normals are calculated and they are clustered.
        Then a waypoint is published by determining a characteristic point of the cluster.

        Input
        :param msg: PointCloud2 message
        '''

        #########################################################################################
        # Using ros2_numpy. Works for python 3.10 and greater only
        # Vector3dVector is optimized for numpy array with float64 datatype using memory mapping
        # pc_array = np.array(rnp.numpify(msg)["xyz"],dtype=np.float64)
        # pc_array_without_inf = pc_array[pc_array[:,0]!=np.inf]
        # pc = o3d.geometry.PointCloud()
        # pc.points = o3d.utility.Vector3dVector(pc_array_without_inf)
        ##########################################################################################

        # Without using ros2_numpy
        pc = self.convertCloudFromRosToOpen3d(msg)
        
        #Fixing orientation of zed point cloud
        #zed's tilt = 0.37 radians
        pc2 = self.orient_cloud(pc,[[0,-1,0],[1,0,0],[0,0,1]]) # 90 Degree rotation about z-axis
        pc3 = self.orient_cloud(pc,[[1,0,0],[0,-0.3616,0.9323],[0,-0.9323,-0.3616]]) # (-pi/2-0.37) radians rotation about x-axis

        # Downsample the point cloud with a voxel of 0.05
        downpc = pc3.voxel_down_sample(voxel_size=0.05)

        # Recompute the normals of the downsampled point cloud
        downpc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
        
        points = np.asarray(downpc.points)
        n1 = np.array([0,0.6,0.8])
        n2 = -n1 # normal in the opposite direction

        normals = np.asarray(downpc.normals)
        
        # Adding dummy point and the normal which we require
        # We take into account normals pointing in opposite direction also since the goal here is to determine the plane
        points2 = np.append(points,np.array([[0,0,0],]),axis=0)
        normals2 = np.append(normals,np.array([n1,]),axis=0)
        points3 = np.append(points,np.array([[0,0,0],]),axis=0)
        normals3 = np.append(normals,np.array([n2,]),axis=0)

        labels = self.cluster(normals2)
        clustered_points = points2[labels==labels[-1]]

        labels2 = self.cluster(normals3)
        clustered_points2 = points2[labels2==labels2[-1]]
        
        
        # find the midpoint of the required cluster and publish waypoint
        pcd = o3d.geometry.PointCloud()
        x_avg,y_avg,z_avg = 0,0,0
        if((labels[-1]!=-1) and (labels2[-1]!=-1)):
            pcd.points = o3d.utility.Vector3dVector(np.append(clustered_points,clustered_points2,axis=0))
            x_avg = np.average(np.append(clustered_points,clustered_points2,axis=0)[:,0])
            y_avg = np.average(np.append(clustered_points,clustered_points2,axis=0)[:,1])
            z_avg = -np.average(np.append(clustered_points,clustered_points2,axis=0)[:,2])
            clustered_pc = np.append(clustered_points,clustered_points2,axis=0)
            self.point_cloud_pub.publish_cloud(clustered_pc[:,0],clustered_pc[:,1],clustered_pc[:,2])
	
        elif (labels[-1]!=-1):
            pcd.points = o3d.utility.Vector3dVector(clustered_points)
            x_avg = np.average(clustered_points[:,0])
            y_avg = np.average(clustered_points[:,1])
            z_avg = -np.average(clustered_points[:,2])
            self.point_cloud_pub.publish_cloud(clustered_points[:,0],clustered_points[:,1],clustered_points[:,2])
            
        elif (labels2[-1]!=-1):
            pcd.points = o3d.utility.Vector3dVector(clustered_points2)
            x_avg = np.average(clustered_points2[:,0])
            y_avg = np.average(clustered_points2[:,1])
            z_avg = -np.average(clustered_points2[:,2])
            self.point_cloud_pub.publish_cloud(clustered_points2[:,0],clustered_points2[:,1],clustered_points2[:,2])
        
    def cluster(self,points):
        '''
        Returns clusters from 3D data

        Input
        :param point: nx3 array where n is the number of 3D points present

        :return:  An array of labels where each point is labelled into a cluster. 
                Labelled -1 if it does not belong to any cluster
        '''

        model = DBSCAN(eps=0.05, min_samples=20)
        model.fit_predict(points)
        pred = model.fit_predict(points)

        print("number of cluster found: {}".format(len(set(model.labels_))))
        print('cluster for each point: ', model.labels_)
        return model.labels_
    
    def convertCloudFromRosToOpen3d(self,ros_cloud: PointCloud2):
        '''
        Converts PointCloud2 ROS message to Open3D point cloud

        Input
        :param ros_cloud: PointCloud2

        :return:  An Open3D point cloud
        '''
    
        # Get cloud data from ros_cloud
        field_names=[field.name for field in ros_cloud.fields]
        cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

        # Check empty
        open3d_cloud = o3d.geometry.PointCloud()
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] 

            # Get rgb
            # Check whether int or float
            # convert_rgbUint32_to_tuple = lambda rgb_uint32: (
            #     (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff))
            # convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
            #     int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value))

            # convert_rgbUint32_to_tuple = lambda rgb_uint32: (
            #     (np.bitwise_and(rgb_uint32,0x00ff0000))>>16, (np.bitwise_and(rgb_uint32 & 0x0000ff00))>>8, (np.bitwise_and(rgb_uint32 & 0x000000ff)))
            # convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
            #     int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value))

            
            # if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            #     rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            # else:
            #     rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            

            # combine
            # open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz)[np.array(xyz)[:,2]<0.5])
            # open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)

            
        else:
            xyz = [(z,y,x) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

        return open3d_cloud
    
    def orient_cloud(self,cloud : o3d.geometry.PointCloud(), R):
        '''
        Rotate Open3D Pointcloud

        Input
        :param cloud: Open3D PointCloud
        :param R: Rotation matrix (3x3 array)

        :return:  An Open3D point cloud which is rotated as per the 
                rotation matrix R
        '''
        cloud = cloud.rotate(R, center=(0,0,0))
        return cloud
 
    def quaternion_to_rotation_matrix(self,Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix



def main(args=None):
    rclpy.init(args=args)

    point_cloud_sub = PointCloudSubscriber()

    rclpy.spin(point_cloud_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    point_cloud_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()