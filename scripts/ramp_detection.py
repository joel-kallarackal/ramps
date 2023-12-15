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
        self.imu_subscription = self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data',
            self.get_orientation,
            10)
        self.imu_subscription
        self.cam_info_subscription = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/right/camera_info',
            self.get_cam_info,
            10)
        self.cam_info_subscription
        self.point_cloud_pub = PointCloudPublisher("cluster_point_cloud")

    def pc_listener_callback(self, msg):
        imu_data = self.imu_data
        cam_info = self.cam_info

        # Using ros2_numpy. Works for python 3.10 and greater only
        # Vector3dVector is optimized for numpy array with float64 datatype using memory mapping
        # pc_array = np.array(rnp.numpify(msg)["xyz"],dtype=np.float64)
        # pc_array_without_inf = pc_array[pc_array[:,0]!=np.inf]
        # pc = o3d.geometry.PointCloud()
        # pc.points = o3d.utility.Vector3dVector(pc_array_without_inf)

        # Without using ros2_numpy
        pc = self.convertCloudFromRosToOpen3d(msg)
        
        # Downsample the point cloud with a voxel of 0.05
        downpc = pc.voxel_down_sample(voxel_size=0.05)

        # Recompute the normals of the downsampled point cloud
        downpc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
        
        o3d.visualization.draw_geometries([downpc])
        o3d.visualization.draw_geometries([self.orient_cloud(downpc)])
        

        points = np.asarray(downpc.points)
        n = [0,0,-1]
        
        normals = np.asarray(downpc.normals)
        
        # Adding dummy point and the normal which we require
        points2 = np.append(points,np.array([[0,0,0],]),axis=0)
        normals2 = np.append(normals,np.array([n,]),axis=0)

        labels = self.cluster(normals2)
        clustered_points = points2[labels==labels[-1]]
        
        
        # Visualize Clusters
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(points2[labels==-1])
        # o3d.visualization.draw_geometries([pcd])
        
        # for i in range(8):
        #     pcd = o3d.geometry.PointCloud()
        #     pcd.points = o3d.utility.Vector3dVector(points2[labels==i])
        #     o3d.visualization.draw_geometries([pcd])
        
        
        self.point_cloud_pub.publish_cloud(clustered_points[:,0],clustered_points[:,1],clustered_points[:,2])
	
        print(labels)

        # Find midpoint of the cluster
        x_avg = np.average(clustered_points[:,0])
        y_avg = np.average(clustered_points[:,1])
        z_avg = np.average(clustered_points[:,2])

                
        
    def cluster(self,points):
        model = DBSCAN(eps=0.05, min_samples=20)
        model.fit_predict(points)
        pred = model.fit_predict(points)

        print("number of cluster found: {}".format(len(set(model.labels_))))
        print('cluster for each point: ', model.labels_)
        return model.labels_
    
    def convertCloudFromRosToOpen3d(self,ros_cloud):
    
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
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)
            
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
            print(len(np.array(xyz)[np.array(xyz)[:,2]<0.1]))
            # open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)

            
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

        # return
        return open3d_cloud
    
    def orient_cloud(self,cloud : o3d.geometry.PointCloud()):
        cloud = cloud.rotate(np.array([[0.877,-0.5,0],[0.5,0.877,0],[0,0,1]]), center=(0,0,0))
        return cloud
    def get_orientation(self,msg: Imu):
        self.imu_data = msg
    def get_cam_info(self,msg: CameraInfo):
        self.cam_info = msg


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