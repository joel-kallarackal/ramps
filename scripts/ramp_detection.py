#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

import numpy as np
import ros2_numpy as rnp

import open3d as o3d
from sklearn.cluster import DBSCAN

from pcl_pub import PointCloudPublisher

class PointCloudSubscriber(Node):

    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depthcam/zed2i/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.point_cloud_pub = PointCloudPublisher("cluster_point_cloud")

    def listener_callback(self, msg):

        # Vector3dVector is optimized for numpy array with float64 datatype using memory mapping
        pc_array = np.array(rnp.numpify(msg)["xyz"],dtype=np.float64)
        pc_array_without_inf = pc_array[pc_array[:,0]!=np.inf]
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(pc_array_without_inf)
        
        # Downsample the point cloud with a voxel of 0.05
        downpc = pc.voxel_down_sample(voxel_size=0.1)

        # Recompute the normals of the downsampled point cloud
        downpc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))

        points = np.asarray(downpc.points)
        n = [0,0,-1]
        
        normals = np.asarray(downpc.normals)
        
        # Adding dummy point and the normal which we require
        points2 = np.append(points,np.array([[0,0,0],]),axis=0)
        normals2 = np.append(normals,np.array([n,]),axis=0)

        labels = self.cluster(normals2)
        clustered_points = points2[labels==labels[-1]]
        
        '''
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points2[labels==-1])
        o3d.visualization.draw_geometries([pcd])
        
        for i in range(8):
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points2[labels==i])
            o3d.visualization.draw_geometries([pcd])
        
        '''
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