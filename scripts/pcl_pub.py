import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np


class PointCloudPublisher(Node):
    def __init__(self,topic = "/point_cloud"):
        super().__init__('point_cloud_publisher')
        self.pc_publisher = self.create_publisher(PointCloud2, topic, 10)
    def publish_cloud(self,x,y,z): 
        self.pc_publisher.publish(self.point_cloud(np.array([x,y,z]).T,"perpendicular"))

    def point_cloud(self,points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx3 array of xyz positions.
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        """
        ros_dtype = PointField.FLOAT32
        itemsize = np.dtype(np.float32).itemsize  # A 32-bit float takes 4 bytes.
        # data = points.astype(dtype).tobytes()
        data = np.array(points,dtype=np.float32).tobytes()
        fields = [PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyz')]
        header = Header()
        header.frame_id=parent_frame

        return PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3),  # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )
        

