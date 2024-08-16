import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('point_cloud_filter')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lexus3/os_center/points',
            self.point_cloud_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            PointCloud2,
            '/lexus3/os_center/points/filtered',
            10
        )
        
        self.declare_parameter('z_threshold_lower', 1.0)
        self.declare_parameter('z_threshold_upper', 2.0)


    def point_cloud_callback(self, msg):
        filtered_points = []

        z_threshold_lower = self.get_parameter('z_threshold_lower').get_parameter_value().double_value
        z_threshold_upper = self.get_parameter('z_threshold_upper').get_parameter_value().double_value
        
        for point in pc2.read_points(msg, skip_nans=True):
            if point[2] > z_threshold_lower and point[2] < z_threshold_upper:
                filtered_points.append(point)

        if filtered_points:
            filtered_msg = pc2.create_cloud(
                msg.header, msg.fields, filtered_points
            )
            self.publisher_.publish(filtered_msg)
            self.get_logger().info(f'Point cloud between z={z_threshold_lower} and z={z_threshold_upper} published.')


def main(args=None):
    rclpy.init(args=args)

    filter = PointCloudFilter()

    rclpy.spin(filter)

    filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()