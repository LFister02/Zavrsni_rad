import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import csv

class VoxelCollector(Node):
    def __init__(self):
        super().__init__('save_pointcloud')
        self.occupied_voxels = set()

        # Subscribe na point cloud centere
        self.subscription = self.create_subscription(PointCloud2,'/octomap_point_cloud_centers', self.callback, 10)
        self.get_logger().info("Spremanje je pokrenuto.")

    def callback(self, msg):
        counter = 0
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            voxel = (round(point[0], 3), round(point[1], 3), round(point[2], 3))
            self.occupied_voxels.add(voxel)
            counter += 1

    def save_to_csv(self, filename = 'ime_datoteke.csv'):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z'])
            for voxel in self.occupied_voxels:
                writer.writerow(voxel)
        self.get_logger().info(f"Točke su uspješno spremljene!")

def main(args=None):
    rclpy.init(args=args)
    node = VoxelCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_to_csv()  # automatski spremi prilikom prekida
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
