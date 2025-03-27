from autoware_perception_msgs.msg import PredictedObjects
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header


class DummyPerceptionPublisher(Node):
    def __init__(self):
        super().__init__("dummy_perception_publisher")

        # Publisher for PointCloud2
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, "/perception/obstacle_segmentation/pointcloud", 10
        )
        self.pointcloud_timer = self.create_timer(1.0, self.publish_dummy_perception)

        # Publisher for PredictedObjects
        self.objects_publisher = self.create_publisher(
            PredictedObjects, "/perception/object_recognition/objects", 10
        )
        self.objects_timer = self.create_timer(0.1, self.publish_dummy_objects)  # 10 Hz

        # Publisher for OccupancyGrid
        self.occupancy_grid_publisher = self.create_publisher(
            OccupancyGrid, "/perception/occupancy_grid_map/map", 10
        )
        self.occupancy_grid_timer = self.create_timer(1.0, self.publish_dummy_occupancy_grid)

    def publish_dummy_perception(self):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = "base_link"
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="channel", offset=20, datatype=PointField.UINT8, count=1),
        ]
        msg.is_bigendian = False
        msg.is_dense = False

        self.pointcloud_publisher.publish(msg)
        self.get_logger().info("Publishing dummy perception data")

    def publish_dummy_objects(self):
        msg = PredictedObjects()
        msg.header = Header()
        msg.header.frame_id = "base_link"
        msg.objects = []

        self.objects_publisher.publish(msg)
        self.get_logger().info("Publishing dummy predicted objects")

    def publish_dummy_occupancy_grid(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = "base_link"
        msg.info.origin.orientation.w = 1.0
        msg.data = []

        self.occupancy_grid_publisher.publish(msg)
        self.get_logger().info("Publishing dummy occupancy grid map")


def main(args=None):
    rclpy.init(args=args)
    dummy_perception_publisher = DummyPerceptionPublisher()
    rclpy.spin(dummy_perception_publisher)
    dummy_perception_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
