import math

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


class PoseSubscriber(Node):

    def __init__(self):
        super().__init__("pose_subscriber")
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/sensing/gnss/pose_with_covariance",  # Replace with your topic name
            self.listener_callback,
            10,
        )

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            "/sensing/gnss/pose_with_covariance_modified",  # Replace with your desired output topic name
            10,
        )

        # Initialize previous position
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = 0.0

    def listener_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_yaw = 0.0
        # Calculate yaw if previous position is available
        if self.prev_x is not None and self.prev_y is not None:
            delta_x = current_x - self.prev_x
            delta_y = current_y - self.prev_y
            if delta_x <= 0.05 and delta_y <= 0.05:
                current_yaw = self.prev_yaw
            else:
                current_yaw = math.atan2(delta_y, delta_x)
            msg.pose.pose.orientation.z = math.sin(current_yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(current_yaw / 2.0)
        else:
            # Default orientation if no previous data
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = 0.0
            msg.pose.pose.orientation.w = 1.0

        # Update covariance values
        msg.pose.covariance[0] = 0.1  # xx
        msg.pose.covariance[7] = 0.1  # yy
        msg.pose.covariance[14] = 0.1  # zz
        msg.pose.covariance[35] = 3.0  # yaw

        # Log the modified message
        self.get_logger().info(f"Modified Pose: {msg}")

        # Publish the modified message
        self.publisher_.publish(msg)

        # Update previous position
        self.prev_x = current_x
        self.prev_y = current_y
        self.prev_yaw = current_yaw


def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
