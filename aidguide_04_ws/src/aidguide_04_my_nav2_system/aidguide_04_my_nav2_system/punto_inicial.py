import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class Publisher(Node):

    def __init__(self):
        super().__init__('initial_pose_pub_node')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        timer_period = 0.5  # seconds
        self.timer_ = self.create_timer(timer_period, self.callback)

    def callback(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # ✅ Muy importante
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.2
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.get_logger().info('📍 Publicando posición inicial: X=0.2, Y=0.0, W=1.0')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    try:
        rclpy.spin_once(publisher, timeout_sec=2.0)  # ✅ Evita que se repita infinitamente
    except KeyboardInterrupt:
        publisher.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
