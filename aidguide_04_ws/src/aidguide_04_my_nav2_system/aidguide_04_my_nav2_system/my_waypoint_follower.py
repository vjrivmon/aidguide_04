import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped

class WaypointFollowerClient(Node):
    def __init__(self):
        super().__init__('waypoint_follower_client')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.get_logger().info('Waypoint Follower Client initialized')

    def define_waypoints(self):
        """Lista de puntos que ha de recorrer el robot."""
        waypoints = []



        # Waypoint 1
        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.header.stamp = self.get_clock().now().to_msg()
        pose1.pose.position.x = 1.261
        pose1.pose.position.y = -2.611
        pose1.pose.position.z = 0.0
        pose1.pose.orientation.w = 1.0
        waypoints.append(pose1)
        
        
            

        # Waypoint 2
        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.header.stamp = self.get_clock().now().to_msg()
        pose2.pose.position.x = 1.26
        pose2.pose.position.y = -2.60
        pose2.pose.position.z = 0.0
        pose2.pose.orientation.w = 1.0
        waypoints.append(pose2)

        # Waypoint 3
        pose3 = PoseStamped()
        pose3.header.frame_id = 'map'
        pose3.header.stamp = self.get_clock().now().to_msg()
        pose3.pose.position.x = 1.270897
        pose3.pose.position.y = -1.390872
        pose3.pose.orientation.w = 1.0
        waypoints.append(pose3)


        # Waypoint 3-4 intermedio 
        pose4 = PoseStamped()
        pose4.header.frame_id = 'map'
        pose4.header.stamp = self.get_clock().now().to_msg()
        pose4.pose.position.x = 1.508040
        pose4.pose.position.y = -1.390872
        pose4.pose.orientation.w = 1.0
        waypoints.append(pose4)

        # Waypoint 4
        pose4 = PoseStamped()
        pose4.header.frame_id = 'map'
        pose4.header.stamp = self.get_clock().now().to_msg()
        pose4.pose.position.x = 1.928040
        pose4.pose.position.y = -1.398490
        pose4.pose.orientation.w = 1.0
        waypoints.append(pose4)

        # Waypoint 5
        pose5 = PoseStamped()
        pose5.header.frame_id = 'map'
        pose5.header.stamp = self.get_clock().now().to_msg()
        pose5.pose.position.x = 1.967944
        pose5.pose.position.y = 3.029384
        pose5.pose.orientation.w = 1.0
        waypoints.append(pose5)


        return waypoints

    def send_waypoints(self, waypoints):
        """Enviar la lista de puntos al action server."""
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server no disponible')
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info('Enviando waypoints al servidor...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Manejar la respuesta del servidor."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Currently at waypoint: {current_waypoint}')

    def get_result_callback(self, future):
        """Handle the result from the action server."""
        result = future.result().result
        self.get_logger().info('Result received:')
        self.get_logger().info(f'Missed waypoints: {len(result.missed_waypoints)}')
        self.get_logger().info(f'Error code: {result.error_code}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    waypoint_client = WaypointFollowerClient()
    waypoints = waypoint_client.define_waypoints()
    waypoint_client.send_waypoints(waypoints)
    rclpy.spin(waypoint_client)

if __name__ == '__main__':
    main()
