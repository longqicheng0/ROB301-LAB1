import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from rclpy.parameter import Parameter


class MotorNode(Node):
    """Publishes Twist velocity commands to drive a robot's motors."""

    def __init__(self):
        # --- ROS Parameters ---
        sim_time_param = Parameter('use_sim_time', Parameter.Type.BOOL, False)
        super().__init__("motor_node", parameter_overrides=[sim_time_param])

        # --- Parameters ---
        cmd_vel_topic = "cmd_vel"
        publish_rate_hz = 10.0
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 0.5  # rad/s
        self.forward_duration = 1.0 / self.linear_speed
        self.angular_duration = 360 / (math.degrees(self.angular_speed))

        self.start_pose = np.array([
            [0.0], #x
            [0.0], #y
            [0.0], #theta
        ])

        self.goal_pose = np.array([
            [200/100], #x - m
            [15/100], #y - m
            [math.radians(135)], #theta - radians
        ])

        self.x_travel = self.goal_pose[0,0] - self.start_pose[0,0]
        self.y_travel = self.goal_pose[1,0] - self.start_pose[1,0]

        self.diag_dist_travel = math.sqrt(self.x_travel**2 + self.y_travel**2) #distance travelled
        self.diag_angle_travel = math.atan2(self.y_travel, self.x_travel) #heading angle 
        self.final_angle_adjust = self.goal_pose[2,0] - self.diag_angle_travel #angle after travel

        self.forward_duration = self.diag_dist_travel / self.linear_speed
        self.first_turn_duration = self.diag_angle_travel / self.angular_speed
        self.second_turn_duration = self.final_angle_adjust / self.angular_speed

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)

        # --- State for the example drive pattern ---
        self._state_elapsed = 0.0
        self._period = 1.0 / publish_rate_hz

        # --- Timer drives periodic publishing ---
        self.timer = self.create_timer(self._period, self.timer_callback)

        self.get_logger().info(
            f"motor_node started: publishing to '{cmd_vel_topic}' at {publish_rate_hz} Hz"
        )

    def timer_callback(self):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()

        # Simple two-state example pattern: drive forward, then turn, repeat.
        if self._state_elapsed < self.forward_duration:
            twist_stamped.twist.linear.x = self.linear_speed
            twist_stamped.twist.angular.z = 0.0
        elif self._state_elapsed < self.forward_duration + self.angular_duration:  # "turn"
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = - self.angular_speed
        else:
            self.get_logger().info("Finished Path!")

        self.cmd_vel_pub.publish(twist_stamped)
        self._state_elapsed += self._period


def main():
    rclpy.init()
    node = MotorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
