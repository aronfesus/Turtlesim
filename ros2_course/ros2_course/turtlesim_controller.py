import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')
        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = None

    def pose_callback(self, msg):
        self.pose = msg

    def go_straight(self, speed, distance):
        # Start the straight movement logic here
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.angular.z = 0.0

        # Calculate time to move
        T = distance / speed
        start_time = self.get_clock().now()

        while rclpy.ok():
            current_time = self.get_clock().now()
            if (current_time - start_time).nanoseconds > T * 1e9:
                break

            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self, timeout_sec=0.01)

        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)

    def turn(self, omega, angle):
        # Start the turning logic here
        vel_msg = Twist()
        vel_msg.angular.z = math.radians(omega) if angle > 0 else -math.radians(omega)

        # Calculate time to turn
        T = abs(angle / omega)
        start_time = self.get_clock().now()

        while rclpy.ok():
            current_time = self.get_clock().now()
            if (current_time - start_time).nanoseconds > T * 1e9:
                break

            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self, timeout_sec=0.01)

        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)

    def koch_snowflake(self, speed, omega, size, depth):
        self.turn(omega, 60)
        for _ in range(3):
            self.koch_curve(speed, omega, size, depth)
            self.turn(omega, -120)  # Turning the outside angle of an equilateral triangle

    def koch_curve(self, speed, omega, size, depth):
        if depth == 0:
            self.go_straight(speed, size)
            return
        size /= 3.0
        # Draw the four sides of the Koch curve at this depth
        self.koch_curve(speed, omega, size, depth - 1)
        self.turn(omega, 60)  # Turn to draw the first side of the "triangle"
        self.koch_curve(speed, omega, size, depth - 1)
        self.turn(omega, -120)  # Turn to draw the base of the "triangle"
        self.koch_curve(speed, omega, size, depth - 1)
        self.turn(omega, 60)  # Turn to draw the last side of the "triangle"
        self.koch_curve(speed, omega, size, depth - 1)

# The rest of the class remains unchanged


def main(args=None):
    rclpy.init(args=args)
    turtlesim_controller = TurtlesimController()

    # Draw a Koch snowflake with a given speed, turn speed (omega), initial size and recursion depth
    turtlesim_controller.koch_snowflake(speed=2.0, omega=60, size=4.0, depth=4)

    # Cleanup and shutdown
    turtlesim_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

