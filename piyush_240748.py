
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class MySquareTurtle(Node):
    def __init__(self):
        super().__init__('my_square_turtle')

        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

        self.start_x = None
        self.start_y = None
        self.start_theta = None

        self.side = 3.0
        self.speed = 1.0
        self.turn_speed = 1.5

        self.state = 'GOING_FORWARD'
        self.corners_done = 0
        self.target_angle = None

        self.get_logger().info("Hey! Turtle’s ready to draw a square. Let's go!")

    def pose_cb(self, pose):
        if self.start_x is None:
            self.start_x = pose.x
            self.start_y = pose.y
            self.start_theta = pose.theta
            self.target_angle = self.fix_angle(pose.theta + math.pi / 2)
            self.get_logger().info(f"Starting pos: ({self.start_x:.2f}, {self.start_y:.2f})")

        twist = Twist()

        if self.state == 'GOING_FORWARD':
            distance = self.calc_dist(pose.x, pose.y, self.start_x, self.start_y)
            if distance < self.side:
                twist.linear.x = self.speed
                twist.angular.z = 0.0
            else:
                self.get_logger().info(f"Reached corner #{self.corners_done + 1}, time to turn!")
                twist.linear.x = 0.0
                self.state = 'TURNING'
        elif self.state == 'TURNING':
            angle_diff = self.fix_angle(self.target_angle - pose.theta)
            if abs(angle_diff) > 0.05:
                twist.angular.z = self.turn_speed if angle_diff > 0 else -self.turn_speed
                twist.linear.x = 0.0
            else:
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.corners_done += 1
                self.start_x = pose.x
                self.start_y = pose.y
                self.target_angle = self.fix_angle(self.target_angle + math.pi / 2)
                self.get_logger().info(f"Turn done! Corners done: {self.corners_done}")
                if self.corners_done >= 4:
                    self.get_logger().info("Woohoo! Square completed. Stopping now.")
                    rclpy.shutdown()
                else:
                    self.state = 'GOING_FORWARD'

        self.vel_pub.publish(twist)

    def calc_dist(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def fix_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = MySquareTurtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
