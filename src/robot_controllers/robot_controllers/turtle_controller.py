import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import argparse

from . import config_turtle as CONFIG

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.get_logger().info("Turtle bot controller node started.")
        self.publisher = self.create_publisher(Twist, CONFIG.TOPIC_CONTROL_COMMANDS, 10)
        self.timer = self.create_timer(CONFIG.COMMAND_PERIOD, self.move)
        self.velocity = CONFIG.VELOCITY
        self.angularVelocity = CONFIG.ANGULAR_VELOCITY

    def move(self):
        message = Twist()
        message.linear.x = self.velocity
        message.angular.z = self.angularVelocity
        
        self.publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    turtleController = TurtleController()
    rclpy.spin(turtleController)
    turtleController.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument(
        "--default",
        type=str,
        default="default",
        help="default placeholder argument.",
        )
    try:
        main()
    except Exception as e:
        print(f"Turtle controller main() terminated with exception: {e}")