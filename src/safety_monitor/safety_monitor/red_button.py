import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import argparse

from . import config as CONFIG 

class BigRedButton(Node):
    def __init__(self):
        super().__init__('big_red_button')
        self.get_logger().info("Big red button node started.")
        self.publisher = self.create_publisher(Bool, CONFIG.TOPIC_EMERGENCY_STOP_STATUS, 10)
        self.buttonStatus = CONFIG.RED_BUTTON_INITIAL_STATE

        if self.buttonStatus == "PRESSED":
            self.pressButton()
        elif self.buttonStatus == "RELEASED":
            self.releaseButton()
        else:
            assert False, "Invalid initial button state in config.py"
    
    def releaseButton(self) -> int:
        message = Bool()
        message.data = False
        try: 
            self.publisher.publish(message)
            self.buttonStatus = "RELEASED"
            self.get_logger().info("Button released.")
            return 0
        except Exception as e:
            self.get_logger().info(f"Failed to release button: {e}")
            return 1
    
    def pressButton(self) -> int:
        message = Bool()
        message.data = True
        try: 
            self.publisher.publish(message)
            self.buttonStatus = "PRESSED"
            self.get_logger().info("Button pressed.")
            return 0
        except Exception as e:
            self.get_logger().info(f"Failed to press button: {e}")
            return 1
    
    def toggleButton(self) -> int:
        if self.buttonStatus == "PRESSED":
            return self.releaseButton()
        else:
            return self.pressButton()
        
    def run(self):
        while rclpy.ok():
            _ = input("Press enter to toggle emergency stop button.")
            self.toggleButton()

    def callback_keyClick(self):
        raise NotImplementedError("This might be a better way. Not yet implemented.")

def main(args=None):
    rclpy.init()
    bigRedButton = BigRedButton()
    bigRedButton.run()
    bigRedButton.destroy_node()
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
        print(f"Emergency stop main() terminated with exception: {e}")