import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import argparse

from . import config_ur as CONFIG

class URController(Node):
    def __init__(self):
        super().__init__('ur_controller')
        self.get_logger().info("ur controller node started.")

        self.publisher = self.create_publisher(JointTrajectory, CONFIG.TOPIC_CONTROL_COMMANDS, 10)
        self.timer = self.create_timer(CONFIG.COMMAND_PERIOD, self.move)

        self.jointNames = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.goalConfigurations = [
            [0.0, -1.57, -2.2, -0.78, 1.57, 0.0],
            [-3.14, -1.57, -2.2, -0.78, 1.57, 0.0]
            ]
        self.finalJointVelocities = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]

        for i, conf in enumerate(self.goalConfigurations):
            assert (len(self.goalConfigurations[i]) == len(self.jointNames) and
                    len(self.finalJointVelocities[i]) == len(self.jointNames)), "positions and velocities must match joint names' length"
            assert (len(self.goalConfigurations) == len(self.finalJointVelocities)), "positions and velocities must have the same lenght"
        self.counter = 0

    def move(self) -> None:
        self.counter += 1
        trajectoryMessage = JointTrajectory()
        trajectoryMessage.joint_names = self.jointNames

        point = JointTrajectoryPoint()
        point.positions = self.goalConfigurations[self.counter % len(self.goalConfigurations)]
        point.velocities = self.finalJointVelocities[self.counter % len(self.finalJointVelocities)]
        point.time_from_start = Duration(sec=CONFIG.MOTION_DURATION)
        
        trajectoryMessage.points.append(point)
        
        self.get_logger().info(f"publishing goal configuration no. {self.counter}")
        self.publisher.publish(trajectoryMessage)


def main(args=None):
    rclpy.init(args=args)
    urController = URController()
    rclpy.spin(urController)
    urController.destroy_node()
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
        print(f"ur controller main() terminated with exception: {e}")