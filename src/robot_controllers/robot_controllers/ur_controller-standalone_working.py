import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import argparse

from . import config_ur as CONFIG

class URController(Node):
    def __init__(self):
        super().__init__('ur_controller')
        self.get_logger().info("ur controller node started.")
        
        self.actionClient = ActionClient(self, FollowJointTrajectory, CONFIG.TOPIC_CONTROL_COMMANDS)

        self.get_logger().info(f"Waiting for action server")
        self.actionClient.wait_for_server()

        # self.publisher = self.create_publisher(JointTrajectory, CONFIG.TOPIC_CONTROL_COMMANDS, 10)
        # self.timer = self.create_timer(CONFIG.COMMAND_PERIOD, self.move)
        
        self.jointNames = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        self.velocity = CONFIG.VELOCITY
        self.angularVelocity = CONFIG.ANGULAR_VELOCITY
        self.activeGoal = False

    def move(self, positions: list, velocities: list) -> None:
        assert (len(positions) == len(self.jointNames) and
                len(velocities) == len(self.jointNames)), "positions and velocities must match joint names length"
        
        self.activeGoal = True
        goalMessage = FollowJointTrajectory.Goal()
        goalMessage.trajectory.joint_names = self.jointNames
        
        trajectoryPoint = JointTrajectoryPoint()
        trajectoryPoint.positions = positions
        trajectoryPoint.velocities = velocities
        trajectoryPoint.time_from_start = Duration(sec=5)
        
        goalMessage.trajectory.points.append(trajectoryPoint)

        self.get_logger().info('Requesting goal from action server...')
        # self.future_goalRequest = self.actionClient.send_goal_async(goalMessage, feedback_callback=self.feedback_callback)
        self.future_goalRequest = self.actionClient.send_goal_async(goalMessage)
        self.future_goalRequest.add_done_callback(self.callback_goalRequestResponse)

    def callback_goalRequestResponse(self, future):
        goalRequestResult = future.result()
        if not goalRequestResult.accepted:
            self.get_logger().info('Goal rejected by server.')
        else:
            self.get_logger().info('Goal accepted by server.')
            self.get_logger().info('Waiting for goal result....')
            self.future_goalResult = goalRequestResult.get_result_async()
            self.future_goalResult.add_done_callback(self.callback_goalResult)

    def callback_goalResult(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal finished with result: {result}')
        self.activeGoal = False

    # def feedback_callback(self, feedback_msg):
    #     # You can process feedback here if needed
    #     pass
    #     ####

# class URController(Node):
#     def __init__(self):
#         super().__init__('ur_controller')
#         self.get_logger().info("ur controller node started.")
        
#         self.jointNames = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
#         self.publisher = self.create_publisher(JointTrajectory, CONFIG.TOPIC_CONTROL_COMMANDS, 10)
#         self.activeGoal = False
#     def move(self, positions: list, velocities: list) -> None:
#         assert (len(positions) == len(self.jointNames) and
#                 len(velocities) == len(self.jointNames)), "positions and velocities must match joint names length"
        
#         trajectoryMessage = JointTrajectory()
#         trajectoryMessage.joint_names = self.jointNames
#         trajectoryPoint = JointTrajectoryPoint()
        
#         trajectoryPoint.positions = positions
#         trajectoryPoint.velocities = velocities
#         trajectoryPoint.time_from_start = Duration(sec=5)

#         trajectoryMessage.points.append(trajectoryPoint)
#         self.publisher.publish(trajectoryMessage)

def main(args=None):
    rclpy.init(args=args)
    urController = URController()
    pointA = [0.0, -1.57, -2.2, -0.78, 1.57, 0.0]
    pointB = [-3.14, -1.57, -2.2, -0.78, 1.57, 0.0]
    finalVelocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    while rclpy.ok():
        urController.move(pointB, finalVelocities)
        urController.get_logger().info("Waiting for active goal to complete...")
        while rclpy.ok() and urController.activeGoal:
            rclpy.spin_once(urController)
        
        urController.move(pointA, finalVelocities)
        urController.get_logger().info("Waiting for active goal to complete...")
        while rclpy.ok() and urController.activeGoal:
            rclpy.spin_once(urController)
    
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