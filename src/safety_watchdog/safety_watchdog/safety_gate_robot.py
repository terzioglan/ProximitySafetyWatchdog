import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Bool, Float64
from builtin_interfaces.msg import Duration

import argparse, signal, time
from typing import Optional
from functools import partial

from . import config_robot as CONFIG_ROBOT
from . import config_gate as CONFIG_GATE

SUBSCRIBER_COUNT_CHECK_PERIOD = 2 # in seconds
def subscriberCountCheck(node: Node, topics: list, signum, frame):
    '''
    Safety-critical topics should not have zero or more than one publishers.
    '''
    for topic in topics: 
        publisherCount = len(node.get_publishers_info_by_topic(topic))
        if publisherCount != 1:
            node.get_logger().warning(f"Unsafe publisher count N={publisherCount} on Topic:{topic}")
    signal.alarm(SUBSCRIBER_COUNT_CHECK_PERIOD) 

class TrajectoryStatus(object): # only used by safety_gate_robot
    def __init__(self,):
        self.active = False
        self.onset = None
        self.trajectory = None
        self.velocityMultiplier = None
        self.goalHandle = None
    
    def setTrajectory(self, goalHandle, trajectory, velocityMultiplier, active = True) -> None:
        """
        This is only called when the robot is in an operationally functional state, and the trajectory
        goal is being submitted to the controller.
        """
        self.active = active
        self.onset = time.time()
        self.goalHandle = goalHandle
        self.trajectory = trajectory
        self.velocityMultiplier = velocityMultiplier
    
    def initiateTrajectory(self, trajectory: JointTrajectory, velocityMultiplier: float) -> None:
        """
        This replaces the self.setTrajectory when the robot is not in a functional state, i.e., "STOP", to
        story the latest trajectory request, and execute it later when the "STOP" flag is cleared.
        """
        self.active = True
        self.onset = None
        self.goalHandle = None
        self.trajectory = trajectory
        self.velocityMultiplier = velocityMultiplier
    
    def getRemainingTime(self,) -> Optional[float]:
        """
        Computes the remaining time of the currently executed trajectory segment. This time is used to 
        re-generate the rest of the trajectory in case the operation mode changed mid-trajectory.
        """
        if self.active:
            originalSec = float(self.trajectory.points[-1].time_from_start.sec)
            originalNanosec = float(self.trajectory.points[-1].time_from_start.nanosec)
            originalEndTime = self.onset + originalSec + originalNanosec / 1_000_000_000.0
            remainingTime = originalEndTime - time.time() 
            if remainingTime > 0.0:
                return remainingTime
            else:
                return None  
        else:
            return None
    
    def reComputeTrajectory(self, currentVelocityMultiplier: float) -> JointTrajectory: # not a good way to do this. TODO:
        """
        Using the previous and current velocity multipliers, this function recomputes a trajectory goal,
        simply by recomputing and re-assigning the time to get to the goal based on the current remaining time
        of the trajectory and the ratio of the previous and current velocity multipliers.
        """
        remainingTime = self.getRemainingTime() 
        if remainingTime:
            rescaledRemainingTime = remainingTime * self.velocityMultiplier / currentVelocityMultiplier
            self.velocityMultiplier = currentVelocityMultiplier
            self.trajectory.points[-1].time_from_start.sec = int(rescaledRemainingTime)
            self.trajectory.points[-1].time_from_start.nanosec = int(1_000_000_000.0*rescaledRemainingTime%1) 
        else:
            pass
        return self.trajectory

    def resetTrajectory(self,):
        self.__init__()

class SafetyGateRobot(Node):
    def __init__(self):
        super().__init__('safety_gate_robot')
        self.get_logger().info("Safety gate node started for the robot.")

        self.actionClient = ActionClient(self, FollowJointTrajectory, CONFIG_ROBOT.TOPIC_TARGET_CONTROL_COMMANDS)

        self.get_logger().info(f"Waiting for action server...")
        self.actionClient.wait_for_server()
        self.get_logger().info(f"...connected to action server")

        self.robotOpStatus = CONFIG_GATE.RobotOpStatus()
        self.get_logger().info(f"Robot status initialized: Mode:{self.robotOpStatus.mode}, Proximity: {self.robotOpStatus.proximity}.")
        
        self.trajectoryStatus = TrajectoryStatus()
        self.lastTrajectoryRequest = None

        self.subscriber_controlCommand = self.create_subscription(JointTrajectory,CONFIG_ROBOT.TOPIC_SOURCE_CONTROL_COMMANDS,self.callback_controlCommandReceived,10)            
        self.subscriber_emergencyStop = self.create_subscription(Bool,CONFIG_GATE.TOPIC_EMERGENCY_STOP_STATUS,self.callback_emergencyStop,10)
        self.subscriber_proximitySensor = self.create_subscription(Float64,CONFIG_GATE.TOPIC_PROXIMITY_STATUS,self.callback_proximitySensor,100)
        
        signal.signal(
            signal.SIGALRM, 
            partial(subscriberCountCheck, self, [
                CONFIG_ROBOT.TOPIC_SOURCE_CONTROL_COMMANDS, 
                CONFIG_GATE.TOPIC_EMERGENCY_STOP_STATUS,
                CONFIG_GATE.TOPIC_PROXIMITY_STATUS,
                ]))
        signal.alarm(SUBSCRIBER_COUNT_CHECK_PERIOD)

    def callback_proximitySensor(self, incomingMessage):
        distance = incomingMessage.data
        self.robotOpStatus.proximity = distance
        if self.robotOpStatus.mode != "EMERGENCY_STOP":
            possibleFasterNextStates = CONFIG_GATE.STATE_TRANSITION_HYSTERESIS_THRESHOLDS[self.robotOpStatus.mode]["FASTER"]
            possibleSlowerNextStates = CONFIG_GATE.STATE_TRANSITION_HYSTERESIS_THRESHOLDS[self.robotOpStatus.mode]["SLOWER"]
           
            # are we slowing down ...
            if "STOP" in possibleSlowerNextStates:
                if distance <= CONFIG_GATE.STATE_TRANSITION_HYSTERESIS_THRESHOLDS[self.robotOpStatus.mode]["SLOWER"]["STOP"]:
                    self.stopRobot()
                    self.robotOpStatus.mode = "STOP"
                    self.get_logger().info(f'Robot op mode changed to {self.robotOpStatus.mode} based on proximity {int(distance)} mm')
                    return 
            if "SLOW" in possibleSlowerNextStates:
                if distance <= CONFIG_GATE.STATE_TRANSITION_HYSTERESIS_THRESHOLDS[self.robotOpStatus.mode]["SLOWER"]["SLOW"]:
                    self.robotOpStatus.mode = "SLOW"
                    self.get_logger().info(f'Robot op mode changed to {self.robotOpStatus.mode} based on proximity {int(distance)} mm')
                    if self.trajectoryStatus.active and self.trajectoryStatus.onset != None:
                        self.reSubmitTrajectory()
                    elif self.trajectoryStatus.active and self.trajectoryStatus.onset == None:
                        self.submitTrajectoryRequest(self.trajectoryStatus.trajectory)
                    return
            # if "FULL_SPEED" in possibleSlowerNextStates:  # redundant but for completeness
            #     if distance <= CONFIG.STATE_TRANSITION_HYSTERESIS_THRESHOLDS[self.robotOpStatus.mode]["SLOWER"]["FULL_SPEED"]:
            #         self.robotOpStatus.mode = "FULL_SPEED"
            #         self.get_logger().info(f'Robot op mode changed to {self.robotOpStatus.mode} based on proximity {distance} mm')
            #         return
            
            # ... or speeding up ?
            if "SLOW" in possibleFasterNextStates:
                if distance > CONFIG_GATE.STATE_TRANSITION_HYSTERESIS_THRESHOLDS[self.robotOpStatus.mode]["FASTER"]["SLOW"]:
                    self.robotOpStatus.mode = "SLOW"
                    self.get_logger().info(f'Robot op mode changed to {self.robotOpStatus.mode} based on proximity {int(distance)} mm')
                    if self.trajectoryStatus.active and self.trajectoryStatus.onset != None:
                        self.reSubmitTrajectory()
                    elif self.trajectoryStatus.active and self.trajectoryStatus.onset == None:
                        self.submitTrajectoryRequest(self.trajectoryStatus.trajectory)
                    return
            if "FULL_SPEED" in possibleFasterNextStates: 
                if distance > CONFIG_GATE.STATE_TRANSITION_HYSTERESIS_THRESHOLDS[self.robotOpStatus.mode]["FASTER"]["FULL_SPEED"]:
                    self.robotOpStatus.mode = "FULL_SPEED"
                    self.get_logger().info(f'Robot op mode changed to {self.robotOpStatus.mode} based on proximity {int(distance)} mm')
                    if self.trajectoryStatus.active and self.trajectoryStatus.onset != None:
                        self.reSubmitTrajectory()
                    elif self.trajectoryStatus.active and self.trajectoryStatus.onset == None:
                        self.submitTrajectoryRequest(self.trajectoryStatus.trajectory)
                    return
            
    def callback_emergencyStop(self, incomingMessage):
        if incomingMessage.data:
            self.stopRobot()
            self.robotOpStatus.mode = "EMERGENCY_STOP"
            self.get_logger().info('Big red button pressed! Stopping the robot.')
        else:
            self.robotOpStatus.mode = "STOP" # make sure to start operation with the slowest op mode
            self.get_logger().info('Big red button released! Resuming robot operation.')

    def callback_controlCommandReceived(self, incomingCommand):
        self.get_logger().info('Received control command.')
        if self.trajectoryStatus.active:
            self.stopRobot()
        if self.robotOpStatus.mode not in ["EMERGENCY_STOP", "STOP"]:
            self.get_logger().info(f"Sending control command. Operation mode: {self.robotOpStatus.mode}")
            self.submitTrajectoryRequest(incomingCommand)
        if self.robotOpStatus.mode == "STOP":
            self.trajectoryStatus.initiateTrajectory(
                trajectory=incomingCommand,
                velocityMultiplier = CONFIG_GATE.STATUS_VELOCITY_MULTIPLIERS[self.robotOpStatus.mode],
                )

    def submitTrajectoryRequest(self, jointTrajectory: JointTrajectory):
        adjustedTrajectory = self.adjustTrajectoryVelocity(jointTrajectory)
        
        goalMessage = FollowJointTrajectory.Goal()
        goalMessage.trajectory.joint_names = adjustedTrajectory.joint_names
        
        for trajectoryPoint in adjustedTrajectory.points:
            goalMessage.trajectory.points.append(trajectoryPoint)

        self.get_logger().info('Requesting goal from action server...')
        self.lastTrajectoryRequest = adjustedTrajectory
        self.future_goalRequest = self.actionClient.send_goal_async(goalMessage)
        self.future_goalRequest.add_done_callback(self.callback_goalRequestResult)
    
    def adjustTrajectoryVelocity(self, jointTrajectory: JointTrajectory) -> JointTrajectory:
        adjustedTrajectory = JointTrajectory()
        adjustedTrajectory.joint_names = jointTrajectory.joint_names
        for point in jointTrajectory.points:
            originalDuration = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1_000_000_000.0
            scaledDuration = originalDuration / CONFIG_GATE.STATUS_VELOCITY_MULTIPLIERS[self.robotOpStatus.mode]
            point.time_from_start = Duration(sec=int(scaledDuration), nanosec=int(1_000_000_000.0*scaledDuration%1))
            adjustedTrajectory.points.append(point)
        return adjustedTrajectory

    def stopRobot(self,) -> None:
        self.get_logger().info('Sending stop command.')
        if self.trajectoryStatus.active:
            if self.trajectoryStatus.goalHandle is not None:
                self.trajectoryStatus.goalHandle.cancel_goal_async()    # can handle responses in callbacks TODO:
            self.trajectoryStatus.resetTrajectory()

    def callback_goalRequestResult(self, future):
        goalRequestResult = future.result()
        if not goalRequestResult.accepted:
            self.get_logger().info('Goal rejected by server.')
        else:
            self.get_logger().info('Goal accepted by server.')
            self.get_logger().info('Waiting for goal result....')
            self.trajectoryStatus.setTrajectory(
                goalHandle = goalRequestResult,
                trajectory = self.lastTrajectoryRequest,
                velocityMultiplier = CONFIG_GATE.STATUS_VELOCITY_MULTIPLIERS[self.robotOpStatus.mode]
                )
            self.future_goalResult = goalRequestResult.get_result_async()
            self.future_goalResult.add_done_callback(self.callback_goalResult)

    def callback_goalResult(self, future):
        result = future.result().result
        # self.get_logger().info(f'Goal finished with result: {result}')
        self.get_logger().info(f'Goal result received.')
        self.trajectoryStatus.resetTrajectory()
    
    def reSubmitTrajectory(self,) -> None:
        # self.get_logger().info("Recomputing trajectory.")
        reComputedTrajectory = self.trajectoryStatus.reComputeTrajectory(CONFIG_GATE.STATUS_VELOCITY_MULTIPLIERS[self.robotOpStatus.mode])
        self.stopRobot()
        self.submitTrajectoryRequest(reComputedTrajectory)


def main(args=None):
    rclpy.init(args=args)
    safetyGate = SafetyGateRobot()
    rclpy.spin(safetyGate)
    safetyGate.destroy_node()
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
        print(f"Safety gate main() terminated with exception: {e}")