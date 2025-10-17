
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

import argparse, signal, os
from functools import partial

from . import config_turtle as CONFIG_ROBOT
from . import config_gate as CONFIG_GATE

SUBSCRIBER_COUNT_CHECK_PERIOD = 2 # in seconds
def subscriberCountCheck(node: Node, topics: list, signum, frame):
    '''
    Safety-critical topics should not have more than one or zero publishers.
    '''
    for topic in topics: 
        publisherCount = len(node.get_publishers_info_by_topic(topic))
        if publisherCount != 1:
            node.get_logger().warning(f"Unsafe publisher count N={publisherCount} on Topic:{topic}")
    signal.alarm(SUBSCRIBER_COUNT_CHECK_PERIOD) 

class SafetyGate(Node):
    def __init__(self):
        super().__init__('safety_gate_turtle')
        self.get_logger().info("Safety gate node started for the robot.")

        self.declare_parameter('platform', 'turtle')
        robotPlatform = self.get_parameter('platform').get_parameter_value().string_value
        if os.path.isfile("./src/safety_watchdog/safety_watchdog/translators_"+robotPlatform+".py"):
            from . import translators_turtle as translators
        else:
            self.get_logger().error(f"Translators for platform {robotPlatform} are not implemented!")
            rclpy.shutdown()
        
        self.publisher = self.create_publisher(translators.MESSAGE_TYPE, CONFIG_ROBOT.TOPIC_TARGET_CONTROL_COMMANDS, 10)

        self.subscriber_controlCommand = self.create_subscription(translators.MESSAGE_TYPE,CONFIG_ROBOT.TOPIC_SOURCE_CONTROL_COMMANDS,self.callback_controlCommandReceived,10)
        self.subscriber_emergencyStop = self.create_subscription(Bool,CONFIG_GATE.TOPIC_EMERGENCY_STOP_STATUS,self.callback_emergencyStop,10)
        self.subscriber_proximitySensor = self.create_subscription(Float64,CONFIG_GATE.TOPIC_PROXIMITY_STATUS,self.callback_proximitySensor,100)

        self.adjustSpeedCommand = translators.adjustSpeedCommand    # method
        self.stopCommand = translators.generateStopCommand()        # value

        self.robotOpStatus = CONFIG_GATE.RobotOpStatus()
        self.get_logger().info(f"Robot status initialized: Mode:{self.robotOpStatus.mode}, Proximity: {int(self.robotOpStatus.proximity)}.")

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
           
            # slowing down ...
            if "STOP" in possibleSlowerNextStates:
                if distance <= CONFIG_GATE.STATE_TRANSITION_HYSTERESIS_THRESHOLDS[self.robotOpStatus.mode]["SLOWER"]["STOP"]:
                    self.robotOpStatus.mode = "STOP"
                    self.get_logger().info(f'Robot op mode changed to {self.robotOpStatus.mode} based on proximity {int(distance)} mm')
                    self.passAdjustedControllerCommand(self.stopCommand)
                    return 
            if "SLOW" in possibleSlowerNextStates:
                if distance <= CONFIG_GATE.STATE_TRANSITION_HYSTERESIS_THRESHOLDS[self.robotOpStatus.mode]["SLOWER"]["SLOW"]:
                    self.robotOpStatus.mode = "SLOW"
                    self.get_logger().info(f'Robot op mode changed to {self.robotOpStatus.mode} based on proximity {int(distance)} mm')
                    return
            # if "FULL_SPEED" in possibleSlowerNextStates:  # redundant but for completeness
            #     if distance <= CONFIG_GATE.STATE_TRANSITION_HYSTERESIS_THRESHOLDS[self.robotOpStatus.mode]["SLOWER"]["FULL_SPEED"]:
            #         self.robotOpStatus.mode = "FULL_SPEED"
            #         self.get_logger().info(f'Robot op mode changed to {self.robotOpStatus.mode} based on proximity {int(distance)} mm')
            #         return
            
            # ... or speeding up ?
            if "SLOW" in possibleFasterNextStates:
                if distance > CONFIG_GATE.STATE_TRANSITION_HYSTERESIS_THRESHOLDS[self.robotOpStatus.mode]["FASTER"]["SLOW"]:
                    self.robotOpStatus.mode = "SLOW"
                    self.get_logger().info(f'Robot op mode changed to {self.robotOpStatus.mode} based on proximity {int(distance)} mm')
                    return
            if "FULL_SPEED" in possibleFasterNextStates:  # redundant but for completeness
                if distance > CONFIG_GATE.STATE_TRANSITION_HYSTERESIS_THRESHOLDS[self.robotOpStatus.mode]["FASTER"]["FULL_SPEED"]:
                    self.robotOpStatus.mode = "FULL_SPEED"
                    self.get_logger().info(f'Robot op mode changed to {self.robotOpStatus.mode} based on proximity {int(distance)} mm')
                    return

    def callback_emergencyStop(self, incomingMessage):
        if incomingMessage.data:
            self.passAdjustedControllerCommand(self.stopCommand)
            self.robotOpStatus.mode = "EMERGENCY_STOP"
            self.get_logger().info('Big red button pressed! Stopping the robot.')
        else:
            self.robotOpStatus.mode = "STOP" # make sure to start operation with the slowest op mode
            self.get_logger().info(f'Big red button released! Resuming robot operation. Mode: {self.robotOpStatus.mode}')

    def callback_controlCommandReceived(self, incomingCommand):
        if self.robotOpStatus.mode not in ["EMERGENCY_STOP", "STOP"]:
            self.get_logger().info(f"Sending control command. Operation mode: {self.robotOpStatus.mode}, Proximity: {int(self.robotOpStatus.proximity)} mm")
            self.passAdjustedControllerCommand(incomingCommand)

    def passAdjustedControllerCommand(self, command):
        adjustedCommand = self.adjustSpeedCommand(command, CONFIG_GATE.STATUS_VELOCITY_MULTIPLIERS[self.robotOpStatus.mode])
        self.publisher.publish(adjustedCommand)
            
def main(args=None):
    rclpy.init(args=args)
    safetyGate = SafetyGate()
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