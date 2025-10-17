from geometry_msgs.msg import Twist

MESSAGE_TYPE = Twist

def adjustSpeedCommand(command, velocityMultiplier):
    command.linear.x *= command.linear.x * velocityMultiplier
    command.angular.z *= command.angular.z * velocityMultiplier
    return command

def generateStopCommand():
    stop = Twist()
    stop.linear.x = 0.0
    stop.linear.y = 0.0
    stop.linear.z = 0.0
    stop.angular.x = 0.0
    stop.angular.y = 0.0
    stop.angular.z = 0.0
    return stop
            