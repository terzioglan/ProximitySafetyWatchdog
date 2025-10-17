## Description
This package contains two entry points:
1. [`ur_controller`](./robot_controllers/ur_controller.py): Spawns a node which generates and publishes a `JointTrajectory` to one of the two points specified in the [constructor](./robot_controllers/ur_controller.py#L18). 
These trajectories are intended for a UR5 robot, and are periodically published one after another following the specifications in [`config_ur.py`](./robot_controllers/config_ur.py).
The controller attempts to get the robot to the goal configuration from the current robot configuration in 5 seconds.

    Both of the trajectory requests sent by the controller only contains a single joint trajectory point, which may be quite far away from the robot's current configuration.
    Although it is questionable whether it is a good idea to send such a trajectory request, this scenario is implemented as an edge case for the safety gate.
    The controller might intentionally or unintentionally send such a trajectory request, and the safety gate should be able to handle it.
2. [`turtle_controller`](./robot_controllers/turtle_controller.py): Spawns a node which generates and publishes a `Twist` to move a turtle bot on a [circular path](./robot_controllers/turtle_controller.py#L18). 
These messages are generated and published following the specifications in [`config_turtle.py`](./robot_controllers/config_turtle.py).