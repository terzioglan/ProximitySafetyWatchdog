## Description
This package contains two entry points:
1. [`ur_controller`](./robot_controllers/ur_controller.py): Spawns a node which generates and publishes a `JointTrajectory` to one of the two points specified in the [constructor](./robot_controllers/ur_controller.py#L18). 
These trajectories are intended for a UR5 robot, and are periodically published one after another following the specifications in [`config_ur.py`](./robot_controllers/config_ur.py).
2. [`turtle_controller`](./robot_controllers/turtle_controller.py): Spawns a node which generates and publishes a `Twist` to move a turtle bot on a [circular path](./robot_controllers/turtle_controller.py#L18). 
These messages are generated and published following the specifications in [`config_turtle.py`](./robot_controllers/config_turtle.py).