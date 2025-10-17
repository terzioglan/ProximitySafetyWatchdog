# ProximitySafetyWatchdog
A modular control system that adjusts a robot's speed based on object proximity and responds immediately to an emergency stop signal.

workspace repo is broken down into three packages. 
these packages have these entry points, 
these entry points are these.


# Prerequisites
Ubuntu 22.04.5 LTS
ROS2 Humble Hawksbill https://docs.ros.org/en/humble/

# Installation
sudo apt install ros-humble-desktop
sudo apt-get install ros-${ROS_DISTRO}-ur
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
Check if running:
 sudo systemctl status docker
run if not
 sudo systemctl start docker

add user to docker group
 sudo usermod -aG docker $USER
relog/restart, or run
  newgrp docker

git clone repo
follow instructions on ros2_ur5_interface
run the docker
check the conatiner id
and copy the repo into docker
docker cp <SOURCE DIRECTORY>/ProximitySafetyWatchdog <CONTAINER  ID>:/home/ubuntu

in the docker container
build repo
in ~/ProximitySafetyWatchdog run
colcon build

## For turtlesim
sudo apt install ros-humble-turtlesim - no need

usage
start with ur_controller

however this isn't great because ros2_ur5_interface only supports trajectory goals, and does not launch ur_controllers for trajectory control.
this overloads the utility of the safety gate
a better approach to robot control, if such slowing-down/speeding-up is necessary, without overloading the function and consequently the reusability of the gate is to use a velocity-based controller.
send velocity (or force, or effort) commands to the robot in a closed loop fashion.
this way the translation of the control signals to lower values can be achieved with minimal effort to follow the desired trajectory, and also without overloading the function of the safety gate.
an example of how this can be done is also provided based on the turtlesim, since it was already included in the ros installation on the docker image
to view this example run 
this
this
this 
in ~/ros2_ws run
source install/setup.bash
ros2 launch ros2_ur5_interface sim.launch.py

in the container in a new terminal, navigate to ~/ProximitySafetyWatchdog separate terminals run

source install/setup.bash
ros2 run robot_controllers ur_controller

source install/setup.bash
ros2 run safety_monitor red_button

source install/setup.bash
ros2 run safety_monitor proximity_sensor

source install/setup.bash
ros2 run safety_watchdog safety_gate_robot

initial state is "EMERGENCY_STOP" so go to the red button terminal and release the break

for turtle example 

in ~/ProximitySafetyWatchdog using the same separate terminals run
ros2 run robot_controllers turtle_controller
ros2 run safety_monitor red_button
ros2 run safety_monitor proximity_sensor
ros2 run safety_watchdog safety_gate_turtle


## References & Sources
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/installation.html
https://docs.docker.com/engine/install/ubuntu/
https://github.com/pla10/ros2_ur5_interface


## todo:
- define buttonCallback in red_button.py so that you don't have to make a blocking call for user input and do other stuff.
- define each common property--e.g., source and taget topic names for connected nodes--without requiring repetition.
- check setup.py and package.xml contents for each package.
- remove redundant imports.
- make the main package able to handle any message type that's not rclpy.std_msgs.msg.Twist
- can make config.py parameters available across the ros run. Not sure if that makes sense here though since \(almost\) all config.py parameters are actually private.
- can use interval data structures to check proximity thresholds. But that requires using other packages, e.g., pandas, pyinterval (?)
- can spawn multiple threads to handle several safety_gate callbacks in parallel (e.g. MultiThreadedExecutor). though it doesn't take much time to callback to return so i'm not sure if that's absolutely necessary.
- can check if the robot stopped after the stop message is sent, and keep sending the stop message after you make sure the robot stops.
- might make sense to differentiate the big_red_button shutdown vs. proximity shutdown instead of using a single "STOP" robot state for both. 
- known bug is a feature: if the red_button is ran and it is released, since safety_gate starts with robotOpMode="STOP" as default, you need to press/release the red button to start operation. 
- can publish robotOpStatus on a topic to make it available for other modules to monitor. this topic should be protected and limited to authorized nodes only
- make emergency_stop_status and proximity sensor topics only available to write to their associated nodes.
- get rid of print commands.
- namespaces for packages.
- can put volatile stuff, like hysteresis width, passable by arguments.
- reshape the remainder of the trajectory after and emergency or proximity stop 
- ros2_ur5_interface only spawns a trajectory controller. Such responsivity would be better suited with a velocity controller, I think.
- fork ros2_ur5_interface, and add ur_controller velocity control to the launch files. refactor the safety_gate.
- safety_gate_robot.SafetyGateRobot can be broken down into two classes. One for handling the state machine logic only, the other for managing back/forty message passing.


