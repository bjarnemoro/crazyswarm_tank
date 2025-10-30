# Crazyswarm2
A ROS 2-based stack for Bitcraze Crazyflie multirotor robots, with the necessary configurations to make the setup easier here at SML.

The documentation is available here: https://imrclab.github.io/crazyswarm2/. Apart from help in our lab, you can ask for help in their [Discussion](https://github.com/IMRCLab/crazyswarm2/discussions) forum.

## Information
This aim of this project is to provide a plug and play script for a crazyflie test in the KTH water tank.
Its based on the crazyswarm2 project which takes care of the communication with the crazyflies
This project has configured the original crazyswarm project to use our custom marker setup and the optitrack mocap system.

## Run

In order to run the expertiment you first need to connect to the crazyflie:
```
ros2 launch crazyflie launch.py backend:=cflib topics.poses.qos.mode:=sensor

```

Then you can launch the takeoff script as follows:
```
ros2 launch crazyflie_examples takeoff_land.launch.py
```
this script will start a manouver which involves taking off. Then going to a goal position
The goal positions should be published to the '/landing_pad/pose' as a PoseStamped.
Once it has arrived at its goal position the crazyflie will land at the goal

while running the above script you can abort its landing by published to the abort topic as follows:
```
ros2 topic pub /abort_goto std_msgs/msg/Bool "data: true" --once
```
By aborting the crazyflie will return to its starting position and land there
