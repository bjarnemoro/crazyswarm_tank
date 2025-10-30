# Crazyswarm2
A ROS 2-based stack for Bitcraze Crazyflie multirotor robots, with the necessary configurations to make the setup easier here at SML.

The documentation is available here: https://imrclab.github.io/crazyswarm2/. Apart from help in our lab, you can ask for help in their [Discussion](https://github.com/IMRCLab/crazyswarm2/discussions) forum.

## Information


```
ros2 launch crazyflie launch.py backend:=cflib topics.poses.qos.mode:=sensor
ros2 launch crazyflie_examples takeoff_land.launch.py
```
