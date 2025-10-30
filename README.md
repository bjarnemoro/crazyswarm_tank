# Crazyswarm2
A ROS 2-based stack for Bitcraze Crazyflie multirotor robots, with the necessary configurations to make the setup easier here at SML.

The documentation is available here: https://imrclab.github.io/crazyswarm2/. Apart from help in our lab, you can ask for help in their [Discussion](https://github.com/IMRCLab/crazyswarm2/discussions) forum.



## Hardware setup

make sure you have the following hardware available 
1. [Mocap Optitrack](https://optitrack.com/software/motive/) system version Motive 2.0 (this is the system that this repo was tested for).
2. A crazyflie drone.
3. A crazyradio to connect to the drone.

In order to run the experiments, make sure that your computer is connected to the same network as the computer that runs the mocap system and that it is streaming the position of all the markers seens from the camera. 

Also make sure that you have the available `uri` address of the crazyfly drone. you can use the crafyflie `cfclient` to connect to the drone with your radio and check it if you don't know which IP the drone is using.

#### Install the crazyfly client
To set up your drone it is good that you install the crazyfly `client` according to the [following instructions](https://www.bitcraze.io/documentation/system/client-and-library/) and in specific you can find the installation [here](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/)

You can use the client to quickly connect to a drone using your crazyradio. The **important** part is that once you connect to the drone you set up an `uri` address for the drone because this is what it is going to be used by the crazyswarm library to talk to the drone via your radio.

To do so launch the client with 
```
cfclient
```
and then on the GUI that appears start scanning for available drones using the button `scan`. once you are connected to the drone you can check the `uri` of the drone by clicking on the tab `connect/configure2.x`. You will find a tab with the address of the drone with something like : `/0/80/2M/E7E7E7E71`. It is common to just change the last number to refer to different drones (e.g. ...71 is the first drone, ...72 is the second drone, and so on)


## Software installation instructions
This aim of this project is to provide a plug and play script for a crazyflie test in the KTH water tank. Its based on the crazyswarm2 project which takes care of the communication with the crazyflies
This project has configured the original crazyswarm project to use our custom marker setup and the optitrack mocap system.

In order to complete the experiment, make sure you create a new ros2 workspace and install the following dependancies:

### Create workspace

```
mkdir -p tank_experiment_ws/src
cd tank_experiment_ws/src
```

---
*Optional*: we recommend also create a python `virtual` evironment in your folder to avoid issues with other packages installtions you might have installed globally on your computer. You can use the package manager `uv` python for that (see [here](https://docs.astral.sh/uv/getting-started/installation/) )

Create virtual environment

```
uv venv
```

install packages using `pip` after activation of the environment 

```
source .venv/bin/activate
uv pip install your_package
```
---


#### Install dependencies
Inside the `\src` folder install the following dependencies :
1.  Modified crazyswarm2 package : https://github.com/bjarnemoro/crazyswarm_tank/tree/main
2.  Optitrack mocap drivers      : https://github.com/smarc-project/mocap_optitrack

Please be aware that some python packages might be needed in order run the experiment. please download then as needed if errors arise.

once you have installed the dependencies you can run the build command from the folder `tank_experiment_ws`


## Running the experiments


### Setting up the mocap
The creazyflies do not need a specific topic from the mocap in order to fly. They just need the mocap to send the whole point cloude of markers visible from the mocap. The way the position of the crazyfly is tracked is by minimum distance algorithm. Namely, the crazyfly library requires you to set up the crazyflie position at the beginning of the mission, which you should always do according to the file below.

---
Inside `crazyswarm2/crazyfly/config/crazyfies.yaml` set the initial position of your crazyflie as
```
  cf06: # das hatte heute schon funktioniert
    # enabled: true
    enabled: true
    uri: radio://0/80/2M/E7E7E7E706
    initial_position: [ -1.3, 0.1, 0.1]
    type: cf21
```
and make sure that the uri is the correct one. For all the drones that you don't use just mark `enabled:false`. Moreover, you should use the cf21 type which corresponds to the settings at the end of the crafylie.yaml 

```

robot_types:
  cf21:
    motion_capture:
      # tracking: "vendor" # one of "vendor", "librigidbodytracker"
      tracking: "librigidbodytracker" # one of "vendor", "librigidbodytracker"
      # only if enabled; see motion_capture.yaml
      # marker: default_single_marker
      marker: arrangement_v3
      dynamics: default
    big_quad: false
    battery:
      voltage_warning: 3.8  # V
      voltage_critical: 3.7 # V
    # firmware_params:
    #   kalman:
    #     pNAcc_xy: 1.0 # default 0.5
    firmware_logging:
      enabled: true
      # default_topics:
      #   pose:
      #     frequency: 1 # Hz
      custom_topics:
        low_level_control: 
          frequency: 50
          vars: ["ctrlMel.cmd_thrust", "ctrlMel.cmd_roll", "ctrlMel.cmd_pitch", "ctrlMel.cmd_yaw"]
        # your_topic_name:
          # frequency: 10
          # vars: ["stateEstimateZ.x", "stateEstimateZ.y", "stateEstimateZ.z", "acc.x", "acc.y", "acc.z"]
          ## vars: ["acc.x", "acc.y", "acc.z"]

```

you should not change anything of this other than the marker arrangment name which will be specififed in the `motion_capture.yaml` file. 

--- 

Inside the file `crazyswarm2/crazyfly/config/motion_capture.yaml` make sure that the mocap and the hostname are setting correctly. For the hostname you should check the ip address on the newtrok of your mocap system to connect to it.

```
ros__parameters:
    # one of "optitrack", "optitrack_closed_source", "vicon", "qualisys", "nokov", "vrpn", "motionanalysis"
    type: "optitrack" #"qualisys"
    # Specify the hostname or IP of the computer running the motion capture software
    hostname: "10.0.0.96"
```

as a last step you should select the marker arrangment that you would like to adopt for your drone. This is done again the `motion_capture.yaml` where you can create different marker arrangements. The one given for the drones is:

```
arrangement_v3:
    offset: [0.0, 0.0, 0.0]
    points:
        p0: [ 0.000, 0.000, 0.006]
        p1: [ 0.017,-0.028,-0.008]
        p2: [-0.009, 0.025,-0.014]
        # p0: [ 0.022,-0.030,-0.010]
        # p0: [-0.010, 0.027,-0.009]
```
You can change to have the arrangment of the markers that you wish, which respect to the center of the crazyflie.


#### Setting up the mocap topic.
Inside the mocap you need to create two rigid bodies. One for the crazyflie and one for the landing pad. We will use these topics to read the position of the crazyflie relative to the landing pad. Note that this topic is not the one used by the crazyswarm library to get the position of the crazyflie as this is handled independently from the markers point cloud.

Inside the folder `src/mocap_optitrack/config` make sure that that the mocap file looks as following 


```
#
# Definition of all trackable objects
# Identifier corresponds to Trackable ID set in Tracking Tools
#
mocap_node:
    ros__parameters:
        rigid_bodies:
            # 1:
            #     pose: "Robot_1/pose"
            #     pose2d: "Robot_1/ground_pose"
            #     odom: Robot_1/Odom
            #     tf: tf
            #     child_frame_id: "Robot_1/base_link"
            #     parent_frame_id: "world"
            # 2:
            #     pose: "Robot_2/pose"
            #     pose2d: "Robot_2/ground_pose"
            #     odom: Robot_2/Odom
            #     tf: tf
            #     child_frame_id: "Robot_2/base_link"
            #     parent_frame_id: "world"
            5:
                pose: "landing_pad/pose"
                # pose2d: "Robot_2/ground_pose"
                # odom: Robot_2/Odom
                tf: tf
                child_frame_id: "landing_pad/base_link"
                parent_frame_id: "world"
            
            6:
                pose: "crazyfly/pose"
                # pose2d: "Robot_2/ground_pose"
                # odom: Robot_2/Odom
                tf: tf
                child_frame_id: "crazyfly/base_link"
                parent_frame_id: "world"
            
        optitrack_config:
                multicast_address: "239.255.42.99"
                command_port: 1510
                data_port: 1511
                enable_optitrack: true
```

If your file is correct, then you can start the mocap node and check that you receive the position of the landing pad and the crazflie by running 

```
ros2 launch mocap.launch.py
```



#### Make sure everything is good

If everything is good you should launch first the crazyswarm launch file
```
ros2 launch crazyflie launch.py backend:=cflib topics.poses.qos.mode:=sensor
```
if you see the position of the drone publiched as `cf01/pose` for example, (and it is correct), then you are good!




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
