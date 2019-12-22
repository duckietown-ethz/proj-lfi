# Intersection Navigation
The following are instructions to set up and run the Intersection Navigation Demo.

This demo uses the Intersection Navigation solution proposed by the Fall 2019 ETH AMOD `proj-lfi` group.

## Video of expected results
![](media/all3.gif)

## Cloning the repository
```
git clone https://github.com/duckietown-ethz/proj-lfi.git
cd proj-lfi
# Fetch submodule for slightly modified car-interface
git submodule update --init
```

## Building `proj-lfi`
```
dts devel build -f --arch arm32v7 -H DUCKIEBOT_NAME.local
```
## Building `proj-lfi-car-interface`
```
cd car-interface
dts devel build -f --arch arm32v7 -H DUCKIEBOT_NAME.local
```

## Before running the demo
- Make sure kinematics calibration isn't too bad (up to 0.2m side drift per meter travelled straight in open loop is acceptable)
- Shutdown all `dt-car-interface`, `dt-duckiebot-interface` and `dt-core` containers.
- Ensure lighting is consistent 

## Running the demo
### Start supporting containers
```
dts duckiebot demo --demo_name all_drivers --duckiebot_name DUCKIEBOT_NAME --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy
dts duckiebot demo --demo_name all --duckiebot_name DUCKIEBOT_NAME --package_name car_interface --image duckietown/proj-lfi-car-interface:daffy-arm32v7
```

### Connect a Keyboard controller
```
dts duckiebot keyboard_control DUCKIEBOT_NAME --base_image duckietown/dt-core:daffy-amd64
```
If the robot can be moved around manually the supporting containers have started correctly and you can proceed. 

Leave the Keyboard controller open to start lane following later. 

### Adjust parameters of other nodes 
Fire up a container for communicating with the nodes on the Duckiebot from your computer.
```
dts start_gui_tools DUCKIEBOT_NAME --base_image duckietown/dt-core:daffy-amd64
```
Lower the camera resolution to speed up image processing, localization resolution is still better than what the controller can achieve. In the container:
```
rosparam set /DUCKIEBOT_NAME/camera_node/res_h 192 && rosparam set /theducknight/camera_node/res_w 256
```
Lower the maximum angular velocity command, this prevents the robot from turning too sharply or on the spot. Turning on the spot will cause objects in the robot's view to move too fast, possibly leading to tracking failure and hence incorrect localization.
```
rosparam set /DUCKIEBOT_NAME/kinematics_node/omega_max 4
```
Slow down the robot a little. (The effect of this adjustment depends on `gain` parameter calibration, as different robots move at different velocities for the same motor commands)
```
rosparam set /theducknight/kinematics_node/v_bar 0.15
```

Leave this shell open, you can use it later to change which turn the robot will take and enable additional visualization. 

You can also `rqt_console &` or `rqt_plot &`.

### Run the localization pipeline
```
dts duckiebot demo --demo_name proj-lfi --duckiebot_name DUCKIEBOT_NAME --package_name estimator --image duckietown/proj-lfi:master-arm32v7 --debug
```
All the nodes might take a couple of minutes to start up. When everything is ready `/DUCKIEBOT_NAME/lane_controller_node/intersection_navigation_pose` will be constantly published. You can check it with the `rostopic` command line tool.

### See it in action 
- Place Duckiebot in a lane directed towards a four way intersection.
- Start lane following with the keyboard controller. 

The robot will drive up to the stop line and stop for two seconds. It will then traverse the intersection.
- Stop lane following with the keyboard controller.

#### Change the direction it takes
For the demo, the direction in which the robot will exit the intersection is set via a ROS parameter.
The default is going straight. It can be changed in real time by issuing one of the follwing commands:
```
rosparam set /DUCKIEBOT_NAME/virtual_lane_node/trajectory left
rosparam set /DUCKIEBOT_NAME/virtual_lane_node/trajectory right
rosparam set /DUCKIEBOT_NAME/virtual_lane_node/trajectory straight
```
The intersection navigation system is not informed about the kind of intersection (3-way or 4-way), nor about the direction from which it approaches three way intersections. 

If you do not stop lane following, the demo will continue indefinetely. The robot will however likely leave the road if setup to make an impossible turn.

### Visualization 
Visualization is useful to understand or debug the stopline based localization. 

Enable publication of the stopline detection debug image and all candidate pose estimates:
```
rosparam set /DUCKIEBOT_NAME/localization_node/verbose 1
``` 
Enable visualization of the trajectory in rviz:
```
rosparam set /DUCKIEBOT_NAME/virtual_lane_node/verbose 1
``` 
Enabling verbose on these nodes will also send a lot of additional information to the log and will have a negative performance impact.

We included an rviz configuration file in the `rviz` directory, to use it the duckiebot name has to be updated in the ROS topics.

NOTE: The localization debug image is not published when the node is not active (eg. in lane following mode). For convenience we have set the finite state machine to also activate the node during Keyboard Control.

![](media/still-rviz-gif.gif)


### Adjusting parameters:
Parameters can be adjusted to tune resource usage, localization performance and planning.
#### `virtual_lane_node/`
Name                      | Description | Default | Allowed  
--------------------------|-------------|--------:|---------:
`end_condition_distance`  |             | 0.5     |
`end_condition_angle_deg` |             | 45      |

#### `localization_node/`
Name                      | Description | Default | Allowed  
--------------------------|-------------|--------:|---------:
`integration_enabled`     |             | 1       |

#### `birdseye_node/`
Name                      | Description | Default | Allowed  
--------------------------|-------------|--------:|---------:
`verbose` | Enable additional logging and publishing of rectified images | False | Boolean
`rectify` | Rectification is required for correct ground reprojection, set to False to bypass it. (Untested) | True       | Boolean

Adjusting parameters in `lane_controller_node` and `kinematics_node` can be useful to improve trajectory tracking performance.

## Measuring latency with ROS message timestamps
**Relies on wall clock synchronization**, so it's more reliable if you run it via a shell attached to a container running on the Dubkiebot.
The output is negative seconds and positive nanosecods which may be counter-intuitive. As an example, "-1 s  9000000000 ns" equates to a 0.1 sec delay.
```
rostopic echo --offset /DUCKIEBOT_NAME/lane_controller_node/intersection_navigation_pose/header
```
