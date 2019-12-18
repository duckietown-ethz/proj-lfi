# Cloning the repository
```
git clone https://github.com/duckietown-ethz/proj-lfi.git
cd proj-lfi
# Fetch submodules
git submodule update --init
```

## Building `proj-lfi`
dts devel build -f --arch arm32v7 -H DUCKIEBOT_NAME.local

## Building `proj-lfi-car-interface`
```
cd car-interface
dts devel build -f --arch arm32v7 -H DUCKIEBOT_NAME.local
```

## Running
```
dts duckiebot demo --demo_name all_drivers --duckiebot_name DUCKIEBOT_NAME --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy
dts duckiebot demo --demo_name all --duckiebot_name DUCKIEBOT_NAME --package_name car_interface --image duckietown/proj-lfi-car-interface:daffy-arm32v7
dts duckiebot demo --demo_name proj-lfi --duckiebot_name DUCKIEBOT_NAME --package_name estimator --image duckietown/proj-lfi:master-arm32v7 --debug
```

## Connect a Keyboard controller
```
dts duckiebot keyboard_control theducknight --base_image duckietown/dt-core:daffy-amd64
```
## Prepare a ROS enabled command line
```
dts start_gui_tools theducknight --base_image duckietown/dt-core:daffy-amd64
```

# Demo Beta
- Update submodules `core` and `car-interface`.
- Change duckiebot name in the Makefile
- `make car-interface`
- `make core`
- `make estimator`
- get `duckiebot interface` running (the only container we haven't touched)
- `make run_carint`
- get a keyboard controller running
- get a gui tools container running and use `rosparam get/set` in it to make sure kinematics calibration is decent (use `gain`=1.0 and only adjust `trim`)
- if you need multiple `rqt` family tools you can get them using this single container. Just run them in the background (eg `rqt_image_view &`)
- set camera resolution:
```
rosparam set /theducknight/camera_node/res_h 192 && rosparam set /theducknight/camera_node/res_w 256
```
- limit velocities
```
rosparam set /theducknight/kinematics_node/omega_max 4
rosparam set /theducknight/kinematics_node/v_bar 0.15
```
- `make run_core`
- Wait longer before re enabling stop line detection
```
rosparam set /theducknight/stop_line_filter_node/off_time 3
```
- `make run_est`
- Put the duckiebot on a lane and press `a` on the keyboard controller
- Robot will lane follow, stop at each stopline for two seconds then attempt to go straight on (the wait happens in the FSMMode callback of localization_node and really should be moved somewhere more appropriate, the wait time is a rosparam)
- Press `s` to stop and then manouver it back into a lane if you need to.

## Changing things
Change the direction it takes
```
rosparam set /theducknight/virtual_lane_node/trajectory left
```

Change **end conditions**:
```
rosparam set /theducknight/virtual_lane_node/end_condition_distance 0.5
rosparam set /theducknight/virtual_lane_node/end_condition_angle_deg 45
```

Lower the rate of change of integrated yaw
```
rosparam set /theducknight/localization_node/integration_enabled 1
```

## Notes
Avoid using verbose

## Measuring latency
it comes out in negative seconds and positive nanosecods so "-1 s 9000000000 ns" is a 0.1 sec delay
```
rostopic echo --offset /theducknight/lane_controller_node/intersection_navigation_pose/header
```

## TODO
- Test on different duckiebots **(WORKS)**
- Are we using the omega feedforward? (I believe yes) **(YES)**
- Figure out why nodes keep using CPU when they are set to inactive. Are they being set to inactive? (maybe fsm callbacks are buggy) **(FIXED)**
- Make a new branch to be handed in with cleaner code and a single container for dt-core and estimator. I was told today that if you extend the dt core base image you can overwrite existing packages by using the same name, which is what they actually want us to do. **(WORKS BUT REPO_NAME HAS TO BE SET TO "dt_core" TO OVERWRITE)**
- Make everything more "duckietown compliant" so TAs like us. I'd avoid the makefile in the hand in. **(DONE, USING DEMO FRAMEWORK)**
- Automate camera resolution change similarly to how we were setting the scale factor in `anti-instagram`
- automatically select **end conditions** based on direction to take:
Distance should be at most .3m for going straight.
Angle should be at most 20° for going left or right.
I've been trying to keep the values large to ensure we release control as soon as possible.
- Do something about the translated trajectory for right turns
- Maybe tune the lane PID? **(IT'S NOT PID, IT's a P and a PI, best thing we can do is set omega_max to 4.2 in kinematics_node)**
- Get some kind of cheap visualization for what is going on. Like verbose but without the image. I know we have some rviz tools but I haven't been using them so I don't know if we can show them in the presentation. **(DONE, RVIZ is not TOO expensive)**
- Find and specify the requirements of our solution and show how we fail is they are not respected. (eg calibration, other red things in sight, wrong resolution) **(PARTLY DONE, calibration is quite irrelevant, trim needs to be decent, duckie beeks are not a problem, we need to have at least one unobstructed stopline, we can tolerate some duckies sitting on them, we are quite robust to start position but there is obviously a limit. One delicate thing is starting far right before a right turn, not turning in place is a tough rule)**
- GET THE OLD INDEFINITE NAVIGATION GOING TO SHOW HOW MUCH BETTER OURS IS!!!! **(DONE, and indeed it is bad, but I didn't really try to calibrate my robot that well)**

## Possible improvements
we can prob mention these in the presentation for next years proj-lfi
- Add proper inertia to wheel cmd integration
- avoid doing homography of full image: eg clustering red line segments and doing a bayes filter like for lane filter
- try different clustering algorithm
- try using the curvature stuff
- make lane following controller better on turns or ditch it for something ad hoc

## Things that failed
- integration to get a pose at higher frequency (failed becuase integration is kinematic)
- propagate expected stopline poses based on integration (inspired to lukas kanade tracking,

# Development
The `code` directory contains all of the project's code.
There is a `Makefile` included to build and run the different docker images.

Currently the following modules are included:
- `core`: a fork of `dt-core`.
- `duckiebot-interface`: a fork of `dt-duckiebot-interface`, but with a camera node that plays a bag-file in a loop.
- `apriltag_pose`: Based on the `core` docker image. It contains a node to develop pose estimation based on detected apriltags.

To run all necessary ROS nodes locally (not on the duckiebot) follow these steps:
1. Build all images: `make all`
2. Run `duckiebot-interface`: `make run_dbint`
3. Run `core`: `make run_core`
4. Run `apriltag_pose`: `make run_atpose`

Additionally the container with the ROS GUI-tools can be started with `make tools`.
Individual images can be built with `make [module]`, e.g. `make apriltag_pose`.

## Playing bagfiles
Bagfiles you want to play need to be copied into `code/data/bags`.
The bagfile that is being played by the modified `camera_node` can be specified during runtime with
```
rosparam set /laptop/camera_node/bag_path /data/bags/[mapfile_name]
```
For example
```
rosparam set /laptop/camera_node/bag_path /data/bags/4-way-straight.bag
```

## Enabling apriltag detection

Currently the apriltag detector node needs to be enabled manually for development, because the finite-state-machine hasn't been adapted to our needs yet.
To do this run
```
rostopic pub -1 /laptop/apriltag_detector_node/switch duckietown_msgs/BoolStamped {} True
```
