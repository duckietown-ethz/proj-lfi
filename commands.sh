rosparam set /theducknight/kinematics_node/gain 1.5

## START LANE FOLLOWING: -------------------------------------------------------

#First, start all the drivers in dt-duckiebot-interface
dts duckiebot demo --demo_name all_drivers --duckiebot_name theducknight --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy

#Then, start the glue nodes that handle the joystick mapping and the kinematics:
dts duckiebot demo --demo_name all --duckiebot_name theducknight --package_name car_interface --image duckietown/dt-car-interface:daffy

#Finally, we are ready to start the high-level pipeline for lane following:
dts duckiebot demo --demo_name lane_following --duckiebot_name theducknight --package_name duckietown_demos --image duckietown/dt-core:daffy

#run the keyboard controller and press a
dts duckiebot keyboard_control theducknight --base_image duckietown/dt-core:daffy-amd64

#wait
## CALIBRATION:-----------------------------------------------------------------
#get a shell in the bot
dts duckiebot demo --demo_name base --duckiebot_name theducknight --image duckietown/dt-core:daffy

rosparam set /theducknight/kinematics_node/gain 1
rosservice call /theducknight/kinematics_node/save_calibration

## VERBOSE VIDEO OUTPUT---------------------------------------------------------
rosparam set /theducknight/line_detector_node/verbose true

## LOOK AT GRAPHICAL STUFF:-----------------------------------------------------
#starts the gui tools
dts start_gui_tools theducknight --base_image duckietown/dt-core:daffy-amd64

#then in the console you get type:
rqt_image_view
# ------------------------------------------------------------------------------

# START ROS on laptop (with a mounted volume)
docker run -it --rm --net host -v ~/rosbags:/data duckietown/dt-ros-commons:daffy-amd64 /bin/bash

# REMOTE MASTER get a shell on a local container connected to the duckiebots ros master (with a mounted volume)
docker run -it --rm --net host -e ROS_MASTER_URI="http://192.168.1.26:11311/" -v /home/seb/proj-lfi/packages/estimator/config/virtual_lane_node:/traj duckietown/dt-ros-commons:daffy-amd64 /bin/bash

# REMOTE MASTER get a shell on a local container connected to the duckiebot's ros master
IP="109.202.221.23"
docker run -it --rm --net host -e VEHICLE_NAME="theducknight" -e ROS_IP="$IP" -e ROS_MASTER_URI="http://$IP:11311/" duckietown/dt-ros-commons:daffy-amd64 /bin/bash
# things you can do once you have specified a ROS master
rostopic pub -1 /mystuff/mymsgs std_msgs/String -- "hola" #can't use it on mac docker for some reason
rostopic pub -1 /theducknight/wheels_driver_node/wheels_cmd duckietown_msgs/WheelsCmdStamped {} 1.0 1.0 #can't use it on mac docker for some reason
rostopic echo /mystuff/mymsgs

# JOY -> LF
rostopic pub -1 /theducknight/joy_mapper_node/joystick_override duckietown_msgs/BoolStamped {} False
# LF -> INT COORD
rostopic pub -1 /theducknight/stop_line_filter_node/at_stop_line duckietown_msgs/BoolStamped {} True
# INT COORD -> INT CONTR
rostopic pub -1 /theducknight/coordinator_node/intersection_go duckietown_msgs/BoolStamped {} True
# INT_CONTR -> LF
rostopic pub -1 /theducknight/intersection_navigation_node/intersection_done duckietown_msgs/BoolStamped {} True
rostopic pub -1 /theducknight/line_detector_node/switch duckietown_msgs/BoolStamped {} True
line_detector_node/switch
rostopic pub -1 /theducknight/localization_node/reset std_msgs/Bool True

rostopic pub -1 /theducknight/camera_node/loop std_msgs/Bool True
rosparam set /theducknight/localization_node/verbose True
rosparam set /theducknight/virtual_lane_node/trajectory left
# DISABLE CAR CMD (faster than shutting down the car-interface...)
rosservice call /theducknight/car_cmd_switch_node/switch False

rosparam set /theducknight/camera_node/res_h 192 && rosparam set /theducknight/camera_node/res_w 256
rostopic echo --offset /theducknight/lane_controller_node/intersection_navigation_pose/header

/theducknight/lane_controller_node/intersection_navigation_pose

rosparam set /theducknight/camera_node/res_h 192 && rosparam set /theducknight/camera_node/res_w 256
rosparam set /theducknight/kinematics_node/trim .1
rosparam set /theducknight/kinematics_node/gain 1
rosparam set /theducknight/kinematics_node/omega_max 4
rosparam set /theducknight/kinematics_node/v_bar 0.15
rosparam set /theducknight/kinematics_node/v_max 1
rosparam set /theducknight/stop_line_filter_node/off_time 3
rosparam set /theducknight/virtual_lane_node/trajectory left
rosparam set /theducknight/localization_node/integration_enabled 1
rosparam set /theducknight/virtual_lane_node/end_condition_distance 0.5
rosparam set /theducknight/virtual_lane_node/end_condition_angle_deg 20

rosparam set /theducknight/lane_controller_node/use_feedforward_part 1
rosparam set /theducknight/lane_controller_node/omega_min 4
rosparam set /theducknight/lane_controller_node/omega_max 4

rosparam get /theducknight/lane_controller_node

dts duckiebot demo --demo_name all_drivers --duckiebot_name theducknight --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy --debug
dts duckiebot demo --demo_name all --duckiebot_name theducknight --package_name car_interface --image proj-lfi-car-interface:daffy --debug
dts duckiebot demo --demo_name proj-lfi --duckiebot_name theducknight --package_name duckietown_demos --image duckietown/proj-lfi:master-arm32v7 --debug
dts start_gui_tools theducknight --base_image duckietown/dt-core:daffy-amd64

## STRAIGHT:
rosparam get /theducknight/kinematics_node
{baseline: 0.1, gain: 1.0, k: 27.0, limit: 1.0, omega_max: 8.0, radius: 0.0318, trim: 0.1,
  v_bar: 0.15, v_max: 1.0}
rosparam get /theducknight/lane_controller_node
{d_offset: 0.0, d_ref: 0, d_thres: 0.2615, k_Id: 1, k_Iphi: 0.0, k_d: -3.5, k_theta: -1,
  min_rad: 0.06, object_detected: 0, omega_ff: 0, omega_max: 999, omega_min: -999,
  phi_ref: 0, theta_thres: 0.523, use_feedforward_part: 1, use_rad_lim: false, use_radius_limit: true,
  v_bar: 0.23, wheel_distance: 0.103}
rosparam get /theducknight/virtual_lane_node
	{ ...
		end_condition_angle_deg: 30
		end_condition_distance: 0.28
	  ... }

# RIGHT:
rosparam set /theducknight/kinematics_node/omega_max 4.2
rosparam set /theducknight/localization_node/verbose 1

#LEFT
rosparam set /theducknight/virtual_lane_node/end_condition_angle_deg 30
