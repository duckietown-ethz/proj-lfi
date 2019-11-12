#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roslaunch apriltag_pose node.launch veh:=$VEHICLE_NAME
