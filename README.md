# Cloning the repository
```
git clone https://github.com/duckietown-ethz/proj-lfi.git
cd proj-lfi
# Fetch submodules
git submodule update --init
```
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
rosparam set /laptop/camera_node/bag_path [mapfile_name]
```
For example
```
rosparam set /laptop/camera_node/bag_path 4-way-straight.bag
```

## Enabling apriltag detection

Currently the apriltag detector node needs to be enabled manually for development, because the finite-state-machine hasn't been adapted to our needs yet.
To do this run
```
rostopic pub -1 /laptop/apriltag_detector_node/switch duckietown_msgs/BoolStamped {} True
```
