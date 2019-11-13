DB_NAME = DuckyMcDuckface

DB_NAME_LOCAL = laptop
HOST_LOCAL = localhost

IMAGE_NAME = proj-lfi-intersection
TAG = daffy
IMAGE = $(IMAGE_NAME):$(TAG)

DATA_DIR = $(CURDIR)/../infrastructure/data
CONTAINER_NAME = $(IMAGE_NAME)
CONTAINER_CMD = roslaunch apriltag_pose node.launch

.PHONY: build
build:
	docker -H=$(HOST) build --build-arg ARCH=arm32v7 -t $(IMAGE) "$(CURDIR)"

.PHONY: buildlocal
buildlocal:
	docker build --build-arg ARCH=amd64 -t $(IMAGE) "$(CURDIR)"

.PHONY: run
run:
	docker -H $(DB_NAME).local run -it --rm --name $(CONTAINER_NAME) --net=host --privileged -v /data:/data $(IMAGE) $(CONTAINER_CMD) veh:=$(DB_NAME)

.PHONY: runlocal
runlocal:
	docker run -it --rm --privileged --net=host --name $(CONTAINER_NAME) \
		-v "$(DATA_DIR)":/data \
		-e ROS_MASTER=$(HOST_LOCAL) \
		-e DUCKIEBOT_NAME=$(HOST_LOCAL) \
		-e ROS_MASTER_URI=http://$(HOST_LOCAL):11311 \
		-e DUCKIEFLEET_ROOT=/data/config \
		-e DUCKIEBOT_IP=$(HOST_LOCAL) \
		-e DUCKIETOWN_SERVER=$(HOST_LOCAL) \
		-e QT_X11_NO_MITSHM \
		-e VEHICLE_NAME=$(DB_NAME_LOCAL) \
		$(IMAGE) \
		$(CONTAINER_CMD) veh:=$(DB_NAME_LOCAL)
