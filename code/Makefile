# Set this to 0 to build/run on the duckiebot and to 1 to build/run locally
LOCAL=1

DB_NAME = DuckyMcDuckface
DB_NAME_LOCAL = laptop

HOST = $(DB_NAME).local

DATA_DIR = $(CURDIR)/data
LAUNCHFILE_DIR = $(CURDIR)/launchfiles

TAG = daffy

IMG_NAME_CORE = proj-lfi-core
IMAGE_CORE = $(IMG_NAME_CORE):$(TAG)
IMG_NAME_DBINT = proj-lfi-duckiebot-interface
IMAGE_DBINT = $(IMG_NAME_DBINT):$(TAG)
IMG_NAME_ATPOSE = proj-lfi-apriltag_pose
IMAGE_ATPOSE = $(IMG_NAME_ATPOSE):$(TAG)

ifeq ($(LOCAL),0)
$(info Building/running on duckiebot)
ARG_HOST = -H=$(HOST)
ARCH = arm32v7
else
$(info Building/running locally)
ARCH = amd64
endif

ifeq ($(LOCAL),0)
RUN_CMD_CORE = dts duckiebot demo --demo_name proj-lfi --duckiebot_name $(DB_NAME) --package_name duckietown_demos --image $(IMAGE_CORE) --debug
else
RUN_CMD_CORE = docker run -it --rm --privileged --net=host --name $(IMG_NAME_CORE) \
		-v "$(DATA_DIR)":/data \
		-v "$(LAUNCHFILE_DIR)":/launchfiles \
		--env-file local.env \
		-e VEHICLE_NAME=$(DB_NAME_LOCAL) \
		$(IMAGE_CORE) \
		roslaunch /launchfiles/proj-lfi.launch
endif

ifeq ($(LOCAL),0)
RUN_CMD_DBINT = dts duckiebot demo --demo_name proj-lfi --duckiebot_name $(DB_NAME) --package_name duckietown_demos --image $(IMAGE_DBINT) --debug
else
RUN_CMD_DBINT = docker run -it --rm --privileged --net=host --name $(IMG_NAME_DBINT) \
		-v "$(DATA_DIR)":/data \
		-v "$(LAUNCHFILE_DIR)":/launchfiles \
		--env-file local.env \
		-e VEHICLE_NAME=$(DB_NAME_LOCAL) \
		$(IMAGE_DBINT) \
		roslaunch duckiebot_interface all_drivers.launch veh:=$(DB_NAME_LOCAL)
endif

ifeq ($(LOCAL),0)
RUN_CMD_ATPOSE = dts duckiebot demo --demo_name node --duckiebot_name $(DB_NAME) --package_name apriltag_pose --image $(IMAGE_ATPOSE) --debug
else
RUN_CMD_ATPOSE = docker run -it --rm --privileged --net=host --name $(IMG_NAME_ATPOSE) \
		-v "$(DATA_DIR)":/data \
		-v "$(LAUNCHFILE_DIR)":/launchfiles \
		--env-file local.env \
		-e VEHICLE_NAME=$(DB_NAME_LOCAL) \
		$(IMAGE_ATPOSE) \
		roslaunch apriltag_pose node.launch veh:=$(DB_NAME_LOCAL)
endif

.PHONY: core
core:
	docker $(ARG_HOST) build --build-arg ARCH=$(ARCH) -t $(IMAGE_CORE) "$(CURDIR)/$@"

.PHONY: duckiebot-interface
duckiebot-interface:
	docker $(ARG_HOST) build --build-arg ARCH=$(ARCH) -t $(IMAGE_DBINT) "$(CURDIR)/$@"

.PHONY: apriltag_pose
apriltag_pose:
	docker $(ARG_HOST) build --build-arg ARCH=$(ARCH) -t proj-lfi-$@:$(TAG) "$(CURDIR)/$@"

.PHONY: run_core
run_core:
	$(RUN_CMD_CORE)

.PHONY: run_dbint
run_dbint:
	$(RUN_CMD_DBINT)

.PHONY: run_atpose
run_atpose:
	$(RUN_CMD_ATPOSE)


.PHONY: all
all: core duckiebot-interface apriltag_pose

#.PHONY: run
#run:



ifeq ($(LOCAL),0)
TOOLS_CMD = dts start_gui_tools $(DB_NAME)
else
TOOLS_CMD = dts start_gui_tools --sim localhost
endif

.PHONY: tools
tools:
	$(TOOLS_CMD)
