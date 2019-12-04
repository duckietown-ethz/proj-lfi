# Set this to 0 to build/run on the duckiebot and to 1 to build/run locally
LOCAL=0
ACCESSIBLE_LAUNCHFILES=1
DB_NAME = DuckyMcDuckface
DB_NAME_LOCAL = laptop
DB_IP= # if not set makefile will ping the hostname to get the IP
HOST = $(DB_NAME).local

LAUNCHFILE_DIR = $(CURDIR)/launchfiles

TAG = daffy

IMG_NAME_CORE = proj-lfi-core
IMAGE_CORE = $(IMG_NAME_CORE):$(TAG)
IMG_NAME_DBINT = proj-lfi-duckiebot-interface
IMAGE_DBINT = $(IMG_NAME_DBINT):$(TAG)
IMG_NAME_ATPOSE = proj-lfi-apriltag_pose
IMAGE_ATPOSE = $(IMG_NAME_ATPOSE):$(TAG)
IMG_NAME_FSM = proj-lfi-fsm
IMAGE_FSM = $(IMG_NAME_FSM):$(TAG)
IMG_NAME_EST = proj-lfi-estimator
IMAGE_EST = $(IMG_NAME_EST):$(TAG)

ifeq ($(LOCAL),0)
$(info Building/running on duckiebot)
ARG_HOST = -H=$(HOST)
ARCH = arm32v7
DATA_DIR = /data
else
$(info Building/running locally)
ARCH = amd64
DATA_DIR = $(CURDIR)/data
endif

ifeq ($(LOCAL),0)
ifeq ($(ACCESSIBLE_LAUNCHFILES),0)
RUN_CMD_CORE = dts duckiebot demo --demo_name proj-lfi-core \
		--duckiebot_name $(DB_NAME) --package_name duckietown_demos \
		--image $(IMAGE_CORE) --debug
else
ifeq ($(DB_IP),)
$(info Getting Duckiebot IP address)
DB_IP=$(shell ping -c 1 $(HOST) | grep -oP 'PING.*?\(\K[^)]+' )
endif
$(info $(DB_NAME) is at $(DB_IP))

RUN_CMD_CORE = docker $(ARG_HOST) run -it --rm --privileged --net=host \
		--name demo_$(IMG_NAME_CORE) \
		-v "$(DATA_DIR)":/data \
		-e VEHICLE_NAME=$(DB_NAME) \
		-e DUCKIEBOT_NAME=$(DB_NAME) \
		-e ROS_MASTER_URI="http://$(DB_IP):11311" \
		-e DUCKIEFLEET_ROOT="/data/config" \
		-e DUCKIEBOT_IP=$(DB_IP) \
		-e ROS_MASTER=$(DB_IP) \
		-e DUCKIETOWN_SERVER=$(DB_IP) \
		-e QT_X11_NO_MITSHM="1" \
		--memory="800m" \
		--memory-swap="2800m" \
		-d \
		$(IMAGE_CORE) \
		roslaunch /data/launchfiles/proj-lfi-core.launch \
		&& \
		docker $(ARG_HOST) attach demo_proj-lfi-core
endif
else
RUN_CMD_CORE = docker run -it --rm --privileged --net=host --name $(IMG_NAME_CORE) \
		-v "$(DATA_DIR)":/data \
		-v "$(LAUNCHFILE_DIR)":/launchfiles \
		--env-file local.env \
		-e VEHICLE_NAME=$(DB_NAME_LOCAL) \
		$(IMAGE_CORE) \
		roslaunch /launchfiles/proj-lfi-core.launch
endif

ifeq ($(LOCAL),0)
RUN_CMD_DBINT = dts duckiebot demo --demo_name all_drivers --duckiebot_name $(DB_NAME) --package_name duckiebot_interface --image $(IMAGE_DBINT) --debug
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

ifeq ($(LOCAL),0)
RUN_CMD_FSM = dts duckiebot demo --demo_name fsm_node --duckiebot_name $(DB_NAME) --package_name fsm_mod --image $(IMAGE_FSM) --debug
else
RUN_CMD_FSM = docker run -it --rm --privileged --net=host --name $(IMG_NAME_FSM) \
		-v "$(DATA_DIR)":/data \
		-v "$(LAUNCHFILE_DIR)":/launchfiles \
		--env-file local.env \
		-e VEHICLE_NAME=$(DB_NAME_LOCAL) \
		$(IMAGE_FSM) \
		roslaunch /launchfiles/proj-lfi-fsm.launch
endif

ifeq ($(LOCAL),0)
RUN_CMD_EST = dts duckiebot demo --demo_name node --duckiebot_name $(DB_NAME) --package_name estimator --image $(IMAGE_EST) --debug
else
RUN_CMD_EST = docker run -it --rm --privileged --net=host --name $(IMG_NAME_EST) \
		-v "$(DATA_DIR)":/data \
		-v "$(LAUNCHFILE_DIR)":/launchfiles \
		--env-file local.env \
		-e VEHICLE_NAME=$(DB_NAME_LOCAL) \
		$(IMAGE_EST) \
		roslaunch estimator node.launch veh:=$(DB_NAME_LOCAL)
endif

.PHONY: core
core:
	docker $(ARG_HOST) build --build-arg ARCH=$(ARCH) -t $(IMAGE_CORE) "$(CURDIR)/$@"

.PHONY: duckiebot-interface
duckiebot-interface:
	docker $(ARG_HOST) build --build-arg ARCH=$(ARCH) -t $(IMAGE_DBINT) "$(CURDIR)/$@"

.PHONY: apriltag_pose
apriltag_pose: core
	docker $(ARG_HOST) build --build-arg ARCH=$(ARCH) -t proj-lfi-$@:$(TAG) "$(CURDIR)/$@"

.PHONY: fsm
fsm: core
	docker $(ARG_HOST) build --build-arg ARCH=$(ARCH) -t proj-lfi-$@:$(TAG) "$(CURDIR)/$@"

.PHONY: estimator
estimator: core
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

.PHONY: run_fsm
run_fsm:
	$(RUN_CMD_FSM)

.PHONY: run_est
run_est:
	$(RUN_CMD_EST)


.PHONY: all
all: core duckiebot-interface apriltag_pose fsm estimator



ifeq ($(LOCAL),0)
TOOLS_CMD = dts start_gui_tools $(DB_NAME)
else
TOOLS_CMD = dts start_gui_tools --sim localhost
endif

.PHONY: tools
tools:
	$(TOOLS_CMD)
