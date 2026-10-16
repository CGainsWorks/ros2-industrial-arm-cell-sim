# Industrial arm cell simulation.
#
# Two classes of target, and the distinction matters when reading the README:
#
#   OFFLINE  need only Python, numpy and scipy. Run anywhere, in minutes, and
#            produce every number the README quotes: grasp success rate, cycle
#            time, calibration residual, probe accuracy, controller tuning,
#            protective stop time.
#              make test  make grasp  make cycle-time  make calibration
#              make inspection  make tuning  make reach  make safety
#
#   SIMULATED  need ROS 2 Humble, MoveIt2 and Gazebo Harmonic, so in practice
#              the Docker image. They produce pictures, not numbers, apart
#              from the launch tests.
#                make demo  make rviz  make moveit  make gazebo  make launch-tests
#
#   ETHERNET/IP  needs Docker: the OpENer adapter and an EIPScanner client.
#                make enip-test
#
# `make check` runs everything in the first class and is what CI does.

SHELL := /bin/bash
PYTHON ?= python3
RESULTS ?= results
PLOTS ?= docs/plots
TRIALS ?= 200
SEED ?= 20260101

.DEFAULT_GOAL := help

# ---------------------------------------------------------------------------
# Offline
# ---------------------------------------------------------------------------

.PHONY: help
help:  ## Show this help
	@grep -hE '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-16s\033[0m %s\n", $$1, $$2}'

.PHONY: test
test:  ## Run the unit tests (no ROS required)
	$(PYTHON) -m pytest tests/ -q

.PHONY: lint
lint:  ## Check formatting and obvious errors
	$(PYTHON) -m flake8 src/ analysis/ tests/ simulation/ scripts/

.PHONY: cell
cell:  ## Regenerate cell.yaml, the URDF and the Gazebo world from one table
	$(PYTHON) simulation/generate_cell.py

.PHONY: grasp
grasp:  ## Grasp success rate and cycle time over $(TRIALS) randomised poses
	$(PYTHON) analysis/grasp_benchmark.py --trials $(TRIALS) --seed $(SEED) \
		--condition all --output $(RESULTS)/grasp_benchmark.json

.PHONY: grasp-quick
grasp-quick:  ## The same, nominal condition only, 40 trials
	$(PYTHON) analysis/grasp_benchmark.py --trials 40 --seed $(SEED) \
		--output $(RESULTS)/grasp_quick.json

.PHONY: cycle-time
cycle-time:  ## Cycle-time breakdown from an existing benchmark run
	$(PYTHON) analysis/cycle_time.py --input $(RESULTS)/grasp_benchmark.json \
		--output $(RESULTS)/cycle_time.json

.PHONY: calibration
calibration:  ## Hand-eye residual, and what the field diagnostics miss
	$(PYTHON) analysis/calibration_residual.py --repeats 25 --seed $(SEED) \
		--output $(RESULTS)/calibration.json

.PHONY: calibration-sensitivity
calibration-sensitivity:  ## Grasp success against injected hand-eye error
	$(PYTHON) analysis/calibration_sensitivity.py --trials 50 --seed $(SEED) \
		--output $(RESULTS)/calibration_sensitivity.json

.PHONY: inspection
inspection:  ## Probe accuracy and seating-detection threshold
	$(PYTHON) analysis/inspection_accuracy.py --repeats 40 --seed $(SEED) \
		--output $(RESULTS)/inspection.json

.PHONY: tuning
tuning:  ## Relay auto-tune, step responses and stability margins, with plots
	$(PYTHON) analysis/pid_tuning.py --output $(RESULTS)/pid_tuning.json --plots $(PLOTS)

.PHONY: reach
reach:  ## Validate the collision scene: reachability over the pick region
	$(PYTHON) analysis/reach_map.py --output $(RESULTS)/reach_map.json

.PHONY: safety
safety:  ## Protective stop on a light curtain break: stop times, detection, ISO 13855 placement
	$(PYTHON) analysis/safety_stop.py --cycles 20 --seed $(SEED) --output $(RESULTS)/safety_stop.json

.PHONY: check
check: lint test reach inspection tuning grasp-quick safety  ## Everything that runs without ROS
	@echo
	@echo "All offline checks passed. Results in $(RESULTS)/."

.PHONY: report
report: grasp cycle-time calibration calibration-sensitivity inspection tuning reach safety  ## Every number in the README
	@echo
	@echo "Full report regenerated in $(RESULTS)/ and $(PLOTS)/."

# ---------------------------------------------------------------------------
# Simulated. These require a sourced ROS 2 Humble workspace.
# ---------------------------------------------------------------------------

.PHONY: build
build:  ## colcon build the workspace
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

.PHONY: demo
demo:  ## Launch the whole cell: description, MoveIt2, fieldbus and the cycle
	ros2 launch cell_bringup cell.launch.py auto_run:=true

.PHONY: rviz
rviz:  ## Just the cell description in RViz, with joint sliders
	ros2 launch cell_description description.launch.py

.PHONY: moveit
moveit:  ## MoveIt2 against the cell's collision scene
	ros2 launch cell_motion moveit.launch.py

.PHONY: control
control:  ## Bring up ros2_control: controller manager, joint states, trajectory controller
	ros2 launch cell_motion control.launch.py

.PHONY: verify-ros
verify-ros:  ## Against a running stack: plan, execute, arrive, and refuse a collision
	$(PYTHON) scripts/verify_ros_stack.py

.PHONY: gazebo
gazebo:  ## The cell in Gazebo Harmonic (headless; add gui:=true for the GUI)
	ros2 launch cell_bringup gazebo.launch.py

.PHONY: launch-tests
launch-tests:  ## Headless launch tests: protective stop on the live arm, the cell in Gazebo
	launch_test src/cell_safety/test/test_protective_stop.launch.py
	launch_test src/cell_bringup/test/test_gazebo_world.launch.py

.PHONY: calibrate
calibrate:  ## Bring up the hand-eye calibration node
	ros2 launch cell_calibration calibration.launch.py

# ---------------------------------------------------------------------------
# Docker
# ---------------------------------------------------------------------------

.PHONY: docker-build
docker-build:  ## Build the container image
	docker compose -f docker/docker-compose.yml build

.PHONY: docker-check
docker-check:  ## Run the offline checks in Docker
	docker compose -f docker/docker-compose.yml run --rm check

.PHONY: docker-demo
docker-demo:  ## Run the ROS demo in Docker
	docker compose -f docker/docker-compose.yml up demo

.PHONY: enip-test
enip-test:  ## EtherNet/IP: build OpENer and EIPScanner, run the adapter against a scanner
	docker compose -f docker/enip-compose.yml up --build --abort-on-container-exit --exit-code-from scanner
	docker compose -f docker/enip-compose.yml down --remove-orphans

.PHONY: clean
clean:  ## Remove build and result artefacts
	rm -rf build install log $(RESULTS) .pytest_cache
	find . -name __pycache__ -type d -prune -exec rm -rf {} +
