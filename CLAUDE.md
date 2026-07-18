# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

jsk_aerial_robot is a ROS1 (catkin) metapackage suite for aerial robots, especially transformable/multilinked ones (hydrus, dragon, etc.). It targets ROS Noetic (Ubuntu 20.04) and ROS-O "one" (Ubuntu 22.04/24.04). The repository lives inside a catkin workspace (`~/Research/jsk_aerial_robot_ws`), with external dependencies pulled in via the `aerial_robot_${ROS_DISTRO}.rosinstall` files.

## Common Commands

All catkin commands run **from** the workspace root (`~/Research/jsk_aerial_robot_ws`), after `source /opt/ros/${ROS_DISTRO}/setup.bash` and `source devel/setup.bash`.

```bash
# Build everything / one package
catkin build
catkin build aerial_robot_control

# Run all tests (mirrors CI; `aerial_robot` is the metapackage that depends on everything)
catkin build --catkin-make-args run_tests -- -i --no-deps --no-status -p 1 -j 1 aerial_robot

# Run a single rostest (each robot package has integration tests in robots/<name>/test/)
rostest mini_quadrotor flight_control.test
rostest hydrus hydrus_control.test
rostest dragon dragon_jacobian.test

# Launch a robot in Gazebo simulation (real_machine=False, simulation=True)
roslaunch mini_quadrotor bringup.launch real_machine:=False simulation:=True headless:=False
# MuJoCo backend instead of Gazebo
roslaunch mini_quadrotor bringup.launch real_machine:=False simulation:=True mujoco:=True
```

Formatting: clang-format (C++, Google-based style, 120 columns, config in `.clang-format`) and black (Python, `--line-length=120`) run via pre-commit. Activate with `pip3 install pre-commit && pre-commit install` in the repo root.

## Architecture

### Plugin-based core loop

The runtime entry point is `aerial_robot_base_node` (`aerial_robot_base/src/aerial_robot_base.cpp`). It composes four subsystems and runs `navigator->update(); controller->update();` in a dedicated main-loop thread at `main_rate` (callbacks run on a separate 4-thread spinner):

1. **Robot model** (`aerial_robot_model`) — computes kinematics/statics from URDF via KDL, publishes TF. Plugins (e.g. `multirotor_robot_model`, `underactuated_tilted_robot_model`) are selected via rosparam.
2. **State estimator** (`aerial_robot_estimation`) — sensor plugins (imu, gps, altitude, mocap, vo, plane_detection) fused via Kalman-filter plugins. Configured in `StateEstimation.yaml`; `estimation/mode` selects EGOMOTION_ESTIMATE(0) / EXPERIMENT_ESTIMATE(1) / GROUND_TRUTH(2).
3. **Navigator** (`aerial_robot_control/src/flight_navigation.cpp`, base class `aerial_robot_navigation::BaseNavigator`) — flight-state machine and waypoint/trajectory handling, selected via `flight_navigation_plugin_name`.
4. **Controller** (`aerial_robot_control`, base class `aerial_robot_control::ControlBase`) — pluginlib plugins (`fully_actuated`, `under_actuated`, LQI variants, NMPC) selected via `aerial_robot_control_name`, normally set in each robot's `FlightControl.yaml`.

All four are loaded by name through pluginlib, so a new robot/controller is added by writing a plugin + yaml config, not by modifying the core loop.

### Robot packages

Each directory under `robots/` (dragon, hydrus, hydrus_xi, gimbalrotor, flamingo, mini_quadrotor) is a standalone ROS package following the same pattern:
- `launch/bringup.launch` — single entry point for both real machine (`real_machine:=True`) and simulation (`simulation:=True`); loads yaml configs from `config/` (RobotModel, MotorInfo, Battery, FlightControl, StateEstimation, NavigationConfig, Simulation) and starts `aerial_robot_base_node`.
- `urdf/` — robot description, from which KDL derives all kinematic/inertial parameters.
- `test/*.test` — rostest integration tests; the common pattern starts the simulation and runs `hovering_check.py` (in `aerial_robot_base`) against waypoints. Robots with closed-loop kinematics also have jacobian tests.
- Robots may extend the core classes with their own model/control/navigation plugins (e.g. dragon, hydrus have their own `src/` and `plugins`).

### Simulation

`aerial_robot_simulation` provides hardware-interface plugins for both Gazebo and MuJoCo (`mujoco:=True`). The simulated "spinal" attitude controller runs inside the sim plugin, mirroring the real firmware behavior, so the same FlightControl.yaml works in sim and on hardware.

### Firmware (aerial_robot_nerve)

`aerial_robot_nerve/` contains STM32 firmware, not catkin-buildable ROS nodes in the usual sense:
- **spinal** — the low-level autopilot (attitude estimation/control) on an STM32 (boards: stm32F7, stm32H7, stm32H7_v2), communicating with ROS via rosserial. The `spinal` catkin package must be built first to generate ROS message libraries; the firmware itself is an STM32CubeIDE project under `spinal/mcu_project/boards/<board>/STM32CubeIDE/` (CI builds it with headless STM32CubeIDE, see `.github/workflows/firmware_build.yml`).
- **neuron** — per-link MCU for multilinked robots, connected to spinal over an internal CAN bus.

Changes to spinal/neuron message definitions affect both firmware and ROS sides.

## CI

GitHub Actions runs: `ros_test.yml` (catkin build + run_tests on noetic/focal and one/jammy/noble via `.travis.sh`), `firmware_build.yml` (spinal H7_v2 firmware), `catkin_lint.yml`, and `python3.yml`. Rostests in CI run with `-p 1 -j 1`.
