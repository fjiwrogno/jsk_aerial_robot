# Copilot Instructions — jsk_aerial_robot

## Project Overview
ROS1 (Melodic/Noetic/ROS-O) catkin workspace for **transformable multi-link aerial robots**. The framework uses a plugin-based architecture where core packages provide abstract base classes and robot-specific packages (`robots/`) register specialized plugins via `pluginlib`.

## Architecture (layered, bottom-up)
```
aerial_robot_msgs          → Custom ROS messages (leaf, no internal deps)
aerial_robot_model         → URDF/KDL kinematics & dynamics, RobotModel base class
aerial_robot_estimation    → Sensor fusion, Kalman filter plugins, SensorBase plugins
aerial_robot_control       → Controllers (PID/LQI/fully-actuated), BaseNavigator, trajectory
aerial_robot_base          → Application node: loads model/navigator/controller via pluginlib
aerial_robot_simulation    → Gazebo & MuJoCo hardware-sim plugins
robots/<name>/             → Robot-specific plugin overrides (model, controller, navigation)
```
Each robot package (e.g., `flamingo`, `hydrus`, `dragon`) **only provides plugin subclasses** — never modifies core code. Plugins are selected at runtime via YAML config and launch params.

## Adding a New Robot or Extending an Existing One
Follow the pattern in `robots/flamingo/` or `robots/hydrus/`. Each robot needs:
1. **Robot model plugin** — subclass `aerial_robot_model::transformable::RobotModel`, override `updateRobotModelImpl()`. Register in `plugins/robot_model_plugins.xml`.
2. **Controller plugin** — subclass `aerial_robot_control::PoseLinearController` (or deeper like `UnderActuatedLQIController`), override `controlCore()` and `sendCmd()`. Register in `plugins/flight_control_plugins.xml`.
3. **Navigation plugin** (optional) — subclass `aerial_robot_navigation::BaseNavigator`, override `initialize()`/`update()`. Register in a navigation plugin XML.
4. **Config YAMLs** — `FlightControl.yaml` (controller plugin name + gains), `NavigationConfig.yaml`, `RobotModel.yaml` (model plugin name).
5. **`bringup.launch`** — loads config YAMLs, sets plugin name params, includes `aerial_robot_base` node and `aerial_robot_model.launch`.
6. **`package.xml`** — `<export>` tags referencing plugin XMLs under the correct interface tag (`<aerial_robot_control>`, `<aerial_robot_model>`).

## Key Virtual Methods to Override
| Base Class | Method | Purpose |
|---|---|---|
| `RobotModel` | `updateRobotModelImpl()` | Custom kinematics (gimbal processing, etc.) |
| `PoseLinearController` | `controlCore()` | Map PID outputs → wrench/thrust allocation |
| `PoseLinearController` | `sendCmd()` | Publish actuator commands to spinal MCU |
| `BaseNavigator` | `initialize()` / `update()` | Custom navigation logic |

## Build & Test
```bash
catkin build                                    # Build all packages
catkin build <package_name>                     # Build single package
# Run all tests (sequential, as CI does):
catkin build --catkin-make-args run_tests -- -i --no-deps --no-status -p 1 -j 1 aerial_robot
catkin_test_results --verbose build
```
Tests are rostest XML files (`.test`) in each robot's `test/` directory. They launch the robot in Gazebo simulation and run Python flight-check scripts (e.g., `hovering_check.py`, `flight_check.py`). Example: `robots/flamingo/test/flight_control.test`.

## Code Formatting
- **C++**: clang-format (Google-based style, 120-col limit). Config in `.clang-format`.
- **Python**: black with `--line-length=120`.
- **Pre-commit**: `pip3 install pre-commit && pre-commit install` to auto-format on commit.
- **C++ standard**: C++17 (`-std=c++17`), optimized with `-O3` in RelWithDebInfo.

## Conventions & Patterns
- **Plugin XML registration**: every plugin needs (1) `add_library()` in CMakeLists.txt, (2) a `plugins/*.xml` mapping `name→type→base_class_type`, (3) `<export>` tag in `package.xml`.
- **Namespace convention**: controllers under `aerial_robot_control::`, models under `aerial_robot_model::`, navigation under `aerial_robot_navigation::`, sensors under `sensor_plugin::`.
- **Config hierarchy**: robot configs live in `robots/<name>/config/`. Simulation-specific overrides use a `simulation/` subfolder (e.g., `FlightControl_sim.yaml`).
- **URDF/Xacro**: robot descriptions in `robots/<name>/robots/` (top-level xacro) and `robots/<name>/urdf/` (component macros + `common.xacro` constants). Custom XML tags like `<baselink>`, `<thrust_link>`, `<m_f_rate>` are parsed by the aerial robot model framework.
- **Thread safety in models**: use `aerial_robot_model::mutex_` when reading/writing shared model state from different threads.
- **Robot-to-robot inheritance**: `dragon` depends on `hydrus` (extends its model). Check `package.xml` dependencies when inheriting across robots.

## NMPC Development (separate branches: `develop/MPC_tilt_mt`, `DEV/nmpc`)
- NMPC is developed on **dedicated branches**, not on the main robot branches. The primary branch is `develop/MPC_tilt_mt`.
- Python model generation scripts use **CasADi + acados**, organized under `aerial_robot_control/scripts/nmpc/` with subdirectories per morphology: `tilt_bi/` (birotor), `tilt_qd/` (quadrotor), `tilt_tri/` (trirotor).
- Generated C code targets `aerial_robot_control/include/aerial_robot_control/nmpc/`. These files are **not git-tracked** — they are auto-generated artifacts that may linger on disk after switching branches. Do not commit them; regenerate via the Python scripts when working on NMPC branches.
- CMake integration for acados-generated code must link against the acados shared libraries.

## Key Files for Orientation
- [aerial_robot_base/src/aerial_robot_base.cpp](aerial_robot_base/src/aerial_robot_base.cpp) — main node, plugin loading
- [aerial_robot_control/include/aerial_robot_control/control/base/pose_linear_controller.h](aerial_robot_control/include/aerial_robot_control/control/base/pose_linear_controller.h) — controller base with `controlCore()`/`sendCmd()` hooks
- [aerial_robot_model/include/aerial_robot_model/model/aerial_robot_model.h](aerial_robot_model/include/aerial_robot_model/model/aerial_robot_model.h) — robot model base
- [aerial_robot_control/include/aerial_robot_control/flight_navigation.h](aerial_robot_control/include/aerial_robot_control/flight_navigation.h) — navigation state machine
- [robots/flamingo/launch/bringup.launch](robots/flamingo/launch/bringup.launch) — canonical launch file pattern
