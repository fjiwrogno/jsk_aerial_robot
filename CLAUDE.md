# JSK Aerial Robot - Development Guide

## Project Overview

**JSK Aerial Robot** is a comprehensive ROS1 framework for developing and controlling **transformable multi-link aerial robots** (also called "morphing" or "reconfigurable" drones). The framework supports various robot morphologies including quadrotors, hexrotors, birotor systems, and advanced multi-degree-of-freedom transformable robots.

**Key Robots Supported:**
- **Flamingo** — Aerial-aquatic coaxial bi-rotor (aerial-aquatic hybrid)
- **Hydrus** — Transformable multi-rotor with 2D multilinks (4-6 rotors, configurable geometry)
- **Hydrus-XI** — Extended variant of Hydrus
- **Dragon** — Dual-rotor embedded multi-link robot with multi-DOF aerial transformation
- **Mini Quadrotor** — Simpler quadrotor baseline
- **Gimbalrotor** — Rotor with gimbal mechanism

The framework emphasizes **plugin-based architecture** where core packages provide abstract base classes, and robot-specific packages register implementations via `pluginlib`.

---

## Architecture Overview

### Layered Dependencies (bottom-up)

```
aerial_robot_msgs              → Custom ROS messages (leaf, no internal deps)
    ↓
aerial_robot_model             → URDF/KDL kinematics, dynamics, RobotModel base class
    ↓
aerial_robot_estimation        → Sensor fusion, Kalman filter, StateEstimator
    ↓
aerial_robot_control           → Controllers (PID/LQI/fully-actuated), navigation
    ↓
aerial_robot_base              → Main application node, plugin loader
    ↓
aerial_robot_simulation        → Gazebo & MuJoCo simulation plugins
    ↓
robots/<name>                  → Robot-specific plugin implementations
```

### Core Packages

#### 1. **aerial_robot_msgs**
   - **Location:** `aerial_robot_msgs/`
   - **Purpose:** Custom ROS message definitions
   - **Key Messages:**
     - `State.msg` / `States.msg` — State vector for robot (position, velocity, attitude, angular velocity, acceleration, etc.)
     - `AerialRobotStatus.msg` — Robot control/flight status
     - `FlightNav.msg` — Flight navigation commands
     - `PoseControlPid.msg` — PID controller feedback
     - `Acc.msg` — Acceleration data
     - `WrenchAllocationMatrix.msg` — Wrench allocation for multi-rotor control
   - **Dependencies:** std_msgs, geometry_msgs

#### 2. **aerial_robot_model**
   - **Location:** `aerial_robot_model/`
   - **Purpose:** Robot kinematics, dynamics, and model management via URDF/KDL
   - **Key Features:**
     - Parses URDF descriptions of transformable robots
     - Computes forward kinematics (FK) and Jacobians via KDL
     - Manages rotor configurations (directions, positions, thrust/torque relationships)
     - Provides `RobotModel` base class for robot-specific kinematics overrides
     - Thread-safe access to model state via `mutex_`
   - **Key Classes:**
     - `RobotModel` — Base class; override `updateRobotModelImpl()` for custom kinematics
     - `RobotModelRos` — ROS wrapper that subscribes to joint states
   - **URDF Structure:**
     - Robot descriptions: `robots/<name>/robots/*.xacro` (top-level)
     - Component macros: `robots/<name>/urdf/` (joint definitions, thrust links)
     - Custom tags parsed: `<baselink>`, `<thrust_link>`, `<m_f_rate>` (motor-force rate)
   - **Key Methods:**
     - `updateRobotModel()` — Updates FK/Jacobians based on joint positions
     - `forwardKinematics<T>(link_name)` — Get frame/transform for a link
     - `getRotorNum()`, `getJointNum()` — Query robot configuration
   - **Dependencies:** kdl_parser, eigen_conversions, urdf, tf, pluginlib, spinal (onboard MCU interface)

#### 3. **aerial_robot_estimation**
   - **Location:** `aerial_robot_estimation/`
   - **Purpose:** Multi-sensor fusion for state estimation (odometry, attitude, velocity, etc.)
   - **Key Features:**
     - Kalman filter-based sensor fusion with plugin architecture
     - Supports multiple estimation modes: egomotion (onboard), experiment (mocap), ground truth
     - Plugin system for sensor types: IMU, GPS, Visual Odometry (VO), Altitude, Plane Detection
     - Publishes odometry (`nav_msgs/Odometry`) and TF transforms
   - **Key Classes:**
     - `StateEstimator` — Main fusion engine
     - `SensorBase` — Base class for sensor plugins (IMU, GPS, VO, etc.)
     - Kalman filter implementations in `kf/` subdirectory
   - **Estimation Modes:**
     - Mode 0: `EGOMOTION_ESTIMATE` — Onboard sensors only (IMU, barometer)
     - Mode 1: `EXPERIMENT_ESTIMATE` — For unstable mocap systems
     - Mode 2: `GROUND_TRUTH` — Gazebo ground truth
   - **Key Topics:**
     - Sub: `/imu`, `/altimeter`, `/camera` (sensor data)
     - Pub: `/odometry/filtered`, `/tf` (estimated state + transforms)
   - **Dependencies:** kalman_filter, cv_bridge, geodesy, geographic_msgs, pluginlib, jsk_recognition_msgs

#### 4. **aerial_robot_control**
   - **Location:** `aerial_robot_control/`
   - **Purpose:** Flight control algorithms and feedback laws
   - **Key Features:**
     - Plugin-based controller architecture
     - PID, LQI, and fully-actuated control strategies
     - Trajectory generation (polynomial, sampled trajectories)
     - Dynamic reconfiguration for tuning gains
   - **Key Classes:**
     - `ControlBase` — Abstract base for all controllers
     - `PoseLinearController` — PID-based pose control (position + attitude)
       - Override `controlCore()` to map PID outputs → wrench/thrust allocation
       - Override `sendCmd()` to publish actuator commands to motors/servos
     - `UnderActuatedController` — For under-actuated robots (quads)
     - `UnderActuatedLQIController` — LQI (linear quadratic integral) for under-actuated systems
     - `UnderActuatedTiltedLQIController` — LQI for tilted-rotor systems (e.g., Flamingo)
     - `FullyActuatedController` — For fully-actuated robots (hex with tilted rotors)
   - **Trajectory Module:**
     - Polynomial trajectory generation
     - Sampled trajectory interpolation
     - Reference tracking for trajectory-based control
   - **Key Topics:**
     - Sub: `/odom` (estimated state), `/cmd_nav` (navigation commands)
     - Pub: `/motor_info`, `/servo/cmd` (actuator commands)
   - **Dynamic Reconfiguration:**
     - `PID.cfg` — Tune P, I, D gains for each axis (X, Y, Z, roll, pitch, yaw)
     - `LQI.cfg` — Tune LQI weighting matrices
   - **Dependencies:** aerial_robot_estimation, aerial_robot_model, Eigen3, dynamic_reconfigure, pluginlib, spinal

#### 5. **aerial_robot_base**
   - **Location:** `aerial_robot_base/`
   - **Purpose:** Main application node that orchestrates model/estimation/control/navigation
   - **Key Features:**
     - Loads plugins at runtime via `pluginlib`
     - Separates control loop (high-rate) from callbacks (sensor/nav updates)
     - Two-thread architecture: main loop spinner (control) + callback spinner (4 threads)
   - **Key Class:**
     - `AerialRobotBase` — Singleton that holds robot model, estimator, navigator, controller
   - **Node:** `aerial_robot_base_node` — Entry point
   - **Key Attributes:**
     - `robot_model_ros_` — RobotModel with ROS integration
     - `estimator_` — StateEstimator (sensor fusion)
     - `navigator_` — BaseNavigator (navigation state machine)
     - `controller_` — ControlBase (flight control algorithm)
   - **Main Loop:**
     - Runs at `main_rate` (loaded from param, default ~50 Hz)
     - Calls `mainFunc()` which invokes navigator & controller updates
   - **Initialization Flow:**
     1. Load robot model via `RobotModelRos`
     2. Initialize `StateEstimator` with model
     3. Load navigator plugin (if specified) or use `BaseNavigator`
     4. Load controller plugin (param: `aerial_robot_control_name`, e.g., `"aerial_robot_control/flatness_pid"`)
     5. Start main timer + callback spinners
   - **Dependencies:** aerial_robot_control, aerial_robot_estimation, aerial_robot_model, pluginlib

#### 6. **aerial_robot_simulation**
   - **Location:** `aerial_robot_simulation/`
   - **Purpose:** Simulation-specific plugins for Gazebo and MuJoCo
   - **Key Features:**
     - Gazebo hardware sim plugin (ROS control interface)
     - MuJoCo hardware sim plugin
     - Simulated motor/servo drivers
     - World definitions and URDF variants for simulation
   - **Plugin Files:**
     - `aerial_robot_hw_sim_plugins.xml` — Gazebo plugin registration
     - `flight_controllers_plugins.xml` — ROS control interface plugins
     - `mujoco_robot_hw_sim_plugin.xml` — MuJoCo integration
   - **Key Worlds:** `uuv_gazebo_worlds/` (underwater/aquatic environments)
   - **Dependencies:** gazebo_ros_control, mujoco_ros_control, controller_manager, hector_gazebo_plugins

#### 7. **aerial_robot_nerve**
   - **Location:** `aerial_robot_nerve/`
   - **Purpose:** Onboard microcontroller firmware and communication
   - **Key Features:**
     - STM32 (e.g., Nucleo) firmware for motor/servo control
     - UART/serial communication protocol between main ROS computer and MCU
     - Motor test utilities and calibration
   - **Subdirectories:**
     - `stm32h7_nucleo/` — STM32H7 Nucleo board implementation
     - `spinal/` — Higher-level spinal cord abstraction (ROS messages ↔ UART)
     - `motor_test/` — Motor calibration and testing utilities
   - **Note:** This is MCU firmware, not typically built as part of catkin build (C-only or ARM-specific)

#### 8. **robots/** (Robot-Specific Packages)
   Each robot (flamingo, hydrus, dragon, etc.) is a separate package with:

   - **package.xml** — Declares plugin exports for model/control/navigation
   - **config/** — YAML configuration files
     - `RobotModel.yaml` — Selects robot model plugin class
     - `FlightControl.yaml` — Selects controller plugin + gains (PID, LQI params)
     - `FlightControl_sim.yaml` — Simulation-specific overrides
     - `NavigationConfig.yaml` — Navigation parameters
     - `MotorInfo.yaml`, `Servo.yaml`, `Battery.yaml` — Hardware specs
   - **plugins/** — Plugin registration XMLs
     - `robot_model_plugins.xml` — Maps plugin name → C++ class
     - `control_plugins.xml` (or `flight_control_plugins.xml`) — Controller plugins
     - `navigation_plugin.xml` — Navigation plugins
   - **robots/** — URDF/Xacro descriptions
   - **urdf/** — Macro definitions and common.xacro
   - **launch/bringup.launch** — Main launch file for the robot
   - **src/** — C++ plugin implementations
   - **test/** — Rostest XML files and Python flight-check scripts

   **Example: Flamingo**
   - Model plugin: `FlamingoBirotor` (custom kinematics for coaxial configuration)
   - Controller: `FlamingoPoseLinearController` (tilted LQI for bi-rotor control)
   - Navigation: Custom navigation plugin for water/air transitions
   - Key features: Underwater thrust, aquatic locomotion, aerial transformation

   **Example: Hydrus**
   - Model plugin: `HydrusRobotModel` (multilink kinematics, link transformations)
   - Controller: `HydrusController` (under-actuated LQI)
   - Navigation: Standard navigation
   - Variants: quad (4-rotor), penta (5-rotor), hex (6-rotor)

   **Example: Dragon**
   - Extends Hydrus (depends on hydrus package)
   - Model plugin: `DragonRobotModel` (adds dual-rotor per link)
   - Controller: `DragonController`
   - Navigation: Multi-DOF transformation planner

---

## Key Design Patterns

### 1. Plugin Architecture
- **Base Classes:** `RobotModel`, `ControlBase`, `BaseNavigator`, `SensorBase`
- **Plugin Loading:** `pluginlib::ClassLoader` in `AerialRobotBase`
- **Registration:** Each plugin declares name → class mapping in `plugins/*.xml`
- **Export Tags:** `package.xml` declares which interface each plugin implements via `<export>` tags

**Example Plugin Registration:**
```cpp
// In plugins/robot_model_plugins.xml:
<library path="lib/librobot_model_plugins">
  <class name="flamingo/birotor" type="flamingo::FlamingoBirotor" 
         base_class_type="aerial_robot_model::RobotModel">
    <description>Flamingo bi-rotor model with coaxial configuration</description>
  </class>
</library>
```

### 2. Two-Thread Control Loop
- **Main Loop Thread:** High-rate control (e.g., 50 Hz) via `main_timer_`
  - Calls `navigator_->update()` → `controller_->update()` → `controller_->sendCmd()`
  - Deterministic, separate from ROS callbacks
- **Callback Thread Pool:** 4 threads handle subscribers (joint state, sensor, nav) and service servers
- **Benefit:** Decouples control timing from message latency

### 3. Custom URDF Parsing
- **Standard URDF/Xacro** for robot structure
- **Custom XML tags** parsed by `RobotModel::updateRobotModelImpl()`:
  - `<baselink>` — Root link name
  - `<thrust_link>` — Link where thrust acts
  - `<m_f_rate>` — Motor force rate (thrust per motor command)
  - Example: `<m_f_rate>0.05 0.05 0.04 0.04 0.05 0.05</m_f_rate>` (6 motors)

### 4. Wrench Allocation Matrix
- Maps desired wrench (force + torque) → motor thrust commands
- Computed from kinematics and motor arrangement
- For under-actuated systems (e.g., quadrotor): under-determined; requires control law to select valid wrench
- For fully-actuated (e.g., hex with tilted): inverse of wrench Jacobian

### 5. Modular Estimation with Kalman Filters
- Multiple KF implementations (Extended KF, Unscented KF, etc.)
- Plugins select which sensors to fuse
- Can disable unreliable sensors at runtime (e.g., GPS indoors)

---

## Build System

**Build Tool:** `catkin` (ROS1 build system)

**C++ Standard:** C++17 (`-std=c++17`)

**Compilation Flags:**
- Release: `-O3 -g -DNDEBUG` (RelWithDebInfo)
- Eigen requires optimization for good performance

### Build Commands

```bash
# Build all packages
catkin build

# Build single package
catkin build aerial_robot_control

# Build with verbose output
catkin build --verbose

# Clean build
catkin clean && catkin build

# Build specific robot
catkin build flamingo hydrus dragon
```

### Dependency Management

**Install dependencies (first time):**
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep update --include-eol-distros
rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
```

**.rosinstall files** manage external dependencies:
- `aerial_robot_melodic.rosinstall`, `aerial_robot_noetic.rosinstall`, etc.
- Installs third-party packages: `aerial_robot_3rdparty`, `kalman_filter`, `sensor_interface`, `spinal`

---

## Running and Testing

### Simulation

**Launch robot in Gazebo:**
```bash
# Hydrus (quad variant)
roslaunch hydrus bringup.launch real_machine:=false simulation:=true type:=quad

# Flamingo
roslaunch flamingo bringup.launch real_machine:=false simulation:=true

# Dragon
roslaunch dragon bringup.launch real_machine:=false simulation:=true

# Check model only (no simulation)
roslaunch hydrus bringup.launch real_machine:=false simulation:=false type:=quad
```

**Launch parameters:**
- `real_machine` — Whether to connect to real hardware (default: false)
- `simulation` — Run Gazebo simulator (default: false)
- `headless` — No GUI (default: true)
- `control_mode` — 0=velocity, 1=position, etc. (robot-dependent)
- `estimate_mode` — 0=ego, 1=experiment, 2=ground truth
- `type` — Robot variant (e.g., quad/penta/hex for Hydrus)

### Tests

**Run all tests sequentially (as CI does):**
```bash
catkin build --catkin-make-args run_tests -- -i --no-deps --no-status -p 1 -j 1 aerial_robot
catkin_test_results --verbose build
```

**Test files:**
- Location: `robots/<name>/test/*.test` (rostest XML)
- Test scripts: `robots/<name>/scripts/*_check.py` or `*_demo.py`

**Example test flow:**
1. rostest XML launches robot in Gazebo simulation
2. Python script polls `/odom` or other topics
3. Checks hover stability, transformation, flight control

**Key test examples:**
- `flamingo/test/flight_control.test` — Flamingo hover + control test
- `hydrus/test/hydrus_control.test` — Hydrus multilink hover test
- `dragon/test/dragon_control.test` — Dragon transformation + control test

---

## Code Formatting and Linting

### C++ Style
- **Tool:** `clang-format`
- **Config:** `.clang-format` (Google-based style, 120-column limit)
- **Key settings:**
  - Indent: 2 spaces
  - Pointer binding: right (e.g., `int* x;`)
  - AlwaysBreakTemplateDeclarations: true
  - ColumnLimit: 120

**Format C++ files:**
```bash
clang-format -i src/**/*.cpp include/**/*.h
```

### Python Style
- **Tool:** `black`
- **Config:** `.pre-commit-config.yaml` (line-length: 120)

**Format Python files:**
```bash
black --line-length=120 scripts/*.py
```

### Pre-commit Hook
**Setup (one-time):**
```bash
pip3 install pre-commit
pre-commit install
```

**Manually run:**
```bash
pre-commit run --all-files
```

**Pre-commit checks:**
- YAML, XML, JSON validation
- Trailing whitespace, end-of-file fixes
- Black (Python)
- Clang-format (C++)

---

## Key Files for Orientation

| File | Purpose |
|------|---------|
| `aerial_robot_base/src/aerial_robot_base.cpp` | Main plugin loader; initialize model/estimator/navigator/controller |
| `aerial_robot_model/include/aerial_robot_model/model/aerial_robot_model.h` | Base class; `updateRobotModelImpl()` virtual method |
| `aerial_robot_control/include/aerial_robot_control/control/base/pose_linear_controller.h` | Base controller; override `controlCore()` and `sendCmd()` |
| `aerial_robot_control/include/aerial_robot_control/flight_navigation.h` | Navigation state machine (idle → armed → flying → landed) |
| `aerial_robot_estimation/src/state_estimation.cpp` | Sensor fusion and Kalman filter |
| `robots/flamingo/launch/bringup.launch` | Example launch file structure |
| `robots/flamingo/src/flamingo_birotor.cpp` | Example robot model plugin |
| `robots/flamingo/src/flamingo_controller.cpp` | Example controller plugin |

---

## Extending the Framework: Adding a New Robot

### Step 1: Create Robot Package
```bash
mkdir -p ~/ws/src/robots/my_robot
cd ~/ws/src/robots/my_robot
mkdir config launch src urdf robots plugins test
```

### Step 2: Write package.xml
```xml
<?xml version="1.0"?>
<package>
  <name>my_robot</name>
  <version>1.0.0</version>
  <description>My custom aerial robot</description>
  <maintainer email="me@example.com">My Name</maintainer>
  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>aerial_robot_model</build_depend>
  <build_depend>aerial_robot_control</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>pluginlib</build_depend>

  <run_depend>aerial_robot_base</run_depend>
  <run_depend>aerial_robot_model</run_depend>
  <run_depend>aerial_robot_control</run_depend>
  <run_depend>aerial_robot_simulation</run_depend>

  <test_depend>rostest</test_depend>

  <export>
    <aerial_robot_model plugin="${prefix}/plugins/robot_model_plugins.xml" />
    <aerial_robot_control plugin="${prefix}/plugins/control_plugins.xml" />
  </export>
</package>
```

### Step 3: Write CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_robot)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_model
  aerial_robot_control
  pluginlib
  roscpp)

catkin_package()

# Robot model plugin
add_library(my_robot_model_plugin src/my_robot_model.cpp)
target_link_libraries(my_robot_model_plugin ${catkin_LIBRARIES})

# Controller plugin
add_library(my_robot_controller_plugin src/my_robot_controller.cpp)
target_link_libraries(my_robot_controller_plugin ${catkin_LIBRARIES})

install(TARGETS my_robot_model_plugin my_robot_controller_plugin
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY config launch urdf robots plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

### Step 4: Define Robot Model Plugin
**File: `src/my_robot_model.h`**
```cpp
#pragma once
#include <aerial_robot_model/model/aerial_robot_model.h>

namespace my_robot {
  class MyRobotModel : public aerial_robot_model::RobotModel {
  public:
    MyRobotModel() : RobotModel() {}
    virtual ~MyRobotModel() = default;

  private:
    virtual void updateRobotModelImpl() override {
      // Custom kinematics: e.g., gimbal processing, multilink transformations
      // Call parent's updateRobotModelImpl() first for standard KDL FK
      RobotModel::updateRobotModelImpl();
      // Then apply custom logic
    }
  };
}
```

**File: `plugins/robot_model_plugins.xml`**
```xml
<library path="lib/libmy_robot_model_plugin">
  <class name="my_robot/my_robot_model" type="my_robot::MyRobotModel" 
         base_class_type="aerial_robot_model::RobotModel">
    <description>My custom robot kinematics</description>
  </class>
</library>
```

### Step 5: Define Controller Plugin
**File: `src/my_robot_controller.h`**
```cpp
#pragma once
#include <aerial_robot_control/control/base/pose_linear_controller.h>

namespace my_robot {
  class MyRobotController : public aerial_robot_control::PoseLinearController {
  public:
    MyRobotController() : PoseLinearController() {}
    virtual ~MyRobotController() = default;

  private:
    virtual void controlCore() override {
      // Map PID outputs to wrench (Fx, Fy, Fz, tx, ty, tz)
      // Call computeWrench() to get desired wrench from PID errors
      // Map wrench to motor commands
    }

    virtual void sendCmd() override {
      // Publish motor commands to /motor_info or /servo/cmd
    }
  };
}
```

**File: `plugins/control_plugins.xml`**
```xml
<library path="lib/libmy_robot_controller_plugin">
  <class name="my_robot/my_robot_controller" type="my_robot::MyRobotController" 
         base_class_type="aerial_robot_control::ControlBase">
    <description>My custom controller</description>
  </class>
</library>
```

### Step 6: Create URDF/Xacro
**File: `robots/my_robot.urdf.xacro`** (top-level)
```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include common macros -->
  <xacro:include filename="$(find my_robot)/urdf/common.xacro" />
  
  <!-- Define base link -->
  <link name="base_link" />
  
  <!-- Define rotors -->
  <xacro:rotor_macro name="rotor_0" parent="base_link" xyz="0.1 0 0" />
  
  <!-- Custom tags for this framework -->
  <baselink>base_link</baselink>
  <thrust_link rotor="0">rotor_0_thrust</thrust_link>
  <m_f_rate>0.05 0.05 0.05 0.05</m_f_rate>
</robot>
```

### Step 7: Create Launch File
**File: `launch/bringup.launch`**
```xml
<?xml version="1.0"?>
<launch>
  <arg name="real_machine" default="true" />
  <arg name="simulation" default="false" />
  <arg name="headless" default="true" />
  
  <group ns="my_robot">
    <!-- Load configuration -->
    <rosparam file="$(find my_robot)/config/RobotModel.yaml" command="load" />
    <rosparam file="$(find my_robot)/config/MyRobotControl.yaml" command="load" />
    
    <!-- Set plugin names -->
    <param name="aerial_robot_model_plugin_name" value="my_robot/my_robot_model" />
    <param name="aerial_robot_control_name" value="my_robot/my_robot_controller" />
    
    <!-- Launch main node -->
    <node pkg="aerial_robot_base" type="aerial_robot_base_node" name="aerial_robot_base" 
          output="screen">
      <param name="main_rate" value="50" />
    </node>
  </group>
  
  <!-- Gazebo simulation (optional) -->
  <include file="$(find aerial_robot_simulation)/launch/sim.launch" 
           if="$(arg simulation)">
    <arg name="model_file" value="$(find my_robot)/robots/my_robot.urdf.xacro" />
  </include>
</launch>
```

### Step 8: Write Config YAMLs
**File: `config/RobotModel.yaml`**
```yaml
robot_model:
  plugin_name: my_robot/my_robot_model
  mass: 1.5
  rotor_num: 4
```

**File: `config/MyRobotControl.yaml`**
```yaml
pid:
  x: {p: 5.0, i: 0.1, d: 1.0}
  y: {p: 5.0, i: 0.1, d: 1.0}
  z: {p: 10.0, i: 0.5, d: 2.0}
  roll: {p: 3.0, i: 0.05, d: 0.5}
  pitch: {p: 3.0, i: 0.05, d: 0.5}
  yaw: {p: 2.0, i: 0.02, d: 0.3}
```

### Step 9: Test
```bash
catkin build my_robot
roslaunch my_robot bringup.launch real_machine:=false simulation:=true
```

---

## ROS Nodes and Topics

### Main Node: `aerial_robot_base_node`
**Parameters (read from ROS param server):**
- `~main_rate` (double, Hz) — Main control loop frequency (default: 50)
- `~aerial_robot_model_plugin_name` (string) — Robot model plugin class name
- `~aerial_robot_control_name` (string) — Controller plugin class name (default: "aerial_robot_control/flatness_pid")
- `~flight_navigation_plugin_name` (string) — Navigator plugin class name (optional)
- `~param_verbose` (bool) — Print initialization parameters (default: true)

**Subscribed Topics:**
- `/joint_states` — Joint positions from robot/simulation
- `/imu` — IMU data (estimation)
- `/altimeter` — Altitude data (estimation)
- `/camera/...` — Camera/vision data (estimation)
- `/cmd_nav` — Navigation/goal commands (navigation)
- `/joy` — Joystick input (navigation)

**Published Topics:**
- `/odometry/filtered` — Estimated robot state (nav_msgs/Odometry)
- `/tf` — Transform tree (TF)
- `/motor_info` — Motor commands (aerial_robot_msgs specific)
- `/servo/cmd` — Servo angle commands
- `/aerial_robot_base/control_pid` — PID feedback (aerial_robot_msgs/PoseControlPid)
- `/aerial_robot_base/status` — Robot status (aerial_robot_msgs/AerialRobotStatus)

**Services:**
- `/aerial_robot_base/...` — Various control services (land, takeoff, etc.)

---

## Special Topics and Features

### Sensor Fusion (aerial_robot_estimation)

**Estimation Modes:**
```cpp
enum {
  EGOMOTION_ESTIMATE = 0,      // Onboard sensors only
  EXPERIMENT_ESTIMATE = 1,     // Experiment (mocap) mode
  GROUND_TRUTH = 2             // Gazebo ground truth
};
```

**Sensor Plugins (auto-loaded):**
- `sensor::IMUSensor` — Accelerometer + gyroscope
- `sensor::AltitudeSensor` — Barometric altitude
- `sensor::VisualOdometry` — Camera-based motion estimation
- `sensor::GPSSensor` — GPS (global positioning)
- `sensor::PlaneDetection` — Floor/plane detection for landing

### Control and Wrench Allocation

**Wrench command:** (Fx, Fy, Fz, τx, τy, τz) → motor thrusts

**Thrust allocation:**
```cpp
Eigen::VectorXd motor_commands = wrench_allocation_matrix.inverse() * desired_wrench;
```

For under-actuated systems, the allocation matrix is under-determined; the control law selects a valid solution.

### Navigation State Machine

**States (FlightNav):**
1. **IDLE** — Disarmed, motors off
2. **ARMED** — Motors armed but not flying
3. **FLYING** — Actively controlling
4. **LANDED** — Landed, motors stopped

**Transitions:** Controlled by navigator plugin; typically triggered by RC commands or ROS services.

---

## Debugging and Development Tips

### Enable Verbose Logging
```bash
export ROSCONSOLE_CONFIG_FILE=/path/to/custom_rosconsole.conf
# Configure log levels in the config file
rqt_console  # GUI log viewer
```

### Inspect ROS Graphs
```bash
rqt_graph     # Node/topic dependency graph
rostopic list  # List all topics
rostopic echo /odom  # Print topic data
rosmsg show aerial_robot_msgs/State  # Inspect message structure
```

### Test Model Kinematics
```bash
# Launch model without control
roslaunch <robot>/launch/bringup.launch real_machine:=false simulation:=false

# Publish joint commands to test FK
rostopic pub /joint_states sensor_msgs/JointState "header: {seq: 0, stamp: now, frame_id: 'world'} name: ['joint1'] position: [0.5]"
```

### Inspect Dynamic Reconfigure
```bash
rqt_reconfigure
# Tune PID gains and controller parameters interactively
```

### Record and Replay Rosbag
```bash
rosbag record -a -o my_flight.bag     # Record all topics
rosbag play my_flight.bag --loop       # Replay
rosbag info my_flight.bag              # Inspect contents
```

---

## Known Limitations and Conventions

### Thread Safety
- **Thread-safe access:** `RobotModel` uses `mutex_seg_tf_` for segment transforms
  ```cpp
  std::lock_guard<std::mutex> lock(mutex_seg_tf_);
  auto tf = seg_tf_map_.at(link_name);
  ```
- Always use lock guard when reading/writing shared state from different threads

### NMPC Development
- **NMPC branches:** Developed on `develop/MPC_tilt_mt` and `DEV/nmpc` (not main branch)
- **C code generation:** Python scripts under `aerial_robot_control/scripts/nmpc/` generate C code via **CasADi + acados**
- **Generated code location:** `aerial_robot_control/include/aerial_robot_control/nmpc/`
- **Important:** Generated C files are **not git-tracked**; they are auto-generated. Do not commit them; regenerate when switching NMPC branches.

### Robot-to-Robot Inheritance
- Some robots depend on others (e.g., `dragon` depends on `hydrus`)
  ```cpp
  // dragon/package.xml:
  <build_depend>hydrus</build_depend>  // Extend Hydrus model
  ```
- Check `package.xml` dependencies when adding new robots or modifying base robots

### Configuration Hierarchy
- **Config location:** `robots/<name>/config/`
- **Simulation overrides:** `robots/<name>/config/simulation/` (e.g., `FlightControl_sim.yaml`)
- Load order: default config → simulation override (if sim mode)

---

## References and External Links

- **Wiki:** https://github.com/JSKAerialRobot/aerial_robot/wiki
- **Repository:** https://github.com/JSKAerialRobot/aerial_robot
- **Papers:**
  - Hydrus: Zhao et al., "Transformable multirotor with two-dimensional multilinks," Advanced Robotics, 2016
  - Dragon: Zhao et al., "Design, Modeling, and Control of an Aerial Robot DRAGON," IEEE Robotics and Automation Letters, 2018
  - Flamingo: Custom aerial-aquatic hybrid (see README)

---

## Contact & Maintenance

**Maintainers:**
- Bakui Chou (chou@jsk.imi.i.u-tokyo.ac.jp)
- JSK Lab, University of Tokyo

**License:** BSD

---

**Last Updated:** 2026-04-19
