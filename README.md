# L1AFO-Kinova

Research implementation for the IEEE Transactions on Robotics paper
**“Extreme High-Gain Friction Observer of Flexible Joint Robots With L1 Adaptive Framework”** on a 7-DoF Kinova Gen3.

This repository contains the controllers used in the paper, ROS 2 interfaces, and the Kinova Gen3 models needed for simulation and rigid/flexible-joint dynamics.

> **Safety:** This is research torque-control software. Validate the controller in simulation, check gains and joint conventions for your robot, use the hardware emergency stop, and start with conservative limits before enabling torque control on a real Gen3.

## Repository layout

- `kinova_controller`: controller, real-robot interface, MuJoCo simulation, launch, and configuration
- `robot_model`: Kinova Gen3 MuJoCo and Pinocchio models
- `controller_interface_msgs`: controller command messages used by this project
- `robot_state_msgs`: state and controller-logging messages used by this project

## Supported environment

The original implementation was developed with:

- ROS 2 Foxy and C++17
- Kinova Kortex C++ API 2.3.0
- MuJoCo 2.3.1
- Eigen3, Pinocchio, yaml-cpp, Boost.System, OpenGL/EGL, GLFW3, and ncurses

ROS 2 Foxy is end-of-life and officially targets Ubuntu 20.04 (Focal). The commands below assume a native Ubuntu 20.04 desktop installation. Use an Ubuntu 20.04 container or virtual machine when the host runs a newer Ubuntu release.

## Setup on a clean Ubuntu 20.04 installation

### 1. Install ROS 2 Foxy and build dependencies

Configure the locale and the ROS 2 apt repository:

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ROS 2 and the libraries required by this repository:

```bash
sudo apt update
sudo apt install -y \
  ros-foxy-desktop \
  ros-foxy-pinocchio \
  python3-colcon-common-extensions \
  build-essential \
  cmake \
  git \
  unzip \
  libeigen3-dev \
  libyaml-cpp-dev \
  libboost-system-dev \
  libglfw3-dev \
  libgl1-mesa-dev \
  libegl1-mesa-dev \
  libncurses-dev
```

Source ROS 2 in every new terminal:

```bash
source /opt/ros/foxy/setup.bash
```

### 2. Clone the repository

Clone this repository:

```bash
git clone https://github.com/dbsxogh09/L1AFO-Kinova.git
cd L1AFO-Kinova
```

### 3. Download third-party dependencies

Kortex and MuJoCo binaries are not committed. The following commands install all third-party files under the ignored `third_party` directory.

Download the official Kinova Kortex C++ API 2.3.0 binary distribution for x86-64 Linux:

```bash
mkdir -p third_party/kortex_api
curl -L \
  https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.3.0/linux_x86-64_x86_gcc.zip \
  -o /tmp/kortex-api-2.3.0.zip
unzip -q /tmp/kortex-api-2.3.0.zip -d third_party/kortex_api
```

Download the official MuJoCo 2.3.1 binary distribution:

```bash
curl -L \
  https://github.com/google-deepmind/mujoco/releases/download/2.3.1/mujoco-2.3.1-linux-x86_64.tar.gz \
  -o /tmp/mujoco-2.3.1-linux-x86_64.tar.gz
tar -xzf /tmp/mujoco-2.3.1-linux-x86_64.tar.gz -C third_party
```

The MuJoCo binary archive contains the simulator UI sources but not LodePNG. Download the two LodePNG sources separately; their upstream license notice is embedded in the files:

```bash
mkdir -p third_party/lodepng
curl -L https://raw.githubusercontent.com/lvandeve/lodepng/master/lodepng.h \
  -o third_party/lodepng/lodepng.h
curl -L https://raw.githubusercontent.com/lvandeve/lodepng/master/lodepng.cpp \
  -o third_party/lodepng/lodepng.cpp
```

The resulting layout should be:

```text
third_party/
├── kortex_api/
│   ├── include/client_stubs/BaseClientRpc.h
│   └── lib/release/libKortexApiCpp.a
├── lodepng/
│   ├── lodepng.h
│   └── lodepng.cpp
└── mujoco-2.3.1/
    ├── include/mujoco/mujoco.h
    ├── lib/libmujoco.so.2.3.1
    └── simulate/
        └── simulate.cc
```

Verify the required files before building:

```bash
test -f third_party/kortex_api/include/client_stubs/BaseClientRpc.h
test -f third_party/kortex_api/lib/release/libKortexApiCpp.a
test -f third_party/mujoco-2.3.1/include/mujoco/mujoco.h
test -f third_party/mujoco-2.3.1/simulate/simulate.cc
test -f third_party/lodepng/lodepng.cpp
```

## Build

From the repository root:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The build is successful when all four packages finish. CMake options can be used when dependencies are stored outside the default layout:

```bash
colcon build --symlink-install --cmake-args \
  -DKORTEX_INCLUDE_DIR=/absolute/path/to/kortex_api/include \
  -DKORTEX_LIBRARY=/absolute/path/to/libKortexApiCpp.a \
  -DMUJOCO_PATH=/absolute/path/to/mujoco-2.3.1 \
  -DLODEPNG_PATH=/absolute/path/to/lodepng
```

## Run the simulation

The checked-in configuration starts in simulation with gravity compensation. Model paths are resolved from the installed `robot_model` package and require no user-specific absolute paths.

Start the lifecycle controller and MuJoCo UI:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch kinova_controller run.launch.py
```

The launch file automatically configures and activates `/kinova_controller`. In the MuJoCo window, press `Tab` to toggle the left settings panel and `Shift+Tab` to toggle the right joint/control panel.

In a second terminal, verify the node, lifecycle state, and state stream:

```bash
cd /path/to/L1AFO-Kinova
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 node list
ros2 lifecycle get /kinova_controller
ros2 topic list
ros2 topic hz /extended_robot_state
```

The lifecycle state should be `active`; stop `ros2 topic hz` with `Ctrl+C` after confirming that messages arrive.

## Select and test controllers

> **Simulation only:** The friction controllers execute built-in experiment trajectories immediately after selection. Do not run these commands on real hardware without an independent safety review.

The default YAML selects the implicit L1AFO implementation as the torque interface. Switch from gravity compensation to the joint-space experiment without rebuilding:

```bash
ros2 topic pub --once \
  /controller_config \
  controller_interface_msgs/msg/ControllerConfig \
  "{controller_selector: 'JOINT_PD_FRIC'}"
```

Select the task-space collision-tracking experiment with:

```bash
ros2 topic pub --once \
  /controller_config \
  controller_interface_msgs/msg/ControllerConfig \
  "{controller_selector: 'TASK_PD_FRIC'}"
```

Inspect controller logging signals in another terminal:

```bash
ros2 topic echo /controller_logging
```

Select a friction implementation by editing the applicable `torque_interface` entry in `kinova_controller/config/kinova_controller_config.yaml`:

```yaml
joint_fric_gains:
  torque_interface: "IMPLICIT_L1_FRIC"
```

The paper-related options retained in this repository are `IMPLICIT_L1_FRIC`, `COULOMB_OBSERVER`, `ASYM_FRICTION`, and `STATIC_FRICTION`. The asymmetric model uses direction-dependent velocity, load-torque, and motor-temperature terms. The static model uses Coulomb and saturating velocity terms.

The `asym_friction_gain` and `static_friction_gain` arrays are intentionally empty. These identified or tuned coefficients are robot-specific and can change with the individual actuators, transmission condition, temperature range, payload, and identification trajectory. Perform offline identification and tuning for the target robot, then replace every empty array in the selected model with a finite seven-element vector. For the static model, every `Fv2` velocity scale must also be strictly positive. The controller refuses to initialize `ASYM_FRICTION` or `STATIC_FRICTION` while those parameters are empty or malformed. Validate all fitted values and compensation signs in simulation before use on hardware.

Because the project was built with `--symlink-install`, a YAML-only change does not require rebuilding. Restart the launch process, then publish `JOINT_PD_FRIC` again. C++ or CMake changes do require rebuilding.

## Run package tests

After building, run all registered package tests and print their results:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon test
colcon test-result --verbose
```

The repository currently has limited automated controller tests. The primary smoke test is therefore: successful build, active lifecycle state, a live `/extended_robot_state` stream, a running MuJoCo simulation, and finite values on `/controller_logging`.

## Real robot configuration

For a real robot, set `is_simulation: false` and provide `gen3_ip` in `kinova_controller/config/kinova_controller_config.yaml`. Review every gain and limit before running; the checked-in values are research examples, not universal safe settings. Use the hardware emergency stop and validate the complete procedure in simulation first.

## Implementation map

- `kinova_controller/src/kinova_controller/controller/controller.cpp`
  - `FRIC_joint_controller()`: joint-space experiment controller with the simulation friction-model update
  - `FRIC_task_controller()`: task-space collision-tracking experiment controller; the alternative waypoint trajectory is retained in place
  - `implicit_l1_friction_compensation()`: implicit L1AFO update
  - `coulomb_observer_friction_compensation()`: Coulomb-observer comparison used in the paper
  - `asym_friction_compensation()`: identified asymmetric velocity/load/temperature friction model
  - `static_friction_compensation()`: identified static Coulomb and saturating velocity friction model
- `kinova_controller/src/kinova_controller/controller/controller_gain.cpp`: YAML gain loading and torque-interface selection
- `kinova_controller/src/kinova_controller/data/robot_state.cpp`: Pinocchio dynamics and state conversion
- `kinova_controller/src/kinova_controller/robot/`: MuJoCo and Kortex hardware interfaces

The friction controllers currently reproduce built-in experiment trajectories in
`FRIC_joint_controller()` and `FRIC_task_controller()`. Review those trajectories before running and do not assume that a desired-value topic overrides them. The paper interfaces are retained; select `IMPLICIT_L1_FRIC`, `COULOMB_OBSERVER`, `ASYM_FRICTION`, or `STATIC_FRICTION` as appropriate for the experiment.

## ROS 2 topics

- `controller_config` (`controller_interface_msgs/msg/ControllerConfig`): switch controller mode
- `desired_joint_value` (`controller_interface_msgs/msg/SetDesiredJointValue`): joint reference for controllers that use external references
- `desired_task_value` (`controller_interface_msgs/msg/SetDesiredTaskValue`): task-space reference for controllers that use external references
- `update_gain` (`controller_interface_msgs/msg/UpdateGain`): reload the installed YAML configuration
- `extended_robot_state` (`robot_state_msgs/msg/ExtendedState`): measured robot state
- `controller_logging` (`robot_state_msgs/msg/ControllerLogging`): L1AFO logging signals

## Data and local files

ROS build products, editor settings, experiment data, archives, logs, and locally installed third-party libraries are ignored by Git. Keep large datasets in a separate backup or data repository and publish only a small sample when it is needed to reproduce a figure.

## Citation

If you use this code, cite the associated paper:

```bibtex
@article{lee2026extreme,
  author  = {Lee, Young Bin and Yun, Tae Ho and Kim, Min Jun},
  title   = {Extreme High-Gain Friction Observer of Flexible Joint Robots With {L1} Adaptive Framework},
  journal = {IEEE Transactions on Robotics},
  volume  = {42},
  pages   = {2087--2106},
  year    = {2026},
  doi     = {10.1109/TRO.2026.3686177}
}
```

## License and patent review

This repository is not ready for public distribution. The software license, third-party model notices, and patent-related terms are intentionally deferred for review. Replace every `TODO` license entry and complete that review before making the repository public.
