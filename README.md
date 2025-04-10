# RoboMaster Installation Guide

This guide provides step-by-step instructions for installing the RoboMaster Simulation running in CoppeliaSim using Pixi.

---

## 🧩 Install CoppeliaSim

### macOS

1. Download CoppeliaSim for [Apple Silicon](https://downloads.coppeliarobotics.com/V4_7_0_rev4/CoppeliaSim_Edu_V4_7_0_rev4_macOS14_arm64.zip) or [Intel](https://downloads.coppeliarobotics.com/V4_7_0_rev4/CoppeliaSim_Edu_V4_7_0_rev4_macOS13_x86_64.zip)
2. Unzip and move to `/Applications/coppeliaSim.app`
3. Right-click on `coppeliaSim.app` -> Open -> Open
4. Add the following to the `pixi.toml` in your pixi project:
   ```toml
   [activation.env]
   COPPELIASIM_ROOT_DIR = "/Applications/coppeliaSim.app/Contents/Resources"
   ```
5. CoppeliaSim must be launched from the Terminal (this will become important later):
   ```bash
   /Applications/coppeliaSim.app/Contents/MacOS/coppeliaSim
   ```

#### Troubleshooting

- If you encounter any permission errors, please authorize coppeliaSim in the System Settings:
  ![CoppeliaSim settings](https://www.icorsi.ch/draftfile.php/646215/user/draft/256848284/coppelia_settings.png)

### Ubuntu

1. Download CoppeliaSim for [Ubuntu 22.04](https://downloads.coppeliarobotics.com/V4_7_0_rev4/CoppeliaSim_Edu_V4_7_0_rev4_Ubuntu22_04.tar.xz) or [Ubuntu 24.04](https://downloads.coppeliarobotics.com/V4_7_0_rev4/CoppeliaSim_Edu_V4_7_0_rev4_Ubuntu24_04.tar.xz)
2. Extract CoppeliaSim in a directory of your choice, for example in your `COURSE_FOLDER`:
   ```bash
   cd <COURSE_FOLDER>
   tar xvf CoppeliaSim_Edu_V4_7_0_rev4_Ubuntu<UBUNTU_VERSION>.tar.xz
   ```
3. Add to `pixi.toml`:
   ```toml
   [activation.env]
   COPPELIASIM_ROOT_DIR = "<PATH_TO_COPPELIA>/CoppeliaSim_Edu_V4_7_0_rev4_Ubuntu<UBUNTU_VERSION>"
   ```
4. CoppeliaSim must be launched from the Terminal (this will become important later):
   ```bash
   <PATH_TO_COPPELIA>/CoppeliaSim_Edu_V4_7_0_rev4_Ubuntu<UBUNTU_VERSION>/coppeliaSim.sh
   ```

---

## 🤖 RoboMaster quick setup

We prepared a Pixi project for you with all required dependencies for using the RoboMaster in CoppeliaSim. You can set it up by cloning this repository:

```bash
git clone git@github.com:idsia-robotics/robotics-lab-usi-robomaster.git --recursive
```

Follow the CoppeliaSim Installation Guide to setup CoppeliaSim on your computer.
**Ubuntu:** You need to customize the `COPPELIASIM_ROOT_DIR` in the `pixi.toml` of this repo to point to your CoppeliaSim installation.

Now enter the repository, compile and install the packages:

```bash
cd robotics-lab-usi-robomaster
pixi install
pixi shell
colcon build --symlink-install
```

You can launch CoppeliaSim with this command. This is important to ensure that Coppelia is started with the correct environment variables to see the ROS packages installed in this repo:

```bash
pixi run coppelia
```

---

Your installation is complete! 🎉

The following instructions walk you through the process of setting up a Pixi project from scratch.
We leave them here as a reference, in case you need to do it yourself in the future for new projects.

**You don't need to go through them right now!**

---

## 🤖 Installing RoboMaster SDK

This is a fork of the official DJI RoboMaster Python API modified to solve some issues in using these robots with ROS2.

Inside the `<ROS_PROJECT_FOLDER>`.

```bash
pixi shell
pixi add libopus
pixi add --pypi "robomaster@git+https://github.com/jeguzzi/RoboMaster-SDK.git"
pixi add --pypi "libmedia_codec@git+https://github.com/jeguzzi/RoboMaster-SDK.git#subdirectory=lib/libmedia_codec"
```

---

## 💻 Installing RoboMaster Sim

This package provides RoboMaster models for CoppeliaSim. It includes 3D graphics, dynamics and sensors models and a control interface that mimics the real RoboMaster's network protocol.

Inside the `<ROS_PROJECT_FOLDER>`.

### 1. Install the Dependencies

```bash
pixi shell
pixi add spdlog boost cmake ffmpeg libxslt xmlschema
```

### 2. Configure environment

Create a `pixi-toolchain.cmake` file in your `<ROS_PROJECT_FOLDER>` with the following content:

```cmake
# Force CMake to look for all shared libraries (spdlog, Boost, etc.) inside the pixi environment
set(CMAKE_FIND_ROOT_PATH "$ENV{CONDA_PREFIX}")
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Force Coppelia to load the exact version of the dylibs used at build time, instead of letting
# the OSes to resolve a version at runtime with their usual opaque logic.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH True)
```

Add the following to your `pixi.toml` so that CMake finds the toolchain file:

```toml
[activation.env]
CMAKE_TOOLCHAIN_FILE = "$PIXI_PROJECT_ROOT/pixi-toolchain.cmake"
```

### 3. Clone the repo

```bash
mkdir -p src
cd src
git clone git@github.com:jeguzzi/robomaster_sim.git
```

### 4. Build

```bash
cd ..
colcon build --symlink-install
```

The symlink install option install Python ROS2 packages using symlink from the install to the src folder making them editable. This only applies to Python files and not to config or launch files.

### 5. Test your installation

1. Open CoppeliaSim.
2. Add RoboMaster via: `Model browser (the left sidebar) -> robots -> mobile -> RoboMasterEP`
3. Press play and ensure no errors are printed in the Coppelia log (Bottom).

With CoppeliaSim running and a Robomaster in the scene.

```bash
cd src/robomaster_sim/examples
pixi shell
python discover.py
```

This script should find the RoboMaster in the scene, print some info about it, and close itself.

---

## 🚀 Installing RoboMaster ROS

This package provides ROS drivers for the RoboMaster, whether real or simulated. Inside the `<ROS_PROJECT_FOLDER>`:

```bash
pixi shell
pixi add ros-humble-xacro ros-humble-launch-xml ros-humble-cv-bridge ros-humble-launch-testing-ament-cmake ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-joy
pixi add --pypi numpy-quaternion
```

### 1. Clone the repo

```bash
cd src
git clone https://github.com/jeguzzi/robomaster_ros.git
```

### 2. Build

```bash
cd ..
colcon build --symlink-install
```

### 3. Launch

Source the setup file:

```bash
source install/setup.zsh
```

or

```bash
source install/setup.bash
```

Sourcing this file add your custom packages to the available ROS2 packages (sourced using `pixi shell`) allowing you to launch them.

Launch the RoboMaster driver ROS nodes:

```bash
ros2 launch robomaster_ros ep.launch
```

### 4. Test

In another terminal, inside the `<ROS_PROJECT_FOLDER>`.

```bash
pixi shell
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

The robot should start to rotate counter-clockwise.

Use this command to stop it:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 🔗 Installing simExtROS2

Finally, let's install the CoppeliaSim ROS2 plugin. This allows to interact with the simulation from ROS2, providing topics and services to start and stop the simulation, read the simulation clock, etc.

### 1. Clone the repo

```bash
cd src
git clone git@github.com:jeguzzi/simExtROS2.git -b iron-4.6
```

### 2. Build

```bash
pixi shell
colcon build --symlink-install
```