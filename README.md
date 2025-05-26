# Project R: RoboMaster fire detection

A ROS2 package implementing autonomous navigation and fire detection for the RoboMaster robot. The robot autonomously explores its environment while avoiding obstacles and detects fires using computer vision.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Topics](#topics)
  - [Subscribed Topics](#subscribed-topics)
  - [Published Topics](#published-topics)
- [Parameters](#parameters)
- [Behavior States](#behavior-states)
- [Debug Information](#debug-information)
- [License](#license)
- [Authors](#authors)
- [Acknowledgments](#acknowledgments)

## Features

- **Autonomous Navigation**: The robot explores the environment using range sensors for obstacle detection and avoidance
- **Fire Detection**: Uses computer vision (OpenCV) to detect fires in real-time using color segmentation
- **State Machine**: Implements different behavioral states for navigation and celebration
- **Debug Visualization**: Provides visual feedback for fire detection through debug image topics

## Installation

1. Clone this repository:

```bash
git clone https://github.com/riccardosacco/robotics-final-ws.git --recursive
cd robotics-final-ws
```

2. Build the workspace:

```bash
pixi install
pixi shell
colcon build --symlink-install
```

3. Run Coppelia:

```bash
source install/setup.zsh
pixi run coppelia
```

4. Load one of the scenes from the folder "src/project_r/scenes" into Coppelia

5. Run the RoboMaster EP ToF driver:

```bash
source install/setup.zsh
ros2 launch project_r ep_tof.launch.py
```

6. Run the Robot controller:

```bash
source install/setup.zsh
ros2 launch project_r main.launch.py
```

## Topics

### Subscribed Topics

- `/odom` (nav_msgs/Odometry): Robot odometry
- `/camera/image_color` (sensor_msgs/Image): Camera feed for fire detection
- `/range_[0-3]` (sensor_msgs/Range): Range sensor readings

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist): Robot velocity commands
- `/fire_debug/image` (sensor_msgs/Image): Debug visualization with detected fire regions
- `/fire_debug/mask` (sensor_msgs/Image): Fire detection mask

## Parameters

- `UPDATE_RATE`: Control loop frequency (default: 20 Hz)
- `TARGET_DISTANCE`: Desired distance from obstacles (default: 0.3 m)
- `TOO_CLOSE`: Distance that triggers backup (default: 0.2 m)
- `MIN_FREE_SPACE`: Minimum space needed to move forward (default: 0.5 m)
- `FIRE_MIN_AREA`: Minimum pixel area for fire detection (default: 12000)

## Behavior States

1. **FORWARD**: Robot moves straight until detecting an obstacle
2. **BACKUP**: Robot backs up when too close to obstacles
3. **ROTATING**: Robot rotates to find a clear path
4. **FIRE_FOUND**: Robot stops when fire is detected
5. **CELEBRATING**: Robot performs a victory dance

## Debug Information

To view the fire detection debug images:

```bash
rqt
```

Then select either `/fire_debug/image` or `/fire_debug/mask` topics.

## License

Apache License 2.0

## Authors

- Riccardo Sacco (<riccardo.sacco@usi.ch>)
- R. Kalvitis (<r.kalvitis@usi.ch>)

## Acknowledgments

This project was developed as part of the Robotics course at USI (Università della Svizzera italiana).
