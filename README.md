# Inspire Retarget

A ROS2 package for controlling Inspire robotic hands through motion retargeting from glove-based teleoperation systems. Thank Unitree for open sourcing, most of codes are from: https://support.unitree.com/home/en/H1_developer/Dexterous_hand

## Overview

This package provides a bridge between motion retargeting systems and inspire robotic hands, enabling teleoperation of robotic hands using data gloves. The system subscribes to retargeted joint positions from a motion retargeting pipeline and converts them to control commands for the Inspire hands via serial communication.

## Features

- **Dual Hand Support**: Control both left and right Inspire hands simultaneously
- **Motion Retargeting Integration**: Subscribe to retargeted joint positions from motion retargeting systems
- **Serial Communication**: Direct serial communication with Inspire hands using custom protocol
- **Real-time Control**: Low-latency control loop for responsive hand manipulation
- **ROS2 Integration**: Native ROS2 node with parameterized configuration

## Dependencies

### System Dependencies
- ROS2 (Humble or later, tested with Jazzy)
- Boost libraries (program_options, system, thread)
- Eigen3
- fmt library
- Unitree SDK2

### Package Dependencies
- `rclcpp`
- `std_msgs`
- `boost`
- `fmt`

## Installation

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> inspire_retarget
   ```

2. **Install system dependencies**:
   ```bash
   sudo apt update
   sudo apt install libboost-program-options-dev libboost-system-dev libboost-thread-dev
   ```

3. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select inspire_retarget
   ```

4. **Source the workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Basic Usage

1. **Connect Inspire hands** to USB serial ports (default: `/dev/ttyUSB0` for right hand, `/dev/ttyUSB1` for left hand)

2. **Run the controller node**:
   ```bash
   ros2 run inspire_retarget controller_node
   ```

### Serial Port Configuration

Configure the serial port for your Inspire hands:

```bash
# For custom serial port
ros2 run inspire_retarget controller_node -s /dev/ttyUSB2
```