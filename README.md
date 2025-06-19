# Inspire Retarget

A ROS2 package for controlling Unitree H1 (Inspire) robotic hands through motion retargeting from glove-based teleoperation systems.

## Overview

This package provides a bridge between motion retargeting systems and Unitree H1 robotic hands, enabling teleoperation of robotic hands using data gloves. The system subscribes to retargeted joint positions from a motion retargeting pipeline and converts them to control commands for the Inspire hands via serial communication.

## Features

- **Dual Hand Support**: Control both left and right Inspire hands simultaneously
- **Motion Retargeting Integration**: Subscribe to retargeted joint positions from motion retargeting systems
- **Serial Communication**: Direct serial communication with Inspire hands using custom protocol
- **Real-time Control**: Low-latency control loop for responsive hand manipulation
- **ROS2 Integration**: Native ROS2 node with parameterized configuration
- **DDS Support**: Optional DDS communication for integration with Unitree robotics ecosystem

## Architecture

The package consists of several key components:

### Core Components

1. **InspireHand Class** (`include/inspire_retarget/inspire.h`)
   - Handles serial communication with Inspire hands
   - Provides position, velocity, and force control interfaces
   - Supports error handling and calibration

2. **Controller Node** (`src/controller_node.cpp`)
   - Main ROS2 node that subscribes to retargeted joint positions
   - Converts joint positions to Inspire hand control commands
   - Manages dual hand control

3. **Serial Communication** (`include/inspire_retarget/SerialPort.h`)
   - Low-level serial port communication
   - Configurable baud rate and timeout settings

4. **DDS Utilities** (`include/inspire_retarget/dds/`)
   - Publisher and subscription wrappers for DDS communication
   - Real-time publishing capabilities

### Joint Mapping

The system maps 6-DOF retargeted joint positions to Inspire hand finger controls:

- **Joint 0-3**: Fingers (pinky, ring, middle, index) - mapped to position control [0, 1]
- **Joint 4**: Thumb bend - mapped with custom scaling
- **Joint 5**: Thumb rotation - mapped to position control [0, 1]

## Dependencies

### System Dependencies
- ROS2 (Humble or later)
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

3. **Publish retargeted joint positions** to the `/retargeted_qpos` topic:
   ```bash
   ros2 topic pub /retargeted_qpos std_msgs/msg/Float32MultiArray "data: [0.5, 0.5, 0.5, 0.5, 0.3, 0.5]"
   ```

### Launch File Usage

Use the provided launch file for a complete teleoperation setup:

```bash
ros2 launch inspire_retarget inspire_teleop_launch.py
```

The launch file includes:
- Glove visualizer node
- Cyber retargeting node
- Inspire controller node

### Command Line Options

The controller node supports several command line options:

```bash
ros2 run inspire_retarget controller_node [options]

Options:
  -h, --help                    Show help message
  -s, --serial <port>          Serial port (default: /dev/ttyUSB0)
  --network <interface>        DDS network interface
  --namespace <ns>             DDS topic namespace (default: inspire)
```

## Configuration

### Serial Port Configuration

Configure the serial port for your Inspire hands:

```bash
# For custom serial port
ros2 run inspire_retarget controller_node -s /dev/ttyUSB2
```

### Joint Mapping Configuration

The joint mapping can be customized in the `arrayToEigen` function in `controller_node.cpp`:

```cpp
Eigen::Matrix<double, 6, 1> arrayToEigen(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  // Customize mapping logic here
  // Joint 0-3: Finger positions
  // Joint 4: Thumb bend with custom scaling
  // Joint 5: Thumb rotation
}
```

## API Reference

### Topics

#### Subscribed Topics
- `/retargeted_qpos` (`std_msgs/msg/Float32MultiArray`)
  - Retargeted joint positions from motion retargeting system
  - Expected format: 6-element array [finger1, finger2, finger3, finger4, thumb_bend, thumb_rotation]

### Parameters

- `serial_port` (string, default: "/dev/ttyUSB0")
  - Serial port for Inspire hand communication

- `network` (string, default: "")
  - DDS network interface

- `namespace` (string, default: "inspire")
  - DDS topic namespace

### InspireHand Class Methods

```cpp
// Position control
int16_t SetPosition(const Eigen::Matrix<double, 6, 1> & q);

// Position reading
int16_t GetPosition(Eigen::Matrix<double, 6, 1> & q);

// Velocity control
void SetVelocity(int16_t v0, int16_t v1, int16_t v2, int16_t v3, int16_t v4, int16_t v5);

// Force control
void SetForce(uint16_t f0, uint16_t f1, uint16_t f2, uint16_t f3, uint16_t f4, uint16_t f5);
int16_t GetForce(Eigen::Matrix<double, 6, 1> & f);

// Error handling
void ClearError();

// Calibration
void Calibration();

// ID management
void ChangeID(uint8_t before, uint8_t now);
```

## Troubleshooting

### Common Issues

1. **Serial Port Access Denied**
   ```bash
   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   # Reboot or logout/login
   ```

2. **Inspire Hand Not Responding**
   - Check serial port connection
   - Verify hand power and initialization
   - Use `ClearError()` method to reset hand state

3. **Joint Mapping Issues**
   - Verify input data format (6-element array)
   - Check joint value ranges
   - Review mapping logic in `arrayToEigen` function

### Debug Mode

Enable debug logging by setting the log level:

```bash
ros2 run inspire_retarget controller_node --ros-args --log-level debug
```

## Development

### Building from Source

```bash
cd ~/ros2_ws
colcon build --packages-select inspire_retarget --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Testing

Run the package tests:

```bash
colcon test --packages-select inspire_retarget
```

### Code Style

The package follows ROS2 coding standards. Use the provided linting configuration:

```bash
colcon build --packages-select inspire_retarget --cmake-args -DBUILD_TESTING=ON
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

[License information to be added]

## Authors

- **zimo** (lizimo061@gmail.com)

## Acknowledgments

- Unitree Robotics for the H1 Inspire hand platform
- ROS2 community for the robotics middleware
- Motion retargeting community for teleoperation techniques
