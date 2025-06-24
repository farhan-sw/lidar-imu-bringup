# LIDAR-IMU Bringup Package

This comprehensive guide will help you set up and run the `jolie_localization` hardware launch file from scratch. This package integrates SLAMTEC LIDAR and WIT IMU sensors for robotics applications.

## Table of Contents
1. [System Requirements](#system-requirements)
2. [Initial Setup](#initial-setup)
3. [Install ROS2](#install-ros2)
4. [Workspace Setup](#workspace-setup)
5. [Hardware Setup](#hardware-setup)
6. [USB Permissions & Device Rules](#usb-permissions--device-rules)
7. [Dependencies Installation](#dependencies-installation)
8. [Building the Workspace](#building-the-workspace)
9. [Hardware Configuration](#hardware-configuration)
10. [Running the Hardware Launch](#running-the-hardware-launch)
11. [Troubleshooting](#troubleshooting)
12. [Advanced Usage](#advanced-usage)

## System Requirements

- **Operating System**: Ubuntu 20.04 LTS or Ubuntu 22.04 LTS
- **ROS2 Distribution**: Humble (recommended) or Galactic
- **Hardware**:
  - SLAMTEC LIDAR (C1 model by default)
  - WIT IMU sensor
  - USB-to-Serial adapters (if needed)
- **Minimum RAM**: 4GB
- **Storage**: At least 10GB free space

## Initial Setup

### 1. Update Your System

```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Install Essential Tools

```bash
sudo apt install -y \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-dev \
    udev \
    usbutils
```

## Install ROS2

### For Ubuntu 22.04 (ROS2 Humble)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y
```

### For Ubuntu 20.04 (ROS2 Galactic)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS2 GPG key
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Galactic
sudo apt update
sudo apt install ros-galactic-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y
```

### Setup ROS2 Environment

Add ROS2 to your shell profile:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# OR for Galactic: echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# Reload bashrc
source ~/.bashrc
```

Verify installation:

```bash
ros2 --help
```

## Workspace Setup

### 1. Create ROS2 Workspace

```bash
# Navigate to your desired location (adjust path as needed)
cd ~/Documents/Github/

# Create workspace structure
mkdir -p ros2_ws/src
cd ros2_ws/src
```

### 2. Clone or Copy Your Package

If your package is already in the correct location as shown in the context, you should have:
```
ros2_ws/src/lidar-imu-bringup/
```

If not, you can clone or create the structure:

```bash
# If you need to clone from a repository
git clone https://github.com/farhan-sw/lidar-imu-bringup.git
```

## Hardware Setup

### 1. Connect Your Hardware

1. **LIDAR Connection**:
   - Connect your SLAMTEC LIDAR to a USB port
   - Note which USB port it uses (usually `/dev/ttyUSB0` or `/dev/ttyUSB1`)

2. **IMU Connection**:
   - Connect your WIT IMU to another USB port
   - Note which USB port it uses

### 2. Identify Device Ports

Check connected USB devices:

```bash
# List all USB devices
lsusb

# Check serial devices
ls -la /dev/ttyUSB*

# Check device information
udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0)
```

Expected output should show devices with vendor/product IDs like:
- **CP2102 USB-Serial**: `idVendor=10c4, idProduct=ea60`

## USB Permissions & Device Rules

### 1. Add User to Dialout Group

```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER
```

### 2. Create UDEV Rules for Consistent Device Names

**Important**: First, identify your device IDs by checking connected devices:

```bash
# Check device information for ttyUSB0
udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0) | grep -E "idVendor|idProduct"

# Check device information for ttyUSB1  
udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB1) | grep -E "idVendor|idProduct"
```

Create a single udev rule file for both devices:

```bash
sudo nano /etc/udev/rules.d/99-sensors.rules
```

Add this content (adjust vendor/product IDs based on your actual devices):

```
# LIDAR udev rule (CP2102 USB-Serial converter - typically 10c4:ea60)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"

# IMU udev rule (CH340 USB-Serial converter - typically 1a86:7523)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="imu_usb"
```

**Note**: The example above shows typical IDs for:
- LIDAR: CP2102 converter (`10c4:ea60`)
- IMU: CH340 converter (`1a86:7523`)

Your devices may have different IDs, so **always check first** using the `udevadm` commands above.

### 3. Reload UDEV Rules

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 4. Logout and Login

```bash
# Logout and login again for group changes to take effect
# OR restart your system
sudo reboot
```

### 5. Verify Device Access

After reboot, check if symbolic links are created correctly:

```bash
# Check symbolic links
ls -la /dev/rplidar /dev/imu_usb /dev/ttyUSB*

# Verify they point to different devices
echo "LIDAR: /dev/rplidar -> $(readlink /dev/rplidar)"
echo "IMU: /dev/imu_usb -> $(readlink /dev/imu_usb)"

```

**Expected output:**
```
lrwxrwxrwx 1 root root 7 Jun 24 12:53 /dev/rplidar -> ttyUSB0
lrwxrwxrwx 1 root root 7 Jun 24 12:53 /dev/imu_usb -> ttyUSB1
crwxrwxrwx 1 root dialout 188, 0 Jun 24 12:53 /dev/ttyUSB0
crwxrwxrwx 1 root dialout 188, 1 Jun 24 12:53 /dev/ttyUSB1
```

**Important**: Make sure `/dev/rplidar` and `/dev/imu_usb` point to **different** ttyUSB devices!

## Dependencies Installation

### 1. Install ROS2 Package Dependencies

```bash
# Navigate to workspace root
cd ~/Documents/Github/school/ros2_ws

# Install dependencies using rosdep
sudo rosdep init  # Only if never run before
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Install Additional Dependencies

```bash
# Install additional ROS2 packages
sudo apt install -y \
    ros-humble-sensor-msgs \
    ros-humble-std-srvs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-rclcpp \
    ros-humble-rclpy

# For Galactic, replace 'humble' with 'galactic' in above commands

# Install Python dependencies
pip3 install -r requirements.txt  # If you have a requirements.txt file
# OR install commonly needed packages:
pip3 install pyserial numpy
```

## Building the Workspace

### 1. Build All Packages

```bash
# Navigate to workspace root
cd ~/Documents/Github/school/ros2_ws

# Build the workspace
colcon build

# If you encounter memory issues (for low-memory systems):
colcon build --parallel-workers 1
```

### 2. Source the Workspace

```bash
# Source the workspace
source install/setup.bash

# Add to bashrc for permanent effect
echo "source ~/Documents/Github/school/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 3. Verify Build

```bash
# Check if packages are built
ros2 pkg list | grep jolie_localization
ros2 pkg list | grep sllidar_ros2  
ros2 pkg list | grep wit_ros2_imu
```

## Hardware Configuration

### 1. LIDAR Configuration

The hardware launch file is configured for SLAMTEC C1 LIDAR with these default settings:
- **Serial Port**: `/dev/ttyUSB0` (or `/dev/rplidar` with udev rules)
- **Baudrate**: `460800`
- **Frame ID**: `laser`
- **Scan Mode**: `Standard`

### 2. IMU Configuration

The IMU is configured with these default settings:
- **Port**: `/dev/imu_usb`
- **Baudrate**: `9600`

### 3. Customize Settings (Optional)

You can override default settings when launching. See [Running the Hardware Launch](#running-the-hardware-launch) section.

## Running the Hardware Launch

### 1. Basic Launch

```bash
# Make sure workspace is sourced
source ~/Documents/Github/school/ros2_ws/install/setup.bash

# Launch with default settings
ros2 launch jolie_localization hardware.launch.py
```

### 2. Launch with Custom Parameters

```bash
# Launch with custom LIDAR port
ros2 launch jolie_localization hardware.launch.py lidar_serial_port:=/dev/rplidar

# Launch with custom IMU port
ros2 launch jolie_localization hardware.launch.py imu_port:=/dev/imu_usb

# Launch with multiple parameters
ros2 launch jolie_localization hardware.launch.py \
    lidar_serial_port:=/dev/rplidar \
    imu_port:=/dev/imu_usb \
    lidar_baudrate:=230400

# Launch only LIDAR (disable IMU)
ros2 launch jolie_localization hardware.launch.py use_imu:=false

# Launch only IMU (disable LIDAR)
ros2 launch jolie_localization hardware.launch.py use_lidar:=false

# Launch with all sensors enabled
ros2 launch jolie_localization hardware.launch.py use_lidar:=true use_imu:=true
```

### 3. Available Launch Arguments

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| `use_lidar` | `true` | Enable/disable LIDAR |
| `use_imu` | `true` | Enable/disable IMU |
| `lidar_serial_port` | `/dev/rplidar` | LIDAR serial port |
| `lidar_baudrate` | `460800` | LIDAR communication baudrate |
| `lidar_frame_id` | `laser` | LIDAR frame ID |
| `lidar_inverted` | `false` | Invert LIDAR scan data |
| `lidar_angle_compensate` | `true` | Enable angle compensation |
| `lidar_scan_mode` | `Standard` | LIDAR scan mode |
| `imu_port` | `/dev/imu_usb` | IMU port |
| `imu_baudrate` | `9600` | IMU communication baudrate |

### 4. Verify Launch Success

In a new terminal, check if nodes are running:

```bash
# List active nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor LIDAR data
ros2 topic echo /scan

# Monitor IMU data  
ros2 topic echo /imu/data
```

Expected nodes:
- `/sllidar_node` (LIDAR)
- `/imu` (IMU)

Expected topics:
- `/scan` (LIDAR scan data)
- `/imu/data` (IMU data)

## Troubleshooting

### Common Issues and Solutions

#### 1. Permission Denied Errors

**Problem**: `Permission denied` when accessing `/dev/ttyUSB*`

**Solution**:
```bash
# Check current permissions
ls -la /dev/ttyUSB*

# Add user to groups (if not done before)
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER

# Logout and login again, then test
logout
# Login again and test device access
```

#### 2. Device Not Found

**Problem**: `No such file or directory: '/dev/ttyUSB0'`

**Solutions**:
```bash
# Check connected devices
lsusb
ls -la /dev/ttyUSB*

# If no ttyUSB devices, check if drivers are loaded
dmesg | grep -i usb
dmesg | grep -i cp210

# Install USB-serial drivers if needed
sudo apt install linux-modules-extra-$(uname -r)

# Check with different ports
ros2 launch jolie_localization hardware.launch.py lidar_serial_port:=/dev/ttyUSB1
```

#### 3. Build Errors

**Problem**: `Package 'xyz' not found`

**Solution**:
```bash
# Update rosdep and install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install missing ROS2 packages
sudo apt update
sudo apt install ros-humble-<missing-package>

# Clean and rebuild
rm -rf build install log
colcon build
```

#### 4. LIDAR Connection Issues

**Problem**: LIDAR not publishing data

**Solution**:
```bash
# Check LIDAR connection
sudo minicom -D /dev/ttyUSB0 -b 460800
# Press Ctrl+A then X to exit

# Try different baudrates
ros2 launch jolie_localization hardware.launch.py lidar_baudrate:=230400
ros2 launch jolie_localization hardware.launch.py lidar_baudrate:=115200

# Check LIDAR health
ros2 service call /stop_motor std_srvs/srv/Empty
ros2 service call /start_motor std_srvs/srv/Empty
```

#### 5. IMU Connection Issues

**Problem**: IMU not publishing data

**Solution**:
```bash
# Test IMU connection
sudo minicom -D /dev/ttyUSB1 -b 9600
# You should see binary data

# Try different baudrates
ros2 launch jolie_localization hardware.launch.py imu_baudrate:=115200

# Check IMU topics
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

#### 6. Multiple USB Devices Conflict

**Problem**: Devices swapping ports after reboot or both symbolic links pointing to same device

**Solution**:
```bash
# Check what devices you actually have connected
lsusb
ls -la /dev/ttyUSB*

# Check device specific IDs for each port
udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0) | grep -E 'idVendor|idProduct'
udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB1) | grep -E 'idVendor|idProduct'

# Create specific rules in /etc/udev/rules.d/99-sensors.rules based on YOUR device IDs:
# Example for common device combinations:

# Option 1: LIDAR=CP2102, IMU=CH340 (most common)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="imu_usb"

# Option 2: Both devices are CP2102 (use serial numbers)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<lidar-serial>", SYMLINK+="rplidar"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<imu-serial>", SYMLINK+="imu_usb"

# Option 3: Both devices are CH340 (use devpath)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{../devpath}=="4", SYMLINK+="rplidar" 
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{../devpath}=="5", SYMLINK+="imu_usb"

# Reload rules after editing
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify the symbolic links point to different devices
ls -la /dev/rplidar /dev/imu_usb
```

## Advanced Usage

### 1. Launching with RViz Visualization

```bash
# Install RViz if not already installed
sudo apt install ros-humble-rviz2

# Launch hardware and RViz
ros2 launch jolie_localization hardware.launch.py &
rviz2

# In RViz:
# 1. Set Fixed Frame to "laser" or "imu_link" or "base_link" (depending on your setup)
# 2. Add LaserScan display with topic "/scan"
# 3. Add Imu display with topic "/imu/data"
```

### 2. Recording Data

```bash
# Record sensor data
ros2 bag record /scan /imu/data

# Play back recorded data
ros2 bag play <bag-file>
```

### 3. Parameter Tuning

```bash
# Check current LIDAR parameters
ros2 param list /sllidar_node
ros2 param get /sllidar_node frame_id

# Check IMU parameters  
ros2 param list /imu
ros2 param get /imu port
```

### 4. Integration with Navigation Stack

```bash
# Install navigation packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Launch navigation with your sensors
ros2 launch nav2_bringup navigation_launch.py
```

### 5. Custom Launch Files

Create your own launch file by copying and modifying `hardware.launch.py`:

```bash
cp src/lidar-imu-bringup/source/jolie_localization/launch/hardware.launch.py \
   src/lidar-imu-bringup/source/jolie_localization/launch/my_custom.launch.py

# Edit the file to customize parameters
nano src/lidar-imu-bringup/source/jolie_localization/launch/my_custom.launch.py
```

## Monitoring and Diagnostics

### Real-time Monitoring

```bash
# Monitor node status
ros2 node info /sllidar_node
ros2 node info /imu

# Monitor topic frequencies
ros2 topic hz /scan
ros2 topic hz /imu/data

# Monitor system resources
htop
```

### Logging

```bash
# Check ROS2 logs
ros2 log level set /sllidar_node DEBUG
ros2 log level set /imu DEBUG

# System logs
journalctl -f | grep -i usb
dmesg | tail -20
```

---


For additional help, check the ROS2 documentation or ask questions in ROS community forums or email farhan.sw@gmail.com.