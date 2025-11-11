# ROS2-GO1 Navigation and 3D Reconstruction Project

As a part of this project, we also provide:

## ROS 2 Wrapper for Unitree Legged SDK (GO1) Tested with Ubuntu 22.04

This repository provides the necessary configuration files to integrate the `unitree_legged_sdk` into a ROS 2 (`colcon`) workspace, enabling both C++ and Python bindings.

This wrapper is designed to be an overlay on the original SDK. This enables direct velocity control of Unitree GO1 Quadruped robot through ROS2 python nodes.

### Setup Instructions

1. **Clone this repository:**
    ```bash
    git clone [https://github.com/WinnerBishal/go1_ros2_nav3R.git](https://github.com/WinnerBishal/go1_ros2_nav3R.git)
    ```

2.  **Clone the Original SDK:**
    Navigate to your ROS 2 workspace `src` directory and clone the official `unitree_legged_sdk` repository:
    ```bash
    cd ~/go1_ros2_nav3R/src
    git clone [https://github.com/unitreerobotics/unitree_legged_sdk.git](https://github.com/unitreerobotics/unitree_legged_sdk.git)
    ```
    NOTE: Follow the instruction at [official_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk.git) to download and install the dependencies to successfully follow the steps below.

3.  **Replace/Add Config Files:**
    Copy the files from this repository into the fresh `unitree_legged_sdk` directory you just cloned, replacing the originals:
    * `CMakeLists.txt` -> `unitree_legged_sdk/CMakeLists.txt`
    * `package.xml` -> `unitree_legged_sdk/package.xml`
    * `colcon.pkg` -> `unitree_legged_sdk/colcon.pkg`

4.  **Build the Workspace:**
    Go to the root of your workspace and build:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select unitree_legged_sdk
    ```

5.  **Source and Run example nodes:**
    Source the new setup file and run your nodes.
    ```bash
    source install/setup.bash
    ros2 run go1_comms go1_twist_publisher
    ```
