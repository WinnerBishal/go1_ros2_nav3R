# ROS 2 Wrapper for Unitree Legged SDK

This repository provides the necessary configuration files to integrate the `unitree_legged_sdk` into a ROS 2 (`colcon`) workspace, enabling both C++ and Python bindings.

This wrapper is designed to be an overlay on the original SDK.

## Setup Instructions

1.  **Clone the Original SDK:**
    Navigate to your ROS 2 workspace `src` directory and clone the official `unitree_legged_sdk` repository:
    ```bash
    cd ~/ros2_ws/src
    git clone [https://github.com/unitreerobotics/unitree_legged_sdk/tree/go1](https://github.com/unitreerobotics/unitree_legged_sdk/tree/go1)
    ```

2.  **Replace/Add Config Files:**
    Copy the files from this repository into the `unitree_legged_sdk` directory you just cloned, replacing the originals:
    * `CMakeLists.txt` -> `unitree_legged_sdk/CMakeLists.txt`
    * `package.xml` -> `unitree_legged_sdk/package.xml`
    * `colcon.pkg` -> `unitree_legged_sdk/colcon.pkg`

3.  **Build the Workspace:**
    Go to the root of your workspace and build:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select unitree_legged_sdk
    ```

4.  **Source and Run:**
    Source the new setup file and run your nodes.
    ```bash
    source install/setup.bash
    ros2 run go1_comms go1_twist_publisher
    ```
