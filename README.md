# ROS2-GO1 Navigation and 3D Reconstruction Project

As a part of this project, we also provide:

## ROS 2 Wrapper for Unitree Legged SDK (GO1) Tested with Ubuntu 22.04

This repository provides the necessary configuration files to integrate the `unitree_legged_sdk` into a ROS 2 (`colcon`) workspace, enabling both C++ and Python bindings.

This wrapper is designed to be an overlay on the original SDK. This enables direct velocity control of Unitree GO1 Quadruped robot through ROS2 python nodes.

### Setup Instructions

1. **Clone this repository:**
    ```bash
    git clone https://github.com/WinnerBishal/go1_ros2_nav3R.git
    ```

2.  **Clone the Original SDK:**
    Navigate to your ROS 2 workspace `src` directory and clone the official `unitree_legged_sdk` repository.

    Since ~/go1_ros2_nav3R/src/unitree_legged_sdk already exists and contains three important files `CMakelists.txt`, `colcon.pkg` and `package.xml` that we want to replace into the original sdk, we will first rename the `unitree_legged_sdk` folder inside `~/go1_ros2_nav3R/src/` to `sdk_backup` and clone the original `unitree_legged_sdk`. 
    Then, move the three files from `sdk_backup` to the `unitree_legged_sdk`.

    ```bash
    cd ~/go1_ros2_nav3R/src
    # Save our provided replacements to backup
    mv unitree_legged_sdk sdk_backup   
    # Clone original SDK
    git clone https://github.com/unitreerobotics/unitree_legged_sdk.git 
    cd unitree_legged_sdk
    # Remove original files
    rm -rf CMakelists.txt package.xml       
    cd ../sdk_backup
    # Move our files to original SDK
    mv CMakelists.txt package.xml colcon.pkg ../unitree_legged_sdk/ 
    cd ..
    # Clean the src/
    rm -rf sdk_backup       
    ```

3. **Install/Check Dependencies:**
    The build might fail due to various dependencies missing or being old. Make sure of the following:
    
    Following settings worked on Raspberry Pi 5 with **Ubuntu 24.04** and **ROS2 Jazzy** installed.
    **Before Step 4**, do the following:
    ```bash
    # Install libmsgpack
    sudo apt-get install libmsgpack-dev

    # Remove the old bundled pybind11
    rm -rf ~/ros2_ws/src/unitree_legged_sdk/python_wrapper/third-party/pybind11

    # Clone the new, compatible version
    git clone https://github.com/pybind/pybind11.git ~/ros2_ws/src/unitree_legged_sdk/python_wrapper/third-party/pybind11
    ```

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
