# CHAMP Legged Locomotion Framework for Scotty

This directory contains the custom implementation of the CHAMP (Quadrupedal Locomotion Controller) package tailored specifically for the Scotty quadruped robot. It supports both high-fidelity Gazebo simulation and real-world hardware integration.

---

## 1. Prerequisites and Installation

To set up the workspace, first install the main CHAMP framework packages:
1. Install the base CHAMP locomotion controllers: [chvmp/champ](https://github.com/chvmp/champ)
2. Install the setup assistant: [chvmp/champ_setup_assistant](https://github.com/chvmp/champ_setup_assistant)

*Original Note: There was an issue uploading the complete set of standard CHAMP dependency packages to Git. To resolve this, they are provided as a pre-packaged ZIP archive (`Scotty-main.zip`) in the parent directory. Other Scotty-specific custom integration packages are contained directly in this folder.*

---

## 2. Compilation and Workspace Build

When building the workspace after a clean setup, a specific build order is required. The custom hardware interface packages generate custom messages that must be fully compiled before the downstream controller nodes can be linked:

```bash
catkin clean
# Build the message-generating hardware package first
catkin build scotty_hw_interface
# Build the rest of the workspace
catkin build
source devel/setup.bash
```

*Deploying to hardware: To copy the compiled binaries and configuration folders to the onboard UP Board computer, use the utility script located at `scotty_hw_interface/scripts/copy_to_UP.sh`.*

---

## 3. Running the Controllers

### A. Simulation Mode (Gazebo & RViz)
To launch the complete simulation environment, load the URDF, and initialize the finite state machine without physical hardware:

```bash
roslaunch scotty_controller scotty_main_controller.launch hardware_connected:=False
```

### B. Physical Hardware Mode
To run the high-level ROS nodes connected to the physical robot's SPI/CAN communication loops:

```bash
roslaunch scotty_controller scotty_main_controller.launch hardware_connected:=True
```

---

## 4. Joint Controller Tuning GUI & Calibration

This package features a slider-based GUI designed to manually calibrate motor positions and tune joint proportional-derivative ($K_p$, $K_d$), velocity, and feed-forward torque parameters.

1. **Enable GUI Mode in Code**:
   Prior to compiling, open [scotty_hw_interface/src/hw_interface.cpp](./scotty_hw_interface/src/hw_interface.cpp) and uncomment the debug macro define (this must be commented out for normal walking usage):
   ```cpp
   #define DEBUG_JOINT_CONTROL_GUI
   ```
2. **Build and Launch**:
   Rebuild the workspace and launch the interface test panel:
   ```bash
   roslaunch scotty_hw_interface ScottyJointControlTest.launch
   ```

---

## 5. Folder Structure
* **`scotty_config/`**: Gait parameters, joint limits, and coordinate descriptions.
* **`scotty_control/`**: Finite State Machine (FSM) manager, web dashboard.
* **`scotty_description/`**: Robot visual and collision URDF mesh files.
* **`scotty_hw_interface/`**: ROS C++ hardware node handling SPI and CAN packet marshalling.

---

### Developer Note on Compilation Dependencies
* **CMakeLists custom message generation dependency**: The requirement to build `scotty_hw_interface` first (before running a general `catkin build`) is needed to generate custom ROS messages used by Scotty. It might be possible to resolve this dependency issue directly inside the `CMakeLists.txt` of `scotty_hw_interface` to automate this sequencing.