
# CHAMP for Scotty

CHAMP package for Scotty with custom controller, tools for debugging.


## Installation

Install the champ files from https://github.com/chvmp/champ and setup assistant from https://github.com/chvmp/champ_setup_assistant

There was issue in uploading champ packages to git. But uploaded them some a zip. Other scotty specific packages are here.    
## Compile

In order to compile it after a complete clean :

```bash
  catkin clean
  catkin build scotty_hw_interface
  catkin build
```

This is needed to generate a custom message used in Scotty. Might be able to fix though CMakeLists.txt of scotty_hw_interface.

In order to copy the contents to UP Board, use the script /scotty_hw_interface/scripts/copy_to_UP.sh
## Scotty Controller
The custom controller for Scotty. It can be used for both simulation and hardware.


To run in simulation mode : 

```bash
  roslaunch scotty_controller scotty_main_controller.launch  hardware_connected:=False
```

To run with Scotty : 

```bash
  roslaunch scotty_controller scotty_main_controller.launch  hardware_connected:=True
```
## Joint Controller Tuning GUI
This is to run a GUI for tuning the Position, Kp, Kd, Velocity, Torque.

To execute, run : 

```bash
  roslaunch scotty_hw_interface ScottyJointControlTest.launch
```

Also in the scotty_hw_interface/src/hw_interface.cpp, enable the line below. This has to be commented for normal usage.

```bash
#define DEBUG_JOINT_CONTROL_GUI
```