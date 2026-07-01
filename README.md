# Scotty Quadruped Control Frameworks Repository

This repository contains the complete control system software, kinematic models, microcode, and telemetry validation data developed for **Scotty**, a custom $19.31$~kg quadrupedal robot. 

## Master Thesis Report
The complete Master Thesis report on which this research is based is available directly in the home directory of this repository:
* **[Master Thesis PDF (Vishnudev Kurumbaparambil, 2025)](./Thesis_Analysis,%20Development%20and%20Integration%20of%20Control%20Frameworks%20for%20Scotty,%20a%20Custom%20Quadrupedal%20Robot.pdf)**

This codebase provides two primary architectural paradigms:
1. **CHAMP-based Integration**: A modular, position-based trajectory control pipeline fully integrated with the Robot Operating System (ROS) and validated in Gazebo simulation, alongside preliminary joint-level hardware interfacing.
2. **MIT Mini Cheetah adaptation**: An adaptation of the torque-based, hardware-specific control framework, tested on a suspended calibration rig to identify low-level hardware constraints.

---

## Quick Start & Reproduction Guide

### 1. Reproducing the Gazebo Locomotion Simulation (CHAMP)
To build the ROS workspace and launch Scotty in a simulated environment:

1. **System Requirements**: Ubuntu 20.04 LTS with ROS Noetic and Gazebo 11.
2. **Set up Workspace**:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   # Clone or move this directory (Code) into catkin_ws/src
   ```
3. **Build the packages**:
   ```bash
   cd ~/catkin_ws
   catkin clean
   catkin build scotty_hw_interface
   catkin build
   source devel/setup.bash
   ```
4. **Launch Simulation**:
   ```bash
   roslaunch scotty_controller scotty_main_controller.launch hardware_connected:=False
   ```
   *This loads the state-machine, spawns the URDF model of Scotty in Gazebo in the `Idle` state, and prepares the ROS joint controllers.*

---

### 2. Reproducing the Hardware Interface & Joint Calibration
To test the SPI-to-CAN communication loops and joint position tracking on the physical hardware:

1. **Flash Microcontroller Firmware**:
   * Open the workspace folder [stm32_spine/](./CAN_MOTORFIRMWARE_like_MIT) inside STM32CubeIDE.
   * Flash the compiled firmware onto both low-level STM32F446RE microcontrollers.
2. **Connect the UP Board**:
   * Power on the UP Board (single-board x86 computer) and link it to the STM32 controllers using the SPI bus (`/dev/spidev2.0` and `/dev/spidev2.1`).
   * Remap pin `PA15` to alternate function mode (`SPI_NSS_HARD_INPUT`) in the microcontroller registers to enable automatic MISO line tri-stating and avoid bus collisions.
3. **Launch the Hardware Interface & Calibration GUI**:
   * Export the library paths on the UP Board:
     ```bash
     export LD_LIBRARY_PATH=/home/scotty/champ/lib/:$LD_LIBRARY_PATH
     ```
   * Enable the debug GUI mode inside [hw_interface.cpp](./Scotty_Champ/scotty_hw_interface/src/hw_interface.cpp) by uncommenting:
     ```cpp
     #define DEBUG_JOINT_CONTROL_GUI
     ```
   * Run the control loop on the UP Board:
     ```bash
     roslaunch scotty_controller scotty_main_controller.launch hardware_connected:=True
     ```
   * Launch the slider-based calibration GUI:
     ```bash
     roslaunch scotty_hw_interface ScottyJointControlTest.launch
     ```

---

### 3. Reproducing the MIT Standing Controller (Suspended Rig)
To run the adapted MIT standing controller trials:

1. **Build Cheetah-Software**:
   * Navigate to the Cheetah software package:
     ```bash
     cd HigherLevel_CheetahSoftware
     mkdir build && cd build
     cmake -DMINI_CHEETAH_BUILD=TRUE ..
     ./../scripts/make_types.sh
     make -j4
     ```
2. **Launch Controller**:
   * Connect to the UP Board over Ethernet, copy the compiled binaries, and execute the standing script:
     ```bash
     ./run_mc.sh
     ```
   * *Safety Warning: Due to the high peak torques of the CubeMars AK10-9 actuators (up to 48 Nm), this trial must only be run while the robot chassis is securely suspended in a safety frame. The lack of ground contact forces will cause joint overcorrections and base state-estimator drift, which are documented as platform-specific constraints in Section 4 of the manuscript.*

---

## Codebase Directory Structure

* **[Scotty_Champ/](./Scotty_Champ)**: Houses the CHAMP integration package, including gait parameters (`scotty_config`), custom state-machine and web dashboard controllers (`scotty_controller`), and the C++ SPI-to-CAN hardware bridge (`scotty_hw_interface`).
* **[HigherLevel_CheetahSoftware/](./HigherLevel_CheetahSoftware)**: The adapted MIT Mini Cheetah locomotion framework.
* **[CAN_MOTORFIRMWARE_like_MIT/](./CAN_MOTORFIRMWARE_like_MIT)**: Low-level C firmware for the STM32 controllers managing SPI communication loops and CAN actuator telemetry.
* **[Extras/](./Extras)**: Contains helper tools, SolidWorks URDF exports, custom log viewers, and precompiled UP Board library backups.

---

## Performance & Calibration Baselines

### Hardware Interface Diagnostics
* **SPI Loop Frequency**: $100$~Hz (10 ms cycle time).
* **Communication Stability**: Tested continuously up to $197,020$ packets ($\approx 32.8$ minutes) with a checksum error rate of **0.00%** (zero packet loss) after reconfiguring `SPI_NSS_HARD_INPUT`.
* **CAN bus configuration**: $1$~Mbit/s speed, with a deterministic $300$~$\mu$s inter-message delay.

### Joint-Level Tracking Calibration Errors ($\Delta q = |q_{\text{des}} - q_{\text{act}}|$)
* **Abad**: Mean error $0.1628$ – $0.2517$~rad (standard deviation $0.2835$~rad, reflecting mechanical mounting backlash).
* **Hip**: Mean error $0.0026$ – $0.0281$~rad (standard deviation $0.0735$~rad).
* **Knee**: Mean error $0.0105$ – $0.0976$~rad (standard deviation $0.1754$~rad).

---

## Historical Tasks and Developer Logs

The following sections contain the original notes, tasks, and commit histories from active development.

### Tasks To Do
Date 29-04-2025 Vishnu

Refer Chapter 7. Recommended Next Steps in "Analysis, Development and Integration of Control Frameworks for Scotty, a Custom Quadrupedal Robot" by Vishnudev Kurumbaparambil, 2025

**Other Improvements in CHAMP Controller GUI**
1. More Error Information to users like spi error, ros bridge error etc

**Other Improvements in Low Level Code**
1. Better definition of size and copying between struct and array

            #define TX_LEN (sizeof(spi_tx) / 2)  // Calculate correct size
            memcpy(spi_tx_buffer, data, sizeof(spi_tx));

2. For more readability than mere numbers

            #define for receivedCanBus like CAN1,2,3,4

3. CAN Reception in callback instead of in loop
4. Check CAN receive return success (HAL_CAN_GetRxMessage()).
5. Add timeout for CAN transmission.
6. Improve CAN Bus-Off recovery.
7. Understand proper way of doing softstop_joint().
8. Handle case of out of range value from Motor, unpack fn
9. No limit check for input from MIT, ie control
10. Also a way to handle NaN from MIT or from Motor

---

### Extras Sub-Components Detail
The `Extras/` folder contains the following utilities not detailed in the main guide:
* **Champ_UP_lib**: A set of libraries required to execute compiled ROS C++ binaries on the UP Board. These must be properly sourced (`LD_LIBRARY_PATH`) to avoid runtime errors.
* **MIT_LogViewer**: A custom log visualization tool developed to analyze command and state data generated during tests.
* **STM_Test**: A standalone test script designed to validate and debug the behavior of the low-level firmware independently.
* **URDF_Generation**: Contains raw URDF files generated from SolidWorks and the updated mechanical Assembly file.
* **GroundStation_TerminalHistory_Bckup.txt**: A backup file containing the terminal command history of the ground station system prior to the start of development.


---

### LOGS & Development History
29-04-2025
Current author is Vishnu. His thesis work is finished.

#### Old comments from Vishnu:
The combination of Dave's base and Nils main didnt work. The SPI data was not received. Could be due to the possibility that the DMA code is not present.

With the code Subash shared and Nils main, the SPI is working.
_______________________________________________

This is the base code for my development. It is using base from commit a654bc59d27731861910214b49cedf1fa6ac9bd0 and Nils main.c

It is build through, and SPI data is received. 
_______________________________________________

Creating git for the higher level code also.
Initial commit is having the MIT Mini cheetah repo https://github.com/mit-biomimetics/Cheetah-Software

_____________________________________________
Changes by Subash : 
robot-software-202302201609 : - Tried commenting out the SPI checksum check
                            - Trying to set up of IMU Init
                            - Playing with config 

robot-software-202302201627 : Wifi -> Ethernet

robot-software_noDelay_fullspeed-202302221454 : Missed to commit. Playing with spi speed/delays

robot-software_10usDelay-202302231425 : Trying with SPI speed and delay
robot-software_20usDelay-202302231436 : Trying with different SPI speed/delay
robot-software_15usdelay-202302231632 : Reseting to MIT orginal SPI checksum check and speed

robot-software_Final-202302241657 : SPI delay zero, sbus debug

robot-software_Final-202303171602 : Trying with different hip knee offset

robot-software_Final-202303241519 : Changed to K_MINI_CHEETAH_VECTOR_NAV_SERIAL "/dev/ttyS4"
robot-software_Final-202303241603 : K_SBUS_PORT_MC "/dev/ttyS0"

robot-software_Final-202303241619 : Trying different knee signs
robot-software_Final-202304111827 : Trying different abad, hip, knee offsets
robot-software_Final-202304121309 : again set abad, hip,knee offset to 0 
robot-software_Final-202410241435 : Reset offsets to mit
