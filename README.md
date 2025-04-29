
# SCOTTY

A custom quadrapedal robot.

## Tasks To Do

Date 29-04-2025 Vishnu

Refer Chapter 7. Recommended Next Steps in "Analysis, Development and Integration of Control Frameworks for Scotty, a Custom Quadrupedal Robot" by Vishnudev Kurumbaparambil, 2025

**Other Software Improvements**
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



## Contents

The repository is organized into the following main components:

- CAN_MOTORFIRMWARE_like_MIT     
    Contains the low-level controller code that runs on the STM32 microcontrollers, responsible for SPI and CAN-based motor communication.

- HigherLevel_CheetahSoftware    
    The high-level control framework based on the MIT Mini Cheetah system, adapted and modified for integration with Scotty.

- Scotty_Champ     
    The CHAMP-based high-level controller, tailored for Scotty’s architecture and supporting both simulation and real hardware modes.

- Extras
    - Champ_UP_lib     
        A set of libraries required to execute compiled ROS C++ binaries on the UP Board. These must be properly sourced to avoid runtime errors.
    - MIT_LogViewer             
        A custom log visualisation tool developed to analyse command and state data generated during tests.
    - STM_Test         
        A standalone test script designed to validate and debug the behaviour of the low-level firmware independently.
    - URDF_Generation        
        Contains URDF files generated from SolidWorks, updated Assembly file.
    - GroundStation_TerminalHistory_Bckup.txt         
        A backup file containing the terminal command history of the ground station system prior to the start of development, for future reference.


## LOGS

29-04-2025
Current author is Vishnu. His thesis work is finished.

### Old comments from Vishnu:

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



