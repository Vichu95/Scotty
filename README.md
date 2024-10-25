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

