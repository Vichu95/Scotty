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

