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


