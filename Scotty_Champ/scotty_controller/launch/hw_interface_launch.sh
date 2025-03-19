#!/bin/bash
xterm -hold -e "ssh scotty@192.168.2.40 'cd /home/scotty/champ/hw_interface_202503191317 && export ROS_MASTER_URI=http://192.168.2.31:11311 && export ROS_HOSTNAME=192.168.2.40 && export LD_LIBRARY_PATH=/home/scotty/champ/lib:$LD_LIBRARY_PATH && ./hw_interface' ; bash"
