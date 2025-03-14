#!/bin/bash
set -e  # Exit on error

# Define Source Files
SRC_CPP="/home/roboubu/Documents/VishnuScooty/scotty_ws/src/scotty_hw_interface/src/hw_interface.cpp"
SRC_EXEC="/home/roboubu/Documents/VishnuScooty/scotty_ws/devel/.private/scotty_hw_interface/lib/scotty_hw_interface/hw_interface"

# Define UP Board details
UP_USER="scotty"
UP_IP="192.168.2.40"

# Create a timestamped directory on UP Board
DATE=$(date +"%Y%m%d%H%M")
UP_DIR="/home/scotty/champ/hw_interface_$DATE"

# Create a local temporary folder
TEMP_DIR="/tmp/hw_interface_$DATE"
mkdir -p ${TEMP_DIR}

# Copy files to the local temp directory
cp ${SRC_CPP} ${TEMP_DIR}/
cp ${SRC_EXEC} ${TEMP_DIR}/

# Copy the entire directory to UP Board (asks for password **only once**)
scp -r ${TEMP_DIR} ${UP_USER}@${UP_IP}:${UP_DIR}

# Cleanup local temporary directory
rm -rf ${TEMP_DIR}

echo "✅ Files copied successfully to UP Board: ${UP_DIR}"


