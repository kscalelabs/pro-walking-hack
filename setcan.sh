#!/bin/bash

sudo modprobe mttcan

echo "Setting up IMU on can0..."
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000

# Loop through CAN interfaces 1-2
for i in {1..2}; do
    echo "Setting up can${i}..."
    sudo ip link set can${i} down
    sudo ip link set can${i} type can bitrate 1000000
    sudo ip link set can${i} txqueuelen 10000
    sudo ip link set can${i} up
done

echo "All CAN interfaces configured!"
