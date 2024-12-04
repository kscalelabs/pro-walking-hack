#!/bin/bash

sudo modprobe mttcan
# Loop through CAN interfaces 0-3
for i in {0..2}; do
    echo "Setting up can${i}..."
    sudo ip link set can${i} down
    sudo ip link set can${i} type can bitrate 1000000
    sudo ip link set can${i} txqueuelen 1000
    sudo ip link set can${i} up
done

echo "All CAN interfaces configured!"
