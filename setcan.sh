#!/bin/bash

# First, bring everything down
sudo ip link set can0 down 2>/dev/null
sudo ip link set can1 down 2>/dev/null
sudo ip link set can2 down 2>/dev/null

# Remove modules in reverse order
sudo rmmod gs_usb 2>/dev/null
sudo rmmod mttcan 2>/dev/null
sudo rmmod can_raw 2>/dev/null
sudo rmmod can 2>/dev/null

# Load base CAN modules
sudo modprobe can
sudo modprobe can_raw

# Load gs_usb first
sudo modprobe gs_usb
sleep 0.1

# Now load mttcan
sudo modprobe mttcan
sleep 0.1

# Set up can0 (should be gs_usb)
echo "Setting up can0 for IMU..."
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Set up can1 and can2 (should be mttcan)
echo "Setting up can1..."
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 txqueuelen 1000
sudo ip link set can1 up

echo "Setting up can2..."
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 txqueuelen 1000
sudo ip link set can2 up

echo "All CAN interfaces configured!"
