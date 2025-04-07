#!/bin/bash
# This script creates a cgroup for CPU and memory, limits resources, and runs local_main.
# It is configured to mimic a t3.large instance’s memory limit of 16 GiB.

# Define paths for the cgroup controllers (assuming cgroup v1 is in use)
CGROUP_CPU="/sys/fs/cgroup/cpu/boids_cgroup"
CGROUP_MEM="/sys/fs/cgroup/memory/boids_cgroup"

# Create the cgroup directories if they do not already exist.
if [ ! -d "$CGROUP_CPU" ]; then
    sudo mkdir "$CGROUP_CPU"
fi

if [ ! -d "$CGROUP_MEM" ]; then
    sudo mkdir "$CGROUP_MEM"
fi

# Set the memory limit:
# 16 GiB in bytes is 17179869184.
echo 17179869184 | sudo tee "$CGROUP_MEM/memory.limit_in_bytes"

# Change directory to the build folder where local_main resides.
cd build || { echo "Build directory not found"; exit 1; }

# Run the local_main executable inside the cgroup.
sudo cgexec -g cpu,memory:boids_cgroup ./local_main
