#!/bin/bash

# This script launches the UR5 simulation in Gazebo and the MoveIt planning environment.

# Function to kill background jobs on exit
cleanup() {
    echo "Shutting down background jobs..."
    kill $(jobs -p)
}
trap cleanup EXIT

echo "Sourcing ROS 2 and workspace..."

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Source your workspace
source install/setup.bash

# --- Terminal 1 Command ---
echo "Launching Gazebo Simulation in the background..."
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5 &

# --- Wait for Gazebo to load ---
echo "Waiting 15 seconds for Gazebo to initialize..."
sleep 15

# --- Terminal 2 Command ---
echo "Launching MoveIt and RViz..."
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 use_sim_time:=true launch_rviz:=true
