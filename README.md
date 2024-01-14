# ROS Nodes for Robotic Control and Monitoring

This repository contains three ROS (Robot Operating System) nodes written in Python, designed for robotic control and monitoring purposes within a ROS-based robotic system.

## Contents
1. [node_a.py: Goal Handler](#node_apy-goal-handler)
2. [node_b.py: Velocity Controller](#node_bpy-velocity-controller)
3. [node_c.py: Distance and Velocity Monitor](#node_cpy-distance-and-velocity-monitor)

## node_a.py: Goal Handler

### Purpose
Handles goals for a robot in a ROS environment, interacting with an action server to manage goals based on user input.

### Key Features
- Initializes ROS publisher and action client.
- Setting and cancelling of goals via user commands.
- Utilizes custom ROS messages and services.

### Usage
- Run in a ROS environment.
- Input 'y' to set a new goal, 'c' to cancel the current goal.
- Follow prompts for goal coordinates.

## node_b.py: Velocity Controller

### Purpose
Acts as a velocity controller, adjusting robot's velocity based on current position and desired goal.

### Key Features
- Subscribes to position and goal topics.
- Calculates and adjusts velocities.
- Publishes velocity commands to a topic.

### Usage
- Requires active position and goal topics.
- Run in ROS environment.
- Automatically adjusts velocities based on data.

## node_c.py: Distance and Velocity Monitor

### Purpose
Monitors distance from a set goal and average velocity, providing information via a ROS service.

### Key Features
- Monitors position and velocity data.
- Calculates distance from desired position and average velocity.
- Provides metrics through ROS service.

### Usage
- Run in ROS environment.
- Request information using the ROS service.

## General Requirements
- ROS (Robot Operating System) installation.
- Custom ROS messages and services.
- ROS environment setup.

## Installation
- Clone to ROS workspace src folder.
- Build with `catkin_make`.

## Running the Nodes
You have to open your terminal inside the scripts folder and run these commands bellow to make your programms executable:

`chmod +x node_a.py`
`chmod +x node_b.py`
`chmod +x node_c.py`
`chmod +x bug_as.py`
`chmod +x go_to_point_service.py`
`chmod +x  wall_follow_service.py`

if any error met during this process you can run the command `sudo apt-get install xterm` before you run them again, this command will install xterm for you.

open a new terminal in the directory where your launch.sh program is located and use these two commands to launch the hole assignment 
`chmod +x launch.sh`
`./launch.sh`

1. Source ROS environment: `source devel/setup.bash`
2. Run nodes: `rosrun [package_name] [node_name.py]`

## Contact
For any queries, reach out to [mahnazmohammadkarimi@gmail.com].
