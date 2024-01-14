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
1. Source ROS environment: `source devel/setup.bash`
2. Run nodes: `rosrun [package_name] [node_name.py]`

## License
[MIT License](LICENSE)

## Contributing
Contributions, issues, and feature requests are welcome. Please refer to [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## Authors & Acknowledgment
- [Your Name](https://github.com/yourprofile)
- Thanks to contributors and ROS community.

## Contact
For any queries, reach out to [email@example.com](mailto:email@example.com).
