#!/bin/bash

cd ..
cd ..

# Check if the current directory contains a Catkin workspace
if [ ! -f "src/CMakeLists.txt" ]; then
    echo "This script must be run from the root of a Catkin workspace"
    exit 1
fi

# Build the ROS workspace
catkin_make || { echo "Catkin make failed"; exit 1; }

# Source the setup file
source devel/setup.bash

# Start roscore in the background and get its PID
roscore &
ROSCORE_PID=$!

# Check if roscore is up and running
until rostopic list ; do
  echo "Waiting for roscore to start..."
  sleep 1
done

# Check if xterm is installed
if ! command -v xterm &> /dev/null
then
    echo "xterm could not be found, please install it or use another terminal emulator"
    kill $ROSCORE_PID
    exit 1
fi

# Open a new terminal using xterm and launch the ROS node
xterm -e "source devel/setup.bash && roslaunch assignment_2_2023 assignment1.launch" &
