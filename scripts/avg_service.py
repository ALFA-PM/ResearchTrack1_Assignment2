#! /usr/bin/env python3

"""
.. module: avg_service

	:platform: Unix
	:synopsis: action_client
.. moduleauthor:: Mahnaz Mohammad Karimi

This service node subscribes to the robot's position and velocity data, utilizing a custom message type called Info.
Additionally, it implements a server to calculate and provide two pieces of information: the distance of the robot from the target and the robot's average speed.

Subscribes to:
	/robot_pos_vel
"""

import rospy
import math
from assignment_2_2023.srv import Avg_dist_vel, Avg_dist_velResponse
from assignment_2_2023.msg import Info


# Global variables to store mean velocities and distance
mean_vel_x = 0
mean_vel_z = 0
dist = 0


def clbk_info(msg):
    """
    Callback function receives messages containing information about the robot's position and velocity. This includes the robot's position in the x and y coordinates, as well as its linear velocity in the x direction and angular velocity around the z-axis.
    When Callback function receives a message, it extracts these pieces and saves them into a custom message format called Info().
    This custom message is designed to hold the position (x, y) and velocity (linear velocity x, angular velocity z) of the robot.
    After storing the information in the custom message, the Callback function publishes it on a specific topic called /robot_pos_vel.
    """
    
    global mean_vel_x, mean_vel_z, dist
    # Lists to store linear and angular velocities
    linear_vel_x = []
    angular_vel_z = []
    # Retrieve window size parameter from ROS parameter server
    window_size = rospy.get_param("window_size")
    # Positions and Velocities
    x_robot = msg.x
    y_robot = msg.y
    linear_vel_x.append(msg.vel_x)
    angular_vel_z.append(msg.vel_z)
    # Get the target coordinates
    x_target = rospy.get_param("des_pos_x")
    y_target = rospy.get_param("des_pos_y")
    # Calculate the average linear and angular velocities
    instant_vel_x = linear_vel_x[-window_size:]
    instant_vel_z = angular_vel_z[-window_size:]
    mean_vel_x = sum(instant_vel_x) / len(instant_vel_x)
    mean_vel_z = sum(instant_vel_z) / len(instant_vel_z)
    # Calculate the distance between the target and the robot
    dist = math.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2)


def clbk_avg(request):
    """
    Callback to return the:
    distance between target and robot,
    average of the linear velocity along x axis,
    average of the angular velocity around z axis
    to the service.
    :param request: parameter representing the request sent to the ROS service. When a client node calls the last_target service, it sends a request to the service, and this request is received as a parameter in the clbk_service function.
    """
    
    global mean_vel_x, mean_vel_z, dist
    # Round values for better readability
    dist = round(dist, 3)
    mean_vel_x = round(mean_vel_x, 3)
    mean_vel_z = round(mean_vel_z, 3)
    # Print the calculated values
    print(f"Distance between the robot and the target is: {dist} m")
    print(f"Average of linear velocity along Robot x-axis is: {mean_vel_x} m/s")
    print(f"Average of angular velocity around Robot z-axis is: {mean_vel_z} rad/s")
    print("----------------------------------------------------------------")
    # Return the response with the calculated values
    return Avg_dist_velResponse(dist, mean_vel_x, mean_vel_z)


def main():
    """
    Main function in which the ros node is initialized and the publisher and subscriber are initialized.
    """
    # Initialize the ROS node
    rospy.init_node("avg_dist_vel")
    # Create a ROS service server
    rospy.Service("avg_dist_vel", Avg_dist_vel, clbk_avg)
    # Create a ROS subscriber to listen to the /robot_pos_vel topic
    rospy.Subscriber('/robot_pos_vel', Info, clbk_info)
    # Keep the node running
    rospy.spin()


if __name__ == "__main__":
    main()
