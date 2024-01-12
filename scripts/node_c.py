#!/usr/bin/env python3

# Import necessary modules
import rospy
import math
from assignment_2_2023.msg import msg
from assignment_2_2023.srv import average, averageResponse

# Global variables for storing the average velocity and distance
average_velocity = 0
distance = 0

def calculate_distance(target_pos_x, target_pos_y, current_pos_x, current_pos_y):
    """
    Calculates the Euclidean distance between two points.

    Args:
        target_pos_x (float): Target position x-coordinate.
        target_pos_y (float): Target position y-coordinate.
        current_pos_x (float): Current position x-coordinate.
        current_pos_y (float): Current position y-coordinate.

    Returns:
        float: Calculated distance.
    """
    # Euclidean distance formula
    return math.sqrt((target_pos_x - current_pos_x) ** 2 + (target_pos_y - current_pos_y) ** 2)

def calculate_average_velocity(recent_velocity_readings, window_size):
    """
    Calculates the average velocity from recent readings.

    Args:
        recent_velocity_readings (list): List of recent velocity readings.
        window_size (int): The number of readings to consider for average calculation.

    Returns:
        float: Calculated average velocity.
    """
    # Calculate the average using a sliding window approach
    return sum(recent_velocity_readings) / min(len(recent_velocity_readings), window_size)

def update_global_values(calculated_distance, calculated_avg_velocity):
    """
    Updates the global variables for distance and average velocity.

    Args:
        calculated_distance (float): The calculated distance.
        calculated_avg_velocity (float): The calculated average velocity.
    """
    global distance, average_velocity
    distance = calculated_distance
    average_velocity = calculated_avg_velocity

def calculate_distance_and_average_velocity(msg):
    """
    Callback function for message processing.

    Args:
        msg (msg): The received message object.
    """
    # Retrieve target position from ROS parameters
    target_pos_x = rospy.get_param('/des_pos_x', 0)
    target_pos_y = rospy.get_param('/des_pos_y', 0)

    # Extract current position from the message
    current_pos_x = msg.positionx
    current_pos_y = msg.positiony

    # Calculate distance to the target
    calculated_distance = calculate_distance(target_pos_x, target_pos_y, current_pos_x, current_pos_y)

    # Get the window size for velocity averaging
    velocity_window_size = rospy.get_param('/window_size', 10)
    # Ensure the velocityx field is a list for processing
    recent_velocity_readings = msg.velocityx[-velocity_window_size:] if isinstance(msg.velocityx, list) else [msg.velocityx]
    # Calculate average velocity
    calculated_avg_velocity = calculate_average_velocity(recent_velocity_readings, velocity_window_size)

    # Update the global variables
    update_global_values(calculated_distance, calculated_avg_velocity)

def handle_service_request(_):
    """
    Handles requests to the average service.

    Returns:
        averageResponse: The response containing distance and average velocity.
    """
    return averageResponse(distance, average_velocity)

def initialize_service_and_subscriber():
    """
    Initializes the ROS service and subscriber.
    """
    # Initialize the service
    rospy.Service("info_service", average, handle_service_request)
    rospy.loginfo("Service 'info_service' is ready.")

    # Initialize the subscriber
    rospy.Subscriber("/pos_vel", msg, calculate_distance_and_average_velocity)

def main():
    """
    Main function to initialize the node and start the service.
    """
    rospy.init_node('info_service')
    initialize_service_and_subscriber()
    rospy.spin()

if __name__ == "__main__":
    main()
