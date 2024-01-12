#!/usr/bin/env python3

# Importing necessary ROS libraries
import rospy
from assignment_2_2023.srv import Input, InputResponse

def read_target_positions():
    """
    Reads the last target positions from ROS parameters.

    Returns:
        tuple: A tuple containing the x and y coordinates of the last target position.
    """
    # Retrieve the last target positions from ROS parameters, default to 0 if not set
    last_target_pos_x = rospy.get_param('/des_pos_x', 0)
    last_target_pos_y = rospy.get_param('/des_pos_y', 0)
    return last_target_pos_x, last_target_pos_y

def create_service_response(last_target_pos_x, last_target_pos_y):
    """
    Creates a service response message with the last target positions.

    Args:
        last_target_pos_x (float): The last target x-coordinate.
        last_target_pos_y (float): The last target y-coordinate.

    Returns:
        InputResponse: The service response message.
    """
    # Create an InputResponse message and assign the target positions
    response = InputResponse()
    response.inputx = last_target_pos_x
    response.inputy = last_target_pos_y
    return response

def handle_service_request(_):
    """
    Handles incoming service requests.

    Args:
        _: Unused parameter for service request data.

    Returns:
        InputResponse: The service response message.
    """
    # Read the last target positions and create a response
    last_target_pos_x, last_target_pos_y = read_target_positions()
    return create_service_response(last_target_pos_x, last_target_pos_y)

def initialize_service():
    """
    Initializes and starts the ROS service.
    """
    # Define the 'input' service with the Input service type and request handler
    rospy.Service('input', Input, handle_service_request)
    rospy.loginfo("Service 'input' is ready to provide last target positions.")

def main():
    """
    Main function to initialize the node and start the service.
    """
    # Initialize the ROS node
    rospy.init_node('last_target_service')
    # Initialize and run the service
    initialize_service()
    # Keep the node running
    rospy.spin()

if __name__ == "__main__":
    main()
