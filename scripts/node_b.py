#!/usr/bin/env python3

import rospy
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Input, InputResponse

# Define a class for the service
class LastTargetService:
    def __init__(self):
        # Initialize class variables for the last desired x and y positions
        self.last_des_x = 0
        self.last_des_y = 0

        # Initialize the node with the name 'last_target_service'
        rospy.init_node('last_target_service')
        rospy.loginfo("Last target node initialized")

        # Provide a service named 'input', using the custom service type Input
        rospy.Service('input', Input, self.result_callback)

    # Callback function for the service
    def result_callback(self, _):
        # Create a response message
        response = InputResponse()
        # Set the x and y inputs in the response to the last desired positions
        self.last_des_x = rospy.get_param('/des_pos_x')
        self.last_des_y = rospy.get_param('/des_pos_y')
        response.input_x = self.last_des_x
        response.input_y = self.last_des_y

        # Return the response
        return response

    # Function to keep the node running
    def spin(self):
        rospy.spin()

# Main function
if __name__ == "__main__":
    # Create an instance of the service class
    service = LastTargetService()
    # Start the node
    service.spin()