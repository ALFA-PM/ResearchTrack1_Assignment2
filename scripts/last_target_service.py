#! /usr/bin/env python3

"""
.. module: last_target_service

	:platform: Unix
	:synopsis: action_client
.. moduleauthor:: Mahnaz Mohammad Karimi

This service node, when invoked, provides the coordinates of the most recent target sent by the user

Subscribes to:
	/reaching_goal/goal
"""

import rospy
import assignment_2_2023.msg
from assignment_2_2023.srv import Last_target, Last_targetResponse

# Global variables to store last target coordinates
last_target_x = 0
last_target_y = 0

def clbk_service(request):
    """
    Callback function to return the last coordinates to the service.
    """
    global last_target_x, last_target_y
    print("Last target x coordinate is:", last_target_x)
    print("Last target y coordinate is:", last_target_y)
    print("---------------------------------")
    return Last_targetResponse(last_target_x, last_target_y)

def clbk_goal(msg):
    """
    Callback function to update the last target coordinates.
    """
    global last_target_x, last_target_y
    # Retrieve coordinates of the Goal
    last_target_x = msg.goal.target_pose.pose.position.x
    last_target_y = msg.goal.target_pose.pose.position.y

def main():
    """
    This is the main function where the ROS node is initialized, and both the service server and subscriber are set up.
    """
    rospy.init_node("last_target")
    
    # Create ROS service server
    rospy.Service("last_target", Last_target, clbk_service)
    
    # Create ROS subscriber
    rospy.Subscriber('/reaching_goal/goal', assignment_2_2023.msg.PlanningActionGoal, clbk_goal)
    
    rospy.spin()

if __name__ == "__main__":
    main()
