#! /usr/bin/env python3

"""
.. module: action_client

	:platform: Unix
	:synopsis: action_client
.. moduleauthor:: Mahnaz Mohammad Karimi

This node allows us to set a Goal or to cancel the Goal.

Subscribes to:
	/odom

Publishes to:
	/robot_pos_vel
"""

import time
import rospy
import select
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import Info
from geometry_msgs.msg import Point, Pose, Twist


def clbk_odom(msg):
    """

    Callback function receives messages containing information about the robot's position and velocity. This includes the robot's position in the x and y coordinates, as well as its linear velocity in the x direction and angular velocity around the z-axis.
    When Callback function receives a message, it extracts these pieces and saves them into a custom message format called Info().
    This custom message is designed to hold the position (x, y) and velocity (linear velocity x, angular velocity z) of the robot.

    After storing the information in the custom message, the Callback function publishes it on a specific topic called /robot_pos_vel.

    """

    new_info = Info()
    new_info.x = msg.pose.pose.position.x           # Position of the x coordinate
    new_info.y = msg.pose.pose.position.y           # Position of the y coordinate
    new_info.vel_x = msg.twist.twist.linear.x       # linear velocity, x axis
    new_info.vel_z = msg.twist.twist.angular.z      # angular velocity, z axis

    pub.publish(new_info)                           # Pubblish new message on /robot_pos_vel topic 


def clbk_feedback(feedback):
    """
    Callback function processes feedback received from a client.
    This feedback typically includes messages indicating whether the target has been reached successfully or if it has been cancelled.

    When Callback function receives feedback, it checks the content of the feedback message.
    If the message indicates that the target has been reached, it processes it accordingly, possibly performing some actions or updating internal states.
    Similarly, if the message indicates that the target has been cancelled, it handles this feedback appropriately.
    """
    
    if feedback.stat == "Target reached!":
        print(feedback)
        print("Press 'Enter' to set a new goal")
    if feedback.stat == "Target cancelled!":
        print(feedback)
        
def action():
    """
    Action function deals with the coordinates provided by the user as a goal for the robot to reach.
    Action function takes these coordinates and processes them and sends them as a goal.

    While the robot is in motion towards the goal, we can cancel this goal by clicking "c".
    """

    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    client.wait_for_server()


    while not rospy.is_shutdown():
        time.sleep(0.5)
        print("Set the goal coordinates")           # Get the coordinates from the user

        try:
            x = float(input("Enter x coordinates: "))
            y = float(input("Enter y coordinates: "))
            if -9 <= x <= 9 and -9 <= y <= 9:       # The coordinates should be in range -9 to 9
                print(f"Coordinates seted: (x={x}, y={y})")             # Print coordinates sets
            else:
                print("Invalid input. Please enter x and y coordinates in range -9 to 9.")
                continue
        except ValueError:
            print("Invalid input. Please try again")
            continue

        goal = assignment_2_2023.msg.PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        client.send_goal(goal, None, None, clbk_feedback)

        # The robot reaches the Goal. If wanna cancel Robot goal reading, just need to click "c" to cancel it.
        while not client.get_result():
            print("Press 'c' to cancel the goal!")
            cancel = select.select([sys.stdin], [], [], 0.1)
            if cancel:
                user_input = sys.stdin.readline().strip()
                if user_input == 'c':
                    client.cancel_goal()
                    break
			
			
def main():
    """
    This is the main function where the ROS node gets started, and both the publisher and subscriber are set up.

    """

    global pub, sub

    rospy.init_node('action_client')
    """
    Start the ROS node with the name 'action_client'.

    """

    pub = rospy.Publisher('/robot_pos_vel', Info, queue_size=10) 
    """
    Create a ROS publisher that will send position and velocity information of the robot to the /robot_pos_vel

    """

    sub = rospy.Subscriber('/odom', Odometry, clbk_odom)
    """
    Create a ROS subscriber that listens to messages from the /odom topic and uses the clbk_odom callback function to process them.

    """

    action()

if __name__ == "__main__":
	main()
