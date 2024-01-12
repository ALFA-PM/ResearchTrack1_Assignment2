#! /usr/bin/env python3

# Import necessary libraries for ROS and message types
import rospy
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel, PlanningAction, PlanningGoal, PlanningResult
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

class GoalHandler:
    def __init__(self):
        # Publisher for robot's position and velocity
        self.pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)
        # Action client for sending goals
        self.client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
        # Wait until the action server is available
        self.client.wait_for_server()
        # Track if a goal is active
        self.goal_active = False

    def handle_goal_commands(self):
        # Main loop for handling commands
        rospy.Subscriber("/odom", Odometry, self.publish_position_velocity)
        
        while not rospy.is_shutdown():
            # User command for setting or cancelling goals
            command = input("Press 'y' to set a new goal, 'c' to cancel the current goal: ")
            
            if command == 'y':
                self.set_new_goal()
            elif command == 'c':
                self.cancel_goal()
            else:
                rospy.logwarn("Invalid command. Please enter 'y' or 'c'.")

    def set_new_goal(self):
        # Set a new goal based on user input
        try:
            input_x = float(input("Enter the x-coordinate for new goal: "))
            input_y = float(input("Enter the y-coordinate for new goal: "))
        except ValueError:
            rospy.logwarn("Invalid input. Please enter a valid number.")
            return

        # Update the target position and send the goal
        goal = assignment_2_2023.msg.PlanningGoal()
        goal.target_pose.pose.position.x = input_x
        goal.target_pose.pose.position.y = input_y
        self.client.send_goal(goal)
        self.goal_active = True
        rospy.loginfo(f"New goal set: target_x = {input_x}, target_y = {input_y}")

    def cancel_goal(self):
        # Cancel the current active goal
        if self.goal_active:
            self.client.cancel_goal()
            rospy.loginfo("Current goal has been cancelled")
            self.goal_active = False
        else:
            rospy.loginfo("No active goal to cancel")

    def publish_position_velocity(self, msg):
        # Publish the current position and velocity of the robot
        pos_and_vel = Vel()
        pos_and_vel.pos_x = msg.pose.pose.position.x
        pos_and_vel.pos_y = msg.pose.pose.position.y
        pos_and_vel.vel_x = msg.twist.twist.linear.x
        pos_and_vel.vel_z = msg.twist.twist.angular.z
        self.pub.publish(pos_and_vel)

def main():
    # Initialize the ROS node and start the GoalHandler
    rospy.init_node('set_target_client')
    GoalHandler().handle_goal_commands()

if __name__ == '__main__':
    main()
