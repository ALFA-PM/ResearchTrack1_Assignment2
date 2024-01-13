# Import necessary libraries
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

# Class definition for handling goal setting and cancellation in a ROS environment
class GoalHandler:
    def __init__(self):
        # Initialize ROS publisher and action client for goal handling
        self.pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)
        self.client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
        self.client.wait_for_server()
        self.goal_cancelled = True  # Flag to keep track of goal status

    # Method to handle user commands for setting or cancelling goals
    def handle_command(self, command, goal):
        if command == 'y':
            self.set_new_goal(goal)
        elif command == 'c':
            self.cancel_goal()
        else:
            rospy.logwarn("Invalid command. Please enter 'y' or 'c'.")

    # Method to set a new goal based on user input
    def set_new_goal(self, goal):
        try:
            input_x, input_y = self.get_new_coordinates()
        except ValueError:
            rospy.logwarn("Invalid input. Please enter a valid number.")
            return

        self.update_goal_parameters(input_x, input_y, goal)
        self.send_new_goal(goal)

    # Method to get new goal coordinates from the user
    def get_new_coordinates(self):
        input_x = float(input("Enter the x-coordinate for the new goal: "))
        input_y = float(input("Enter the y-coordinate for the new goal: "))
        return input_x, input_y

    # Method to update goal parameters
    def update_goal_parameters(self, input_x, input_y, goal):
        rospy.set_param('/des_pos_x', input_x)
        rospy.set_param('/des_pos_y', input_y)
        goal.target_pose.pose.position.x = input_x
        goal.target_pose.pose.position.y = input_y

    # Method to send the updated goal to the action server
    def send_new_goal(self, goal):
        self.client.send_goal(goal)
        self.goal_cancelled = False

    # Method to cancel the current goal
    def cancel_goal(self):
        if not self.goal_cancelled:
            self.client.cancel_goal()
            rospy.loginfo("Current goal has been cancelled")
            self.goal_cancelled = True
        else:
            rospy.loginfo("No active goal to cancel")

    # Method to continuously handle goal commands until ROS is shut down
    def handle_goal_commands(self):
        while not rospy.is_shutdown():
            # Subscribe to /odom topic and publish position and velocity
            rospy.Subscriber("/odom", Odometry, self.publish_position_velocity)
            # Get user command
            command = input("Press 'y' to set a new goal or 'c' to cancel the current goal: ")
            # Get current target position
            target_pos_x = rospy.get_param('/des_pos_x')
            target_pos_y = rospy.get_param('/des_pos_y')

            # Create a new goal with the current target position
            goal = assignment_2_2023.msg.PlanningGoal()
            goal.target_pose.pose.position.x = target_pos_x
            goal.target_pose.pose.position.y = target_pos_y
            rospy.loginfo("Current goal: target_x = %f, target_y = %f", target_pos_x, target_pos_y)
            self.handle_command(command, goal)

            rospy.loginfo("Last received goal: target_x = %f, target_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

    # Method to publish the current position and velocity based on Odometry data
    def publish_position_velocity(self, msg):
        position_and_velocity = Vel()
        position_and_velocity.pos_x = msg.pose.pose.position.x
        position_and_velocity.pos_y = msg.pose.pose.position.y
        position_and_velocity.vel_x = msg.twist.twist.linear.x
        position_and_velocity.vel_z = msg.twist.twist.angular.z

        self.pub.publish(position_and_velocity)

# Main function to initialize the ROS node and start the goal handler
def main():
    rospy.init_node('set_target_client')
    handler = GoalHandler()
    handler.handle_goal_commands()

# Entry point of the script
if __name__ == '__main__':
    main()
