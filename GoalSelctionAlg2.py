#!/usr/bin/env python
# license removed for brevity

# rospy is the ROS Python client library.
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import Float32MultiArray

class MoveBaseClient:
    def __init__(self):
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py', anonymous=True)

        # Define instance attribute meanMatrix (for good practise. It is best to initalise
        # instanc attributes within the __init__ as opposed to the mean_callback method.
        self.meanMatrix = None

        # Subscribes to Mean DATA.
        rospy.Subscriber("/gabp/mean/matrix", Float32MultiArray, self.mean_callback)
        
        # Create an act-ion client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()
        
    def move(self, x, y):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        # The pose is relative to the odom frame, and the note this is what the carographer map and i guess octomap
        # or whatever will be as well.
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Move to the specified coordinates
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.w = 1.0
        
        # Sends the goal to the action server.
        self.client.send_goal(goal)
        
        # Waits for the server to finish performing the action.
        wait = self.client.wait_for_result()
        
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return self.client.get_result()

    def mean_callback(self, meanMatrix):
        self.meanMatrix = meanMatrix


if __name__ == '__main__':
    while
    try:
        move_base = MoveBaseClient()
        result = move_base.move(1.0, 2.0)  # Move to (1.0, 2.0)
        if result:
            rospy.loginfo("Goal execution done!")
        # The rospy.spin() function is called in the try block after the call to move() to ensure that the node does
        # not exit immediately.
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
