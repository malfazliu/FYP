#!/usr/bin/env python
# license removed for brevity

# rospy is the ROS Python client library.
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# from std_msgs.msg import Float32MultiArray
import time
class MoveBaseClient:
    def __init__(self):
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py', anonymous=True)
        
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
        wait = self.client.wait_for_result(rospy.Duration(10))
        # If the result doesn't arrive, assume the Server is not available.
        if not wait:
            # rospy.logerr("Action server not available!")
            # rospy.signal_shutdown("Action server not available!")
            rospy.loginfo("Goal NOT reached in time!")
        else:
            rospy.loginfo("Goal reached in time!")
        return self.client.get_result()

# if __name__ == '__main__':

#    try:
#        move_base = MoveBaseClient()
        # Now calling the function to move the robot. This function call finishes only when  'wait' variable finishes
#        result = move_base.move(1.0, 2.0)  # Move to (1.0, 2.0)
#        # And when this happens, we can print:
#        if result:
#            rospy.loginfo("Goal execution done!")
#        # The rospy.spin() function is called in the try block after the call to move() to ensure that the node does
#        # not exit immediately.
#        rospy.spin()
#    except rospy.ROSInterruptException:
#        rospy.loginfo("Navigation test finished.")


if __name__ == '__main__':

    try:
        # Creating an instance of the class 
        move_base = MoveBaseClient()
        while not rospy.is_shutdown():
            # input goal position

            for index in range(0, 19):
                goal_x = 4.0 + index
                goal_y = 1.0 - index
                # If there is a goal set
                if goal_x is not None and goal_y is not None:
                    print("There is a goal ", str(goal_x), str(goal_y))
                    # then run the method, the method finishes when the goal has been reached.
                    result = move_base.move(goal_x, goal_y)
                    # Hence if it has been reached, print "Goal execution is done" and set goal coordinates
                    # to zero.
                    if result:
                        # Reset goal coordinates after completing the move
                        move_base.goal_x = None
                        move_base.goal_y = None
                        rospy.sleep(0.1)  # Wait between loop iterations
                else:
                    print("Waiting for Goal to be set")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


# Note: 
# The rospy.init_node() call and self.client assignment are not set as instance variables
# because they are not meant to be accessed or modified outside of the __init__() method of
# the MoveBaseClient class. It is however asn instance attribute.
# rospy.init_node() initializes a new ROS node and is typically called once per Python
# script or module. It does not need to be saved as a variable because the node is automatically
# registered with the ROS Master and will continue to run until the script or module is terminated.
# Similarly, self.client is assigned as an instance attribute of the MoveBaseClient class so that
# it can be used in the move() method to send goals to the move_base action server. It does not
# need to be saved as a separate variable because it is already stored as an attribute of the
# MoveBaseClient instance.
