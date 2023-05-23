#!/usr/bin/env python
# license removed for brevity

# rospy is the ROS Python client library.
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import Float32MultiArray

import numpy as np


class MoveBaseClient:
    def __init__(self):
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py', anonymous=True)
        
        # Create an act-ion client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()

        # Subbing to Goals
        rospy.Subscriber("/goal_selected", Float32MultiArray, self.goal_callback)

        self.chosen_goal_info = None
        self.chosen_goal_info_msg = None

    def goal_callback(self, msg):
        self.chosen_goal_info_msg = msg
        rospy.loginfo("Goal Subscribed")

    def goal_move_base_process(self):
        if self.chosen_goal_info_msg is None:
            return rospy.logwarn('No goal data received on occ_process goal_move_base_process method')
        else:
            self.chosen_goal_info = np.array(self.chosen_goal_info_msg.data).reshape((1, 5))
            self.move(self.chosen_goal_info[0][1], self.chosen_goal_info[0][2])

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
        wait = self.client.wait_for_result(rospy.Duration(30))
        
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            # rospy.logerr("Action server not available!")
            # rospy.signal_shutdown("Action server not available!")
            rospy.loginfo("Goal NOT reached in time!")
        else:
            rospy.loginfo("Goal reached in time!")
        return self.client.get_result()


if __name__ == '__main__':

    try:
        Instanceee = MoveBaseClient()
        # COMMENT NEXT LINE FOR NORMAL GOAL SUBSCRIOTION
        # data_waypoints = np.genfromtxt('waypointsDATA.csv', delimiter=',')
        while not rospy.is_shutdown():
            # COMMENT FOR LOOP OUT FOR NORMAL GOAL SUBSCRIPTION
            # for index in range(data_waypoints.shape[0]):
            #     move_method_call = Instanceee.move(data_waypoints[index][0], data_waypoints[index][1])
            #     rospy.sleep(0.1)
            # UNCOMMNET BOTTOM TWO LINES FOR NORMAL GOAL SUBSCRIPTION
            goal_move_base_process_method_call = Instanceee.goal_move_base_process()
            rospy.sleep(0.5)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("MoveBaseInteractor finished.")
