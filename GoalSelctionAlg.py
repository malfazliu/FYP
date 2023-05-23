#!/usr/bin/env python
# license removed for brevity

# rospy is the ROS Python client library.
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# I need to import the OGM.
# I need to origin of map
# OccupancyGrid is a message type in the nav_msgs package that represents a 2D map with an occupancy probability
# for each cell.
from nav_msgs.msg import OccupancyGrid



# Note:
# Why are there no subscribers or publishers in my code. Should the goal not be published?
# In your code, there are no explicit publishers or subscribers, but the move_base action server
# and client are using the ROS communication infrastructure to exchange messages.
# The move_base action server is listening to the move_base/goal topic for new goals and sends feedback
# and result messages to the move_base/feedback and move_base/result topics, respectively.
# The move_base action client, on the other hand, sends goal messages to the move_base/goal topic and
# listens to the move_base/feedback and move_base/result topics for feedback and result messages.
# When you call client.send_goal(goal) in the movebase_client function, the move_base action client
# publishes a new goal message to the move_base/goal topic, which is then picked up by the move_base
# action server for execution.
# Therefore, even though there are no explicit publishers or subscribers in your code,
# the move_base action client and server are still using the ROS communication infrastructure
# to exchange messages.




def movebase_client(x,y):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = x
    # Move 0.5 meters forward along the y axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = y
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   
   # The function wait_for_result() is used to wait for
   # the server to finish performing the action. This function blocks the code execution until
   # either the result of the action is received or the connection to the server is lost.
   # By default, wait_for_result() does not specify a timeout value, which means it waits indefinitely
   # until the action server responds. This is useful when the action server is expected to return a
   # result eventually.
   # Waits for the server to finish performing the action. Note:
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# [If the python node is executed as main process (sourced directly) ]
# The if __name__ == '__main__': statement is a common Python idiom that allows a file to be both
# used as a standalone program and imported as a module.
# When a Python file is executed directly as a script, the special __name__ variable is set to '__main__'.
# On the other hand, when a Python file is imported as a module, the __name__ variable is set to the name
# of the module.
# The purpose of the if __name__ == '__main__': statement is to ensure that the code inside the block
# is executed only when the file is executed directly as a script, and not when the file is imported
# as a module. This is because when a file is imported as a module, you may not want the module's code
# to be executed automatically.
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client(x,y)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
