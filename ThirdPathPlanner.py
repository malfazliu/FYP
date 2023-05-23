#!/usr/bin/python3.6

# First simple STRUCTURE setup:
# TODO 1) Import occupancy grid - for an updating one, i assume this will be done in a while loop
# TODO 2) Figure out a way to map the binary grid to x,y cords of the map (I need the same cord system)
# TODO 3) Implement A* using sourced code and set start and end points
# TODO 4) Implement feedback to ROS husky import rospy


# rospy is the ROS Python client library.
# OccupancyGrid is a message type in the nav_msgs package that represents a 2D map with an occupancy probability
# for each cell.
# GetMap is a service type in the nav_msgs package that retrieves the map from the map server.
# PoseStamped is a message type in the geometry_msgs package that represents a pose in 2D space.
# Path is a message type in the nav_msgs package that represents a sequence of poses.
# numpy is a numerical computing library for Python.
# heapq is a library in the Python standard library that implements a priority queue using a heap.


import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import heapq

# Create a class PathPlanner that implements the path planning algorithm:
class PathPlanner:

    # The __init__ method initializes the path planner. It creates a node in the ROS system, retrieves the map from the
    # map server, creates a 2D numpy array from the map data, initializes the start and goal positions, creates a
    # publisher for the path, and subscribes to the goal pose.
    def __init__(self):
        # Initialize the node
        # Note: Yes, you can define this line outside of the __init__ method. In fact, it is common practice to
        # initialize the ROS node at the top of the script, before any other ROS-related operations are performed.
        # If you define the rospy.init_node("path_planner") call outside of the __init__ method and before the class
        # definition, it would have the same effect as if it were inside the __init__ method. The ROS node would still
        # be initialized with the name "path_planner".
        # However, it is generally recommended to keep the node initialization inside the class constructor, as this
        # makes it clear that the node is being initialized as part of the object creation process. This is also useful
        # if you have multiple objects in your code that need to interact with ROS, as each object can initialize its
        # own node with a unique name.
        # Also note the notation: rospy is a module which we imported and contains a bunch of variables and fucntions,
        # if you want to call a specific function you write .function_name() after rospy. The function will have some
        # return values defined within it
        rospy.init_node("third_path_planner", anonymous=True)

        # Subscribe to the occupancy grid
        # In the original script, you use the rospy.wait_for_service("static_map")
        # and map_service = rospy.ServiceProxy("static_map", GetMap) statements to get the static map from the map
        # server and store it in the self.map_msg variable. This approach is sufficient when you only need the map
        # once, but if the map is changing, you need to continuously get the updated map information. This is why we
        # no longer have those lines in the modified script.
        # The line rospy.Subscriber("occupancy_grid", OccupancyGrid, self.occupancy_grid_callback) is used to subscribe
        # to a ROS topic named "occupancy_grid", which publishes messages of type OccupancyGrid.
        # The first argument "occupancy_grid" is the name of the topic to which the node is subscribing.
        # The second argument "OccupancyGrid" is the type of messages that are being published on the topic.
        # The third argument self.occupancy_grid_callback is the callback function that will be called every time a
        # message is received on the "occupancy_grid" topic. The function will be passed the message as its only
        # argument. In this case, the callback function occupancy_grid_callback is defined elsewhere in the script.
        rospy.Subscriber("/map", OccupancyGrid, self.occupancy_grid_callback)

        # Create a publisher for the path
        # The line self.path_pub = rospy.Publisher("path", Path, queue_size=10) creates a publisher in ROS to publish
        # a Path message on the "path" topic.
        # rospy.Publisher("path", Path, queue_size=10) creates a publisher object that will publish Path messages to
        # the "path" topic.
        # A Path message is a ROS message type that represents a sequence of poses, typically used to represent a path
        # or a trajectory in robotic applications. It is defined in the nav_msgs/Path.msg file. A Path message
        # contains a header, a sequence of poses, and a reference frame. The header provides information about the
        # time stamp and the coordinate frame of the message. The poses in the sequence are represented as PoseStamped
        # messages, which include a position and an orientation. The reference frame provides the coordinate system
        # in which the poses are specified. Path messages are commonly used in ROS navigation systems to represent
        # the planned or computed path for a robot to follow.
        # Note this line initialises the path topic publisher in the path planner node
        # what subscribes to this path topic?
        # it depends on the implementation of the syetm that is using the path topic. The path topic is a ROS topic
        # that ie being published by the frist_path_planner node. Any node in the ROS system that wants to receieve the
        # data published on this topic can subscirbe to it. Typically, a robot control node would subscribe to this
        # topic and use the path information to control the robots moevments.
        self.path_pub = rospy.Publisher("path", Path, queue_size=10)

        # Subscribe to the goal pose
        # the line rospy.Subscriber("goal_pose", PoseStamped, self.goal_pose_callback) is setting up a ROS subscriber
        # in the code. This line creates a new subscriber to the topic "goal_pose", which listens for messages of type
        # PoseStamped. The callback function self.goal_pose_callback is specified, which means that every time a
        # message is received on the "goal_pose" topic, this callback function will be executed with the received
        # message as an argument. The purpose of this subscriber is to receive the goal pose that the robot should
        # navigate to.
        # what publishes the goal_pose topic
        #
        # The node that publishes the "goal_pose" topic is not specified in the code that you provided. The source of
        # the "goal_pose" topic could be a node that inputs the desired goal position, or it could be a node that
        # calculates the goal position based on some other information. The exact implementation of the "goal_pose"
        # publisher node depends on the specific application and requirements.
        # All i need is goal position of x,y in real time. Maybe add a delay (so that it doesnt constantly change
        rospy.Subscriber("goal_pose", PoseStamped, self.goal_callback)

        # Initialize the map data
        # The map data needs to be initialized because the callback function "occupancy_grid_callback" will update
        # the map_msg with the latest data received from the occupancy_grid topic. By initializing the map_msg, we
        # ensure that the callback function has a variable to modify and store the updated map data.
        # In Python, initializing a variable to None means that the variable has been declared but no value has been
        # assigned to it yet. This is a way to indicate that the variable exists, but it does not yet have any
        # meaningful data. Later on, the variable can be assigned a value, which can be of any type, including a list,
        # a string, an integer, or any other data type in Python.
        self.map_msg = None
        self.map_array = None  

        # Initialize the start and goal positions
        self.start = None
        self.goal = None

    # whenever there is an occuancy grid topic subscribed to, the occuancy callback methd below is called and exeecuted.
    # The outputs of the Occupancy grid from the subscriber are inputted as map_msg
    def occupancy_grid_callback(self, map_msg):
        # Update the map data
        self.map_msg = map_msg
        # Create a 2D numpy array from the map
        # This code block is used to create a 2D numpy array from the map data stored in the self.map_msg field. The
        # resulting numpy array is stored in the self.map_array field of the PathPlannerNode object.
        # np.array(self.map_msg.data, dtype=np.int8): This line creates a numpy array from the data field of the Map
        # message stored in self.map_msg. The dtype argument is set to np.int8 to specify that the  numpy array should
        # have 8 bit integers as its data type (see onenote)
        # .reshape((self.map_msg.info.height, self.map_msg.info.width)): This line reshapes the numpy array
        # from a 1D array to a 2D array. The shape of the array is specified by the height and width fields of the
        # MapInfo message stored in the info field of the Map message.
        #
        # In summary, this code block creates a 2D numpy array from the data field of the Map message, using the height
        # and width fields from the MapInfo message to reshape the array. The resulting numpy array can be used for
        # further processing and analysis.
        # (now that we have recieved the data for the subsicrber, we meed to process it in a way that useful. To
        # Implement path planning, i need to store the data in an array )
        self.map_array = np.array(self.map_msg.data, dtype=np.int8).reshape(
            (self.map_msg.info.height, self.map_msg.info.width))

    def goal_callback(self, goal_pose):
        # if the map is not available yet, return to the subsciber
        if self.map_msg is None:
            return
        # rospy.loginfo is powerful print debuggin ttol  - we print the subscribed info onto the terminal and also to a
        # log file
        rospy.loginfo(rospy.get_caller_id() + "The Goal position is %s", PoseStamped)
        # confirm our callback wsa executed correctly
        print("Goal callback executed correctly")
        # Now that we have receieved the goal pose data from the subscriber  and have the current map, we need to
        # to conver the x and y of the goal to the occupancy grid format:
        # The following code is setting the goal position in terms of indices within the map data. It takes the goal
        # position from a PoseStamped message and converts it into indices that can be used to access the data in
        # the occupancy grid.
        #
        # Here's a step-by-step explanation of what is happening:
        #
        # 1) goal_pose is an input of type PoseStamped that represents the goal position.
        # 2) goal_pose.pose.position.x and goal_pose.pose.position.y are the x and y coordinates of the goal position,
        # respectively.
        # 3) self.map_msg is an instance variable that holds the map data, which is an object of type OccupancyGrid.
        # 4) self.map_msg.info.origin.position.x and self.map_msg.info.origin.position.y are the x and y coordinates
        # of the origin of the map data, respectively.
        # 5) self.map_msg.info.resolution is the resolution of the map data, which is the size of each cell in the map
        # in real-world units.
        # 6) The difference between the x and y coordinates of the goal position and the origin of the map data are
        # divided by the resolution to get the number of cells that separate the two positions.
        # 7) The result is rounded down to the nearest integer using int() to get the indices of the cells in the map
        # data that correspond to the goal position.
        # 8) The x and y indices are then packed into a tuple self.goal.
        self.goal = (
          int((goal_pose.pose.position.x - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution),
          int((goal_pose.pose.position.y - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution))

        # Perform the A* search to find the path
        # Now that i this callback function we have the goal in @occupancy grid form, we cna apply a*
        path = self.a_star(self.start, self.goal)

        # Publish the path
        path_msg = Path()
        path_msg.header.frame_id = self.map_msg.header.frame_id
        for x, y in path:
            pose = PoseStamped()
            pose.pose.position.x = x * self.map_msg.info.resolution + self.map_msg.info.origin.position.x
            pose.pose.position.y = y * self.map_msg.info.resolution + self.map_msg.info.origin.position.y
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    # implement rrt instead
