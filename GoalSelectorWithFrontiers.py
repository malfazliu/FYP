#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import numpy as np
from nav_msgs.msg import OccupancyGrid
import tf  # compatible with python2. Hence using python2.7. I can also do some source catkin build for python3.
import matplotlib.pyplot as plt
import cv2

class GoalSelector:

    # Defining Class Attributes, resolution (ensure it is the same as the value set within gabp_launch file)
    resolution_GasMap = 0.4
    radius_gas_mapping_state = 15  # In meters.

    def __init__(self):
        # Initializes a rospy node
        rospy.init_node('GoalSelector_py', anonymous=True)

        # Define instance attribute (for good practise. It is best to initalise
        # instance attributes within the __init__ as opposed to the class methods.
        self.meanSTRING = None
        self.meanMSG = None
        self.varSTRING = None
        self.varMSG = None
        self.occSTRING = None
        self.occMSG = None
        self.mean_array = None
        self.var_array = None
        self.cost_array = None
        self.resolution_OGM = None
        self.x_odom = None
        self.y_odom = None
        self.occ_array = None
        self.robot_trans = None
        self.robot_rot = None
        self.array_range = None
        self.goals_ordered_matrix = None
        self.move_base_occMSG = None
        self.move_base_occSTRING = None
        self.move_base_occ_array = None
        self.mb_occ_array = None
        self.x_mb_odom = None
        self.y_mb_odom = None
        self.resolution_mb_OGM = None
        self.chosen_goal_info = None
        self.x_elem_goal_bot_left = None
        self.y_elem_goal_bot_left = None
        self.frontier_indices = None
        self.rank_matrix_2 = None
        # Initialize an empty array with 0 rows and 5 columns
        self.chosen_goal_storage = np.zeros((0, 5))

        # Subscribes to Mean DATA. Everytime msg recieved, it is processed in the mean_callback
        rospy.Subscriber("/gabp/mean/matrix", Float32MultiArray, self.mean_callback)

        # Subscribe to Var data
        rospy.Subscriber("/gabp/var/matrix", Float32MultiArray, self.var_callback)

        # Publish the x, y coordinate.
        self.goal_publisher = rospy.Publisher("goal_selected", Float32MultiArray, queue_size=10)

        # Subscribe to OGM
        rospy.Subscriber("/CARTmap", OccupancyGrid, self.occ_callback)

        # Listen to Robot Base Position. Note this in x,y is alligned with the PID sensor that has been set at base_link
        self.listener = tf.TransformListener()

        # SUbscribe to move_base OGM (THE GLOBAL ONE being made with cart.)
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.move_base_occ_callback)

    def mean_callback(self, msg):  # Note the second argument is the msg it has subscribed to.
        self.meanSTRING = "Reading Mean Data"
        self.meanMSG = msg
        rospy.loginfo('Reading Mean Data')

    def var_callback(self, msg):
        self.varSTRING = "Reading Var Data"
        self.varMSG = msg
        rospy.loginfo('Reading Variance Data')

    def occ_callback(self, msg):
        self.occSTRING = "Reading Occupany Grid Map Data"
        self.occMSG = msg
        rospy.loginfo('Reading Occupancy Grid Map Data')

    def move_base_occ_callback(self, msg):
        self.move_base_occSTRING = "Reading move_base Occupancy Grid Map Data"
        self.move_base_occMSG = msg
        rospy.loginfo(self.move_base_occSTRING)

    def robot_tf_listener(self):
        try:
            (self.robot_trans, self.robot_rot) = (self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0)))
            # Perhaps i can use the robots orientation as a way to ensure loop closure occurs more often.
            # self.robot_rot = (self.listener.lookupTransform('/base_link', '/odom', rospy.Time(0)))[0]

            rospy.loginfo('Robot Position rel. Odom: (' + str(round(self.robot_trans[0], 2)) + ', ' +
                          str(round(self.robot_trans[1], 2)) + ', ' + str(round(self.robot_trans[2], 2)) + ')')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Failed to read Robot Position')

    def array_process(self):
        # Here need to write code that includes self.meanMSG dimesnions. Note By running print of the dimensions, and
        # manually moving robot around, it is confirmed that the dimensions of the variance Matrix from the gabp is
        # always the same as the dimensions of the mean Matrix i.e they update simultanesouly :). So for the purpose of
        # method, will use the dimensions from mean Matrix (but could also be done using the variance matrix).
        if self.meanMSG is None or self.varMSG is None:
            return rospy.logwarn('array_process has failed because no mean/variance data received yet')
        else:
            ## Array Process

            # So the input data is not in numpy format, it has 2 dim. dim[0], has been defined by Callum as x label
            # and dim[1] as y label. Note, I checked by printing them, it is reshape(dim[0], dim[1]). It depends on the
            # data ur using but for callums, for the data to be sorted right its this way.
            # And note the output mean_array.shape, it is (x,y). (Note that .shape returns  rows, columns, hence
            # in mean_array, x is the row, y is the column). The indexing reference is actually bottom left of the map
            # u see in rviz (check onenote for further explanation)
            mean_array = np.array(self.meanMSG.data).reshape((self.meanMSG.layout.dim[0].size,
                                                              self.meanMSG.layout.dim[1].size))
            # print("mean_array dims x,y:", mean_array.shape)
            # Note when I drive around, it is apparant that the max indices are given from bottom left array. I.e the
            # origin is bottom left. I can solve by doing this flip. Now, say I have mapped the max mean conc. point,
            # if i move down in the map (i.e negative y using cad coords), then the max indice should not change, the
            # row should instead simply get smaller
            # Need to flip axis 1, which is the columns, which are y in mean_array.
            # mean_array = np.flip(mean_array, axis=1)
            self.mean_array = mean_array

            var_array = np.array(self.varMSG.data).reshape((self.varMSG.layout.dim[0].size,
                                                            self.varMSG.layout.dim[1].size))
            # print("var_array dims x,y:", var_array.shape)
            # var_array = np.flip(var_array, axis=1)
            self.var_array = var_array
            rospy.loginfo('array_process method has succeeded')
            return

    def occ_process(self):
        if self.occMSG is None:
            return rospy.logwarn('No OCCUPANCY data received on occ_process method')
        else:
            try:
                # Want to put the occ data in the same orientation as the mean and var. The correct input is height,
                # then width, but I need to transpose it as well. AFter this transpose x will be in rows, y will be in
                # columns, and the indexing (which in python corresponds to top left array) will inf act be relative to
                # bottom left of graph in rviz. That is x if the exact same, but y is flipped to the graph uy see in
                # rviz.
                occ_array = np.array(self.occMSG.data).reshape((self.occMSG.info.height, self.occMSG.info.width))
                occ_array = occ_array.transpose()
                print("occ_array dims x,y:", occ_array.shape)
                self.occ_array = occ_array
                # The folowing are from OccMSG so have nothing to do with the processing of occ_array.
                self.resolution_OGM = self.occMSG.info.resolution
                self.x_odom = self.occMSG.info.origin.position.x
                self.y_odom = self.occMSG.info.origin.position.y
                rospy.loginfo('occ_process method has succeeded')
                rospy.loginfo("x_odom, y_odom: (" + str(round(self.x_odom, 2)) + ', ' + str(round(self.y_odom, 2)) +
                              ')')
                print(str((self.occ_array.shape[0]) * self.resolution_OGM) + ', ' +
                      str((self.occ_array.shape[1]) * self.resolution_OGM))
                return
            except ValueError:
                return rospy.logwarn("CANNOT RESHAPE occupancy matrix")

    def move_base_occ_process(self):
        if self.move_base_occMSG is None:
            return rospy.logwarn('No MOVE BASE OCCUPANCY data received on occ_process method')
        else:
            try:
                # Same as other.
                mb_occ_array = np.array(self.move_base_occMSG.data).reshape((self.move_base_occMSG.info.width,
                                                                             self.move_base_occMSG.info.height))
                mb_occ_array = mb_occ_array.transpose()
                self.mb_occ_array = mb_occ_array
                self.resolution_mb_OGM = self.move_base_occMSG.info.resolution
                self.x_mb_odom = self.move_base_occMSG.info.origin.position.x
                self.y_mb_odom = self.move_base_occMSG.info.origin.position.y
                rospy.loginfo('move_base_occ_process method has succeeded')
                rospy.loginfo("x_mb_odom, y_mb_odom: (" + str(round(self.x_mb_odom, 2)) + ', ' +
                              str(round(self.y_mb_odom, 2)) + ')')
                print(str((self.mb_occ_array.shape[0])*self.resolution_mb_OGM) + ', ' +
                      str((self.mb_occ_array.shape[1])*self.resolution_mb_OGM))
                return
            except ValueError:
                return rospy.logwarn("CANNOT RESHAPE move base occupancy matrix")

    def frontier_process(self):
        if self.occ_array is None:
            return rospy.logwarn('No occ_array data received on occ_process method')
        else:
            # Define the kernel for morphological opening as a 3x3 array of ones. The choice of the kernel size in
            # morphological operations depends on the specific image processing task and the characteristics of the
            # image being processed. A smaller kernel is useful when we want to preserve small details and fine
            # structures in the image. However, if the kernel is too small, it may not be effective in removing small
            # noise or small objects in the image. On the other hand, a larger kernel is useful for removing larger
            # objects or smoothing the image, but it may also remove some small details in the image. Therefore, the
            # choice of kernel size is usually a trade-off between the need for removing noise or objects and the need
            # for preserving important details in the image. In the case of the frontier detection, the choice of 3x3
            # kernel seems to be a reasonable size to remove small noise in the image while preserving the overall
            # structure of the occupied cells.
            kernel = np.ones((3, 3), np.uint8)

            # Dilate the occupied cells using a kernel
            dilated = cv2.dilate(self.occ_array.astype(np.uint8), kernel)

            # Erode the dilated image using the same kernel
            eroded = cv2.erode(dilated, kernel)

            # Compute the difference between the eroded and the original image
            diff = eroded - self.occ_array

            # Get the indices of the frontier cells by finding the non-zero elements in the difference image and
            # transposing the result to get a (n, 2) numpy array of frontier cell indices. np.nonzero
            # returns the indices of the cell that are non zero. the colummns it returns are (row, column) i.e x, y
            self.frontier_indices = np.transpose(np.nonzero(diff))

            ## Now to get the frontier costs --> computing a cost value for each frontier cell in the frontier_indices
            # array. These cost values will be used to sort the frontier cells in descending order of importance, with
            # the most important cell appearing first in the frontier array.

            # Initializes an empty list called dists to store the Euclidean distance to the nearest unexplored cell
            # for each frontier cell.
            dists = []
            for f in self.frontier_indices:
                dist = np.inf
                for i in range(max(0, f[0] - 1), min(self.occ_array.shape[0], f[0] + 2)):
                    for j in range(max(0, f[1] - 1), min(self.occ_array.shape[1], f[1] + 2)):
                        if self.occ_array[i, j] == -1:  # unexplored cell
                            d = np.linalg.norm(f - [i, j])
                            if d < dist:
                                dist = d
                dists.append(dist)

            # Normalize the distances and invert them to get the costs
            costs = 1 / (np.array(dists) + 1)
            costs /= np.max(costs)

    def data_process(self):
        if self.mean_array is not None or self.var_array is not None:
            max_mean_indices = np.unravel_index(np.argmax(self.mean_array), self.mean_array.shape)
            # print("max mean element indices x,y:", max_mean_indices)
            # Convert indices to coordinates (with resolution scaling and offset from top left
            x_botleft = max_mean_indices[0] * self.resolution_GasMap - self.resolution_GasMap / 2.0
            y_botleft = max_mean_indices[1] * self.resolution_GasMap - self.resolution_GasMap / 2.0

            # Now realtive to odom Frame
            x = x_botleft + self.x_odom
            y = y_botleft + self.y_odom

            rospy.loginfo('x,y coords of max mean relative to Odom Frame: (' + str(round(x, 2)) + ', ' +
                          str(round(y, 2)) + ')')

            ## Data Process
            # Find coords where sensor is currently relative to bottom left gas map.
            x_botleft_robot = (self.robot_trans[0] - self.x_odom)
            y_botleft_robot = (self.robot_trans[1] - self.y_odom)
            print("var_array dims x,y: (" + str(self.var_array.shape[0]*self.resolution_GasMap) + ', ' +
                  str(self.var_array.shape[1]*self.resolution_GasMap) + ')')
            print(str(x_botleft_robot))
            print(str(y_botleft_robot))
            # Converting coords where sensor is currently relative to bot left gas map to elements:
            x_botleft_robot_elem = int(x_botleft_robot/self.resolution_GasMap) + 1
            y_botleft_robot_elem = int(y_botleft_robot/self.resolution_GasMap) + 1
            print(str(x_botleft_robot_elem))
            print(str(y_botleft_robot_elem))
            # Making a matrix, which stores position of robot, and sets a radius for cost function.
            self.array_range = np.zeros((self.mean_array.shape[0], self.mean_array.shape[1]))
            # The row and column indices for the x,y coordinates (already done)
            row = x_botleft_robot_elem
            col = y_botleft_robot_elem
            # Set the element of the 2D array at the row and column indices to 1
            self.array_range[row, col] = 1
            for i in range(self.mean_array.shape[0]):
                for j in range(self.mean_array.shape[1]):
                    distance = np.sqrt((i-row)**2 + (j-col)**2) * self.resolution_GasMap
                    if 2 <= distance <= self.radius_gas_mapping_state:
                        self.array_range[i, j] = 1

            # Cost function array ( I can also put this in the last for loop but leavign seperate for now)
            num_of_ones = 0
            self.cost_array = self.var_array * self.mean_array
            for i in range(self.cost_array.shape[0]):
                for j in range(self.cost_array.shape[1]):
                    if self.array_range[i][j] == 1.0:
                        num_of_ones += 1
                    else:
                        self.cost_array[i][j] = 0
            print('num_of_ones ' + str(num_of_ones))
            # Making a n x 3 matrix of Cost, element x, element, y in order of the cost.
            self.goals_ordered_matrix = np.zeros((num_of_ones, 3))
            # NOte: If i set: temp_cost_array = self.cost_array, the following for loop will then change the value of
            # self.cost_array because temp_cost_array and self.cost_array refer to the same object in memory. When you
            # set temp_cost_array = self.cost_array, you are creating a new reference to the same object that
            # self.cost_array is pointing to. Therefore, any changes made to temp_cost_array will also be reflected
            # in self.cost_array.
            # If you want to avoid changing the value of self.cost_array, you can create a copy of it instead of
            # creating a new reference. You can do this using the copy() method of numpy arrays. This will create a new
            # copy of self.cost_array in memory, so any changes made to temp_cost_array will not affect self.cost_array.
            # Also note: if i did: temp_cost_Array = self.cost_array*2  , temp_cost_Array is a new array object with
            # the same shape as self.cost_array, but its data is a new object with a new reference. Therefore, modifying
            # temp_cost_Array will not modify self.cost_array.
            temp_cost_array = self.cost_array.copy()
            temp_max_indices = np.unravel_index(np.argmax(temp_cost_array), temp_cost_array.shape)
            for index in range(num_of_ones):
                self.goals_ordered_matrix[index][0] = temp_cost_array[temp_max_indices[0]][temp_max_indices[1]]
                self.goals_ordered_matrix[index][1] = temp_max_indices[0]
                self.goals_ordered_matrix[index][2] = temp_max_indices[1]
                temp_cost_array[temp_max_indices[0]][temp_max_indices[1]] = 0
                temp_max_indices = np.unravel_index(np.argmax(temp_cost_array), temp_cost_array.shape)

            #self.rank_matrix_2 = np.empty(shape=(0, 0))
            #self.rank_matrix_2 = np.array([0, 1, 2])
            self.rank_matrix_2 = []
            for t in range(self.goals_ordered_matrix.shape[0]):
                indexi = self.goals_ordered_matrix[t][1]
                indexj = self.goals_ordered_matrix[t][2]

                flag = 0
                for i in range(int(np.floor(indexi - 5.0/self.resolution_GasMap)),
                               int(np.ceil(indexi + 5.0/self.resolution_GasMap))):
                    for j in range(int(np.floor(indexj - 5.0/self.resolution_GasMap)),
                                   int(np.ceil(indexj + 5.0/self.resolution_GasMap))):
                        if 0 <= i < self.mean_array.shape[0] and 0 <= j < self.mean_array.shape[1]:
                            if self.mean_array[i][j] == 0:
                                self.rank_matrix_2.append([0, i, j])
                                #self.rank_matrix_2 = np.vstack([self.rank_matrix_2, [0, i, j]])
            #print(self.rank_matrix_2)
            self.rank_matrix_2 = np.array(self.rank_matrix_2)


            # Then try/check if can be reached, if not moved to next
            rospy.loginfo('data_process method has succeeded')
            return
        else:
            return rospy.logwarn("data_process method failed because no array inputs")

    def goal_process(self):

        try:
            print(self.rank_matrix_2.shape)
            self.rank_matrix_2.reshape((-1, 3))
            self.rank_matrix_2 = np.unique(self.rank_matrix_2, axis=0)
            print(self.rank_matrix_2.shape)

            for index in range(self.rank_matrix_2.shape[0]):
                # For Now going to compare checkl against move_base map. Choice here, probably best move base because
                # whilst move base could do some weird thing from time to time and mess up the local map and hence map,
                # wheras cart wouldnt, it doesnt matter because at that point the sim breaks and would need to start
                # again.  And obv move base has the cost map and cleaner/better occ grid to work with for obstacle
                # avoidance.


                # In our goals_ordered stored matrix we have the elements x and y relative to bottom left mean/var array
                self.x_elem_goal_bot_left = self.rank_matrix_2[index][1]
                self.y_elem_goal_bot_left = self.rank_matrix_2[index][2]
                # Find coords where cost row is currently relative to bot left mean/var/cart array map.
                x_bot_left_cost = self.rank_matrix_2[index][1]*self.resolution_GasMap - self.resolution_GasMap/2
                y_bot_left_cost = self.rank_matrix_2[index][2]*self.resolution_GasMap - self.resolution_GasMap/2

                # Find cords where cost row is relative to odom
                x_odom_cost = x_bot_left_cost + self.x_odom   # I need to use x_odom here because x_bot_left_cost is
                # also using that array (i.e it has the same bottom left/ extracted odom frame as CARTmap.  Hence this
                # is how I can get the x cord relative to odom. NOte if i were going to do the CARTmap topic thing for
                # the gabp node i think i will just have to calc the self.x/y_odom position and that will remain const
                # because map won't get bigger.
                y_odom_cost = y_bot_left_cost + self.y_odom  # Same here, y_odom is necessary, NOT y_mb_odom

                # Converting the coords to movebase bottom left (here you can also convert to CART bott left (again))
                x_mb_bot_left = x_odom_cost - self.x_mb_odom
                y_mb_bot_left = y_odom_cost - self.y_mb_odom

                # Converting the coords mb bottom left to elemenets (if you wanted to use CART bott left change res.
                elem_x_mb_left = int(x_mb_bot_left/self.resolution_mb_OGM) + 1
                elem_y_mb_left = int(y_mb_bot_left/self.resolution_mb_OGM) + 1

                if 0 < self.mb_occ_array[elem_x_mb_left][elem_y_mb_left] <= 45:
                    # Initialize an empty array with 0 rows and 3 columns
                    self.chosen_goal_info = np.array([self.rank_matrix_2[index][0], x_odom_cost, y_odom_cost,
                                                      index, self.mb_occ_array[elem_x_mb_left][elem_y_mb_left]])
                    # Append the new row to the existing matrix
                    # self.chosen_goal_storage = np.append(self.chosen_goal_storage, [self.chosen_goal_info], axis=0)
                    rospy.loginfo("Safe Goal Found!")
                    goal_message = Float32MultiArray()
                    goal_message.data = self.chosen_goal_info.tolist()
                    # Publish the ROS message to the topic
                    self.goal_publisher.publish(goal_message)
                    rospy.loginfo("Goal Published!")
                    break
                else:
                    if index == self.rank_matrix_2.shape[0] - 1:
                        rospy.logwarn("Safe Goal NOT Found!")
        except AttributeError:
            return rospy.logwarn("Goal Process has not recieved inputs yet")

# Running The GoalSelector
if __name__ == '__main__':
    try:
        # Need to define the instance outside the while loop so that it doesn't keep redefining.
        Instancee = GoalSelector()
        while not rospy.is_shutdown():
            print('-------------------------------------------------------------------')
            robot_tf_listener_method_call = Instancee.robot_tf_listener()
            # Need to call all methods same as you do functions. This prints any prints within.
            array_process_method_call = Instancee.array_process()
            occ_process_method_call = Instancee.occ_process()
            move_base_occ_process_call = Instancee.move_base_occ_process()
            # The next line prints the return of the instance method. Provided there is a return.
            # print('x_odom,y_odom', occ_process_method_call)
            # This method needs to be called third because of its dependencies
            data_process_method_call = Instancee.data_process()
            goal_process_method_call = Instancee.goal_process()
            # Allow for some time to process (rospy.spin() will not allow this behaviour)
            rospy.sleep(0.5)

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        # Save the data to a CSV file. Note you need to terminate to make this save. You cant 'stop console'
        if Instancee.mean_array is not None or Instancee.var_array is not None:
            np.savetxt('Mean.csv', Instancee.mean_array, delimiter=',')
            np.savetxt('Variance.csv', Instancee.var_array, delimiter=',')
            rospy.loginfo("Navigation test finished. Data has been Saved")

            # Plot the CART occupancy data as an image
            plt.imshow(Instancee.occ_array, cmap='jet', vmin=-1, vmax=100, alpha=0.7)
            plt.colorbar()
            plt.show(block=False)

            # Pause for 1 second before plotting the next figure
            plt.pause(1)

            # Plot the MB occupancy data as an image
            plt.figure()
            plt.imshow(Instancee.mb_occ_array, cmap='jet', vmin=-1, vmax=100, alpha=0.7)
            plt.colorbar()
            plt.show(block=False)

            plt.pause(1)

            # Plot the mean data as an image
            plt.figure()
            plt.imshow(Instancee.mean_array, cmap='viridis')
            plt.colorbar()
            plt.show(block=False)

            plt.pause(1)

            # Plot the mean data as an image
            plt.figure()
            plt.imshow(np.flipud(Instancee.mean_array.transpose()), cmap='viridis')
            plt.colorbar()
            plt.show(block=False)

            plt.pause(1)

            # PLotting cost map
            plt.figure()
            plt.imshow(Instancee.cost_array, cmap='viridis')
            plt.colorbar()
            # Plot odom with a cross marker
            plt.plot(-int(Instancee.y_odom/Instancee.resolution_GasMap)+1,
                     -int(Instancee.x_odom/Instancee.resolution_GasMap)+1, marker='x', color='red')

            # Plot the goal chosen with a circle marker
            plt.plot(Instancee.y_elem_goal_bot_left, Instancee.x_elem_goal_bot_left, marker='o', color='red')

            plt.show(block=False)

        else:
            rospy.logwarn("Navigation test finished. Data has NOT been Saved")
