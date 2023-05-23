#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import numpy as np

class GoalSelector:

    # Defining Class Attributes, resolution (ensure it is the same as the value set within gabp_launch file)

    # Add a way to extract these values
    resolution_OGM = 0.05
    resolution_GasMap = 0.4

    def __init__(self):
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('GoalSelector_py', anonymous=True)

        # Define instance attribute (for good practise. It is best to initalise
        # instance attributes within the __init__ as opposed to the class methods.
        self.meanSTRING = None
        self.meanMSG = None
        self.varSTRING = None
        self.varMSG = None
        self.mean_array = None
        self.var_array = None

        # Subscribes to Mean DATA.
        rospy.Subscriber("/gabp/mean/matrix", Float32MultiArray, self.mean_callback)

        # Subscribe to Var data
        rospy.Subscriber("/gabp/var/matrix", Float32MultiArray, self.var_callback)

        # Publish the x, y coordinate.
        self.goal_pub = rospy.Publisher("goal_selector", Float32, queue_size=10)

    def mean_callback(self, msg):
        self.meanSTRING = "Reading Mean Data"
        self.meanMSG = msg

    def var_callback(self, msg):
        self.varSTRING = "Reading Var Data"
        self.varMSG = msg

    def array_process(self):
        # Here need to write code that includes self.meanMSG dimesnions. Note By running print of the dimensions, and
        # manually moving robot around, it is confirmed that the dimensions of the variance Matrix from the gabp is
        # always the same as the dimensions of the mean Matrix i.e they update simultanesouly :). So for the purpose of
        # method, will use the dimensions from mean Matrix (but could also be done using the variance matrix).
        if self.meanMSG is None or self.varMSG is None:
            rospy.logwarn('No data received yet')
            return None
        else:
            # So the input data is not in numpy format, it has 2 dim. dim[0], has been defined by Callum as x label
            # and dim[1] as y label. Note, I checked by printing them, it is reshape(x, y). So the first input of
            # reshape is x which can be accessed via self.meanMSG.layout.dim[0].size.
            # And the second input is y , which can be accessed in the same way but with [1] as the index.
            # And note the output mean_array.shape, it is also (x,y). (Note that .shape returns  rows, columns, hence
            # in mean_array, x is the row, y is the column
            mean_array = np.array(self.meanMSG.data).reshape((self.meanMSG.layout.dim[0].size,
                                                              self.meanMSG.layout.dim[1].size))
            print("mean_array dims x,y:", mean_array.shape)
            # Note when I drive around, it is apparant that the max indices are given from bottom left array. I.e the
            # origin is bottom left. I can solve by doing this flip. Now, say I have mapped the max mean conc. point,
            # if i move down in the map (i.e negative y using cad coords), then the max indice should not change, the
            # row should instead simply get smaller
            # Need to flip axis 1, which is the columns, which are y in mean_array.
            mean_array = np.flip(mean_array, axis=1)
            self.mean_array = mean_array

            var_array = np.array(self.varMSG.data).reshape((self.varMSG.layout.dim[0].size,
                                                            self.varMSG.layout.dim[1].size))
            print("var_array dims x,y:", var_array.shape)
            var_array = np.flip(var_array, axis=1)
            self.var_array = var_array

    def data_process(self):
        if self.mean_array is None:
            rospy.logwarn('No data received yet')
            return None
        else:
            # So summary, rows are first index of the mean_array.shape, and they are in our case the x values. The
            # columns are the second index, i.e [1], and these refer to the y values

            # Get indices of maximum element. Similarly this returns the max index in the rows of mean_array, i,e x
            # as the first index etc.
            max_mean_indices = np.unravel_index(np.argmax(self.mean_array), self.mean_array.shape)
            print("max mean element indices x,y:", max_mean_indices)

            # Convert indices to coordinates (with resolution scaling and offset from top left
            x_topleft = max_mean_indices[0] * self.resolution_GasMap - self.resolution_GasMap / 2.0
            y_topleft = max_mean_indices[1] * self.resolution_GasMap - self.resolution_GasMap / 2.0

            # Now realtive to odom Frame (place holders atm)
            x = 5
            y = 5

            ## Data Process

            # Return coordinates as tuple
            return x_topleft, y_topleft, x, y


meanDataStorage = None
varDataStorage = None
# Running The GoalSelector
if __name__ == '__main__':
    try:
        # Need to define the instance outside the while loop so that it doesn't keep redefining.
        Instancee = GoalSelector()
        while not rospy.is_shutdown():
            # To view the msg via variable, run in debug mode then add a breakpoint on the next line as it runs.
            # It is a 2D array in this case (a matrix). Note x is the wide, y is height.
            meanDataStorage = Instancee.meanMSG
            varDataStorage = Instancee.varMSG
            if Instancee.meanMSG is not None or Instancee.varMSG is not None:
                print("--------------------------------------")
                print(Instancee.meanSTRING)
                print(Instancee.varSTRING)
                # print(str(Instanceee.meanMSG.layout.dim[0].size), str(Instanceee.meanMSG.layout.dim[1].size))
                # print(str(Instanceee.meanMSG.layout.dim[1].size))
                # print(str(Instanceee.varMSG.layout.dim[0].size))
                # print(str(Instanceee.varMSG.layout.dim[1].size))
                print('max mean element COORDS:', Instancee.data_process())
            else:
                rospy.loginfo("Not Reading data, meanMSG and varMSG set to None")
            # Allow for some time to process (rospy.spin() will not allow this behaviour)
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        # Save the data to a CSV file
        if Instancee.mean_array is not None or Instancee.var_array is not None:
            np.savetxt('Mean.csv', Instancee.mean_array, delimiter=',')
            np.savetxt('Variance.csv', Instancee.var_array, delimiter=',')
            rospy.loginfo("Navigation test finished. Data has been Saved")
        else:
            rospy.loginfo("Navigation test finished. Data has not been saved")
