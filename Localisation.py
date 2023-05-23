import rospy
from gazebo_msgs.msg import ModelStates
import tf
import numpy as np

data_matrix = np.zeros((0, 6))
gazebo_msg = None

def callback_true(msg):
    global gazebo_msg
    gazebo_msg = msg
    rospy.loginfo_throttle(1, 'Gazebo model states msg subscribed')
    return gazebo_msg


def localisation_node():
    rospy.init_node('localisation_data_logger', anonymous=True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback_true)
    # Listen to Robot Base Position. Note this in x,y is alligned with the PID sensor that has been set at base_link
    listener = tf.TransformListener()
    return listener


def localisation_process(robot_trans):
    global data_matrix
    if gazebo_msg is not None:
        husky_gazebo = gazebo_msg.pose[2]
        husky_true_x = husky_gazebo.position.x
        husky_true_y = husky_gazebo.position.y
        husky_true_z = husky_gazebo.position.z
        rospy.loginfo('GAZEBO Robot Position rel. Map: (' + str(round(husky_true_x, 4)) + ', ' +
                      str(round(husky_true_y, 4)) + ', ' + str(round(husky_true_z, 4)) + ')')
        data_matrix = np.vstack((data_matrix, np.array([robot_trans[0], robot_trans[1], robot_trans[2], husky_true_x,
                                                        husky_true_y, husky_true_z])))
        rospy.sleep(1.0)
        return data_matrix
    else:
        rospy.logwarn(' Gazebo msg NOTTT RECEIEVED')


def robot_tf_listener(listener):
    try:
        (robot_trans, robot_rot) = (listener.lookupTransform('/map', '/base_link', rospy.Time(0)))
        rospy.loginfo('ESTIMATED Robot Position rel. Map: (' + str(round(robot_trans[0], 4)) + ', ' +
                      str(round(robot_trans[1], 4)) + ', ' + str(round(robot_trans[2], 4)) + ')')
        # For finding what waypoints to select;
        (robot_transODOM, robot_rotODOM) = (listener.lookupTransform('/odom', '/base_link', rospy.Time(0)))
        rospy.loginfo('ODOM Robot Position rel. Map: (' + str(round(robot_transODOM[0], 4)) + ', ' +
                      str(round(robot_transODOM[1], 4)) + ', ' + str(round(robot_transODOM[2], 4)) + ')')
        localisation_process(robot_trans)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn('Failed to read Robot Position')

if __name__ == '__main__':
    try:
        # Initialize node
        listener = localisation_node()
        while not rospy.is_shutdown():
            print('-------------------------------------------------------------------')
            robot_tf_listener(listener)

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        if data_matrix is not None:
            # Save data matrix to file
            np.savetxt('data_localisation.csv', data_matrix, delimiter=',')
            rospy.loginfo("Data has been Saved!")

        else:
            rospy.loginfo('No Data was saved!')
