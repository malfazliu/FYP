import numpy as np
import matplotlib.pyplot as plt

# import waypoint data, append starting position, [0, 0] to front row then map it realtive to map frame
waypoints = np.genfromtxt('waypointsDATA.csv', delimiter=',')
new_row = np.array([0, 0])
waypoints = np.vstack((new_row, waypoints))
waypoints[:, 0] += 2
waypoints[:, 1] += -31.9

# import data localisation
data_AMCL = np.genfromtxt('Saves/data_localisationAMCL.csv', delimiter=',')
# Extract columns 1, 2, 4, 5 from data_EKF
x_est_AMCL = data_AMCL[:, 0]
y_est_AMCL = data_AMCL[:, 1]
x_true_AMCL = data_AMCL[:, 3]
y_true_AMCL = data_AMCL[:, 4]

# Compute x_error and y_error
x_error_AMCL = np.abs(x_true_AMCL - x_est_AMCL)
y_error_AMCL = np.abs(y_true_AMCL - y_est_AMCL)

# import data localisation
data_CART = np.genfromtxt('Saves/data_localisationCART.csv', delimiter=',')
# Extract columns 1, 2, 4, 5 from data_EKF
x_est_CART = data_CART[:, 0]
y_est_CART = data_CART[:, 1]
x_true_CART = data_CART[:, 3]
y_true_CART = data_CART[:, 4]

# Compute x_error and y_error
x_error_CART = np.abs(x_true_CART - x_est_CART)
y_error_CART = np.abs(y_true_CART - y_est_CART)

# Plot x_error and y_error
t_AMCL = np.arange(len(x_error_AMCL))  # time axis
t_CART = np.arange(len(x_error_CART))  # time axis
plt.plot(t_AMCL, x_error_AMCL, label='X Error - AMCL', color='black')
plt.plot(t_AMCL, y_error_AMCL, label='Y Error - AMCL', color='grey')
plt.plot(t_CART, x_error_CART, label='X Error - CART', color='purple')
plt.plot(t_CART, y_error_CART, label='Y Error - CART', color='pink')
plt.xlabel('Time (seconds)')
plt.ylabel('Error (meters)')
plt.yticks([0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2])
plt.title('AMCL and Cartographer Error Metrics')
plt.legend()
plt.show()

# plt.pause(0.1)
#
#
# # Create a new figure
# plt.figure()
# # Plot column 2 against column 1
# plt.plot(x_est, y_est, label='Estimated position')
# # Plot column 5 against column 4
# plt.plot(x_true, y_true, label='True position')

# Plot waypoints
# plt.scatter(waypoints[:, 0], waypoints[:, 1], c='red', marker='x', label='Waypoints')
# # Add labels to the waypoints markers
# num_waypoints = len(waypoints)
# for i, waypoint in enumerate(waypoints):
#     if i == 0:
#         label = 'start'
#         fontweight = 'bold'
#         fontsize = 12
#         offset_x = 0.7
#         offset_y = 0.7
#     elif i == num_waypoints - 1:
#         label = 'end'
#         fontweight = 'bold'
#         fontsize = 12
#         offset_x = 0.7
#         offset_y = 0.7
#     else:
#         label = str(i)
#         fontweight = 'normal'
#         fontsize = 10
#         offset_x = 0.7
#         offset_y = 0.7
#     plt.text(waypoint[0] + offset_x, waypoint[1] + offset_y, label, ha='center', va='center', fontweight=fontweight, fontsize=fontsize)
#
#
# # Add a legend
# plt.legend()
# # Add axis labels and a title
# plt.xlabel('X position (m)')
# plt.ylabel('Y position (m)')
# plt.title('True vs. Estimated Position')
# # Display the plot
# plt.show()
