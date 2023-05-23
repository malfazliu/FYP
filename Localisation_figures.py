import numpy as np
import matplotlib.pyplot as plt

# import waypoint data, append starting position, [0, 0] to front row then map it realtive to map frame
waypoints = np.genfromtxt('waypointsDATA.csv', delimiter=',')
new_row = np.array([0, 0])
waypoints = np.vstack((new_row, waypoints))
waypoints[:, 0] += 2
waypoints[:, 1] += -31.9

# import data localisation
data_EKF = np.genfromtxt('data_localisation.csv', delimiter=',')
# Extract columns 1, 2, 4, 5 from data_EKF
x_est = data_EKF[:, 0]
y_est = data_EKF[:, 1]
x_true = data_EKF[:, 3]
y_true = data_EKF[:, 4]

# Compute x_error and y_error
x_error = np.abs(x_true - x_est)
y_error = np.abs(y_true - y_est)

# Plot x_error and y_error
t = np.arange(len(x_error))  # time axis
plt.plot(t, x_error, label='X Error')
plt.plot(t, y_error, label='Y Error')
plt.xlabel('Time (seconds)')
plt.ylabel('Error (meters)')
plt.title('Cartographer Error Metrics')
plt.legend()
plt.show(block=False)

plt.pause(0.1)


# Create a new figure
plt.figure()
# Plot column 2 against column 1
plt.plot(x_est, y_est, label='Estimated position')
# Plot column 5 against column 4
plt.plot(x_true, y_true, label='True position')

# Plot waypoints
plt.scatter(waypoints[:, 0], waypoints[:, 1], c='red', marker='x', label='Waypoints')
# Add labels to the waypoints markers
num_waypoints = len(waypoints)
for i, waypoint in enumerate(waypoints):
    if i == 0:
        label = 'start'
        fontweight = 'bold'
        fontsize = 12
        offset_x = 0.7
        offset_y = 0.7
    elif i == num_waypoints - 1:
        label = 'end'
        fontweight = 'bold'
        fontsize = 12
        offset_x = 0.7
        offset_y = 0.7
    else:
        label = str(i)
        fontweight = 'normal'
        fontsize = 10
        offset_x = 0.7
        offset_y = 0.7
    plt.text(waypoint[0] + offset_x, waypoint[1] + offset_y, label, ha='center', va='center', fontweight=fontweight, fontsize=fontsize)


# Add axis labels and a title
# Add a legend
plt.legend()
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.title('True vs. Estimated Cartographer Position')
# Display the plot
plt.show()
