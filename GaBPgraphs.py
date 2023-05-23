import matplotlib.pyplot as plt
import numpy as np

mean = np.genfromtxt('Saves/IPP2Mean.csv', delimiter=',')
resolution = 0.4  # Resolution in meters

# Multiply the mean array by 1000 for visualization
mean_array_scaled = mean * 1000

# Get the shape of the mean array
height, width = mean_array_scaled.shape

# Create the x and y axis values in meters
x = [-val * resolution for val in reversed(range(width))]  # Reverse the x-axis values and add a minus sign
y = [val * resolution for val in range(height)]

# Plot the mean data as an image
plt.figure()
plt.imshow(mean_array_scaled, cmap='viridis', vmin=0, vmax=2000)

# Set the tick labels and positions for x and y axes
x_ticks = range(0, width, int(width / 5))
x_tick_labels = ['{:.1f}'.format(val) for val in x[::int(width / 5)]]
plt.xticks(x_ticks, x_tick_labels)

y_ticks = range(0, height, int(height / 6))
y_tick_labels = ['{:.1f}'.format(val) for val in y[::int(height / 6)]]
plt.yticks(y_ticks, y_tick_labels)

colorbar = plt.colorbar()
colorbar.set_label('Concentration (ppm)')
plt.xlabel('Y (m)')
plt.ylabel('X (m)')
plt.title('IPP 2 GaBP Mean Concentration Map')
# plt.show()

plt.show(block=False)
plt.pause(0.1)

variance = np.genfromtxt('Saves/IPP2Variance.csv', delimiter=',')
resolution = 0.4  # Resolution in meters

# Multiply the mean array by 1000 for visualization
var_array_scaled = variance * 1

# Get the shape of the mean array
height, width = var_array_scaled.shape

# Create the x and y axis values in meters
x = [-val * resolution for val in reversed(range(width))]  # Reverse the x-axis values and add a minus sign
y = [val * resolution for val in range(height)]

# Plot the mean data as an image
plt.figure()
plt.imshow(var_array_scaled, cmap='viridis')

# Set the tick labels and positions for x and y axes
x_ticks = range(0, width, int(width / 5))
x_tick_labels = ['{:.1f}'.format(val) for val in x[::int(width / 5)]]
plt.xticks(x_ticks, x_tick_labels)

y_ticks = range(0, height, int(height / 6))
y_tick_labels = ['{:.1f}'.format(val) for val in y[::int(height / 6)]]
plt.yticks(y_ticks, y_tick_labels)

colorbar = plt.colorbar()
colorbar.set_label('Variance')
plt.xlabel('Y (m)')
plt.ylabel('X (m)')
plt.title('IPP 2 GaBP Variance Map')
plt.show()