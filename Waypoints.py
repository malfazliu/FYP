import numpy as np

waypoints = np.array([[0.25, -6.80],
                     [5.48, -6.8],
                     [0.22, -11.3],
                     [8.31, -15.1],
                     [15.1, -5.1],
                     [3.3, -0.34],
                     [5.5, 11.0],
                     [0.0, 11.4],
                     [0.0, 13.4]])
np.savetxt('waypointsDATA.csv', waypoints, delimiter=',')