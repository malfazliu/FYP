# First open a new terminal and execute:


gnome-terminal --tab  --title="Simulation" -- bash -c 'endyarnstart() { exec bash; }; trap endyarnstart INT; roslaunch husky_gazebo husky_playpen.launch;'

# You need to let roslaunch open, before proceeding, otherwise it closes the launch file:
sleep 3 # pause for 5s.

# Open a new tab within this window, then execute:
# for some reason this tab STILL closes when you CTRL C

gnome-terminal --tab  -- bash -c 'endyarnstart() { exec bash; }; trap endyarnstart INT; roslaunch husky_viz view_robot.launch;'
sleep 2


# Record Data
cd; cd ~/MAL
gnome-terminal --tab  -- bash -c 'endyarnstart() { exec bash; }; trap endyarnstart INT; roslaunch scan2frontScan.launch;'
sleep 3

# This would be the case if I were to use husky_cartogapher_navigation

#gnome-terminal --tab  -- bash -c 'endyarnstart() { exec bash; }; trap endyarnstart INT; roslaunch husky_cartographer_navigation cartographer_demo.launch;'

# Because of reasons explained in my instalation process, it wasnt working having both catkin_CART and catkin_CARThusky. Since I only needed the husky.lua and cartographer_demo i just moved them into cartographer ros. Thus now:

gnome-terminal --tab  -- bash -c 'endyarnstart() { exec bash; }; trap endyarnstart INT; roslaunch cartographer_ros cartographer_demo.launch;'

