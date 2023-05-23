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
sleep 2


gnome-terminal --tab  -- bash -c 'endyarnstart() { exec bash; }; trap endyarnstart INT; roslaunch husky_navigation amcl_demo.launch;'


