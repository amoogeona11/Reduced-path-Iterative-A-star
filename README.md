# Reduced-path-Iterative-A-star

Environment
============
Ubuntu 18.04, ROS melodic, Gazebo9, QT Creator, C++

## Save world files
save .world files to your computer's usr/share/gazebo-9/worlds
### backup your original shapes.world file
launch files are in opt/ros/melodic/share/gazebo_ros/launch

## clone repository and catkin make

## Simulation
### Launch world files
'''
roslaunch gazebo_ros shapes_world.launch
'''

or make an editional launch file

'''
roslaunch gazebo_ros launch_file.launch
'''

### Spawn Drone
'''
roslaunch sjtu_drone simple.launch
'''

### Spawn Car
'''
roslaunch r2w_description spawn.launch
'''

### Run algorithm
'''
rosrun move_test astar
'''

for reduced path astar

'''
rosrun move_test reduced_path_astar
'''

### To move the Drone
'''
rosrun sjtu_drone drone_keyboard
'''
