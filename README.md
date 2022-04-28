# ENPM661: Project 3 Part 2
## Run instruction:

Copy the entire 'a_star' folder to catkin_ws.
Run `catkin_make` and `source devel/setup.bash`
Run `export TURTLEBOT3_MODEL=burger`

`cd src/DifferentialDrive_Astar/scripts`
Run `python3 diff_drive_Astar.py --start 1000 1000 0 --goal 4000 4000 --rpm 5 10 --clearance 300` to generate path text file needed for gazebo.

Run `roslaunch a_star environment.launch` for gazebo simualtion.

## Video file in 'video.mkv' shows path from (0.8, 0.8) to (3.5,3.5)