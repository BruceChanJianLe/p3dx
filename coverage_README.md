The 'camera_coverage' fork is branched from https://github.com/BruceChanJianLe/p3dx, which provides a foundational environment and robot system for navigational activities. 

The 'camera_coverage' fork and 'coverage_estimation' file provides a piece of code to estimate how much of the total environment the tugbot has 'seen' with its camera at any one time. The file takes into account obstacles into the calculation, and therefore the calculation is conducted on the total 'free space' of the environment. 

This code is operational for one tugbot, but has the potential for extrapolation to a multi robot scenario. The code subscribes to odometry data from /RosAria/odom. The camera's radius can be adjusted however the resolution should be kept consistent. 


How to Run: 

Step 1: ```cd sim_ws/src```

Step 2: ```git clone https://github.com/BruceChanJianLe/p3dx.git```

To return back one step in your hierarchy: 

Step 3: ```cd ..``

Step 4: ```colcon build```

Step 5: ```source install/setup.bash```

Step 6: ```ros2 launch p3dx_gazebo p3dx_warehouse_world.launch.py```

Step 7: ```ros2 launch p3dx_navigation slam_2d_demo.launch.py```

Step 8: Download the cover_2.py file (coverage_estimation) from this branch and move the file to your desired location. 

This is the line that runs the cover_2 node (make sure to replace with your specific file path): 

Step 9: ```python3 /home/user/ros2_ws/sim_ws/src/p3dx/p3dx_gazebo/scripts/cover_2.py```
