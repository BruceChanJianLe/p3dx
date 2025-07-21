The 'camera_coverage' fork is branched from https://github.com/BruceChanJianLe/p3dx, which provides a foundational environment and robot system for navigational activities. 

The 'camera_coverage' fork and 'coverage_estimation' file provides a piece of code to estimate how much of the total environment the tugbot has 'seen' with its camera at any one time. The file takes into account obstacles into the calculation, and therefore the calculation is conducted on the total 'free space' of the environment. 

This code is operational for one tugbot, but has the potential for extrapolation to a multi robot scenario. The code subscripes to odometry data from /RosAria/odom. The camera's radius can be adjusted however the resolution should be kept consistent. 


How to Run:
```bash colcon build```
```bash source install/setup.bash```
```ros2 launch p3dx_gazebo p3dx_warehouse_world.launch.py```
```ros2 launch p3dx_navigation slam_2d_demo.launch.py```
This is the line that runs the actual code (make sure to replace with your specific file path): 
```python3 /home/user/ros2_ws/sim_ws/src/p3dx/p3dx_gazebo/scripts/cover_2.py```
