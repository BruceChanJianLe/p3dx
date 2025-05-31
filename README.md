# Pioneer-3dx Simulator

> Pioneer-3dx robot has been [discontinued](https://www.generationrobots.com/en/402395-robot-mobile-pioneer-3-dx.html),
> however, many universities and institutes are still using them for experiments.
> To facilitate the simulation in Gazebo for pioneer-3dx, I have decided to make some changes to
> [sidneyrdc](https://github.com/sidneyrdc/p3dx_gazebo) p3dx_gazebo.
> The purpose is to make it better and easier for other users to utilize it.
> I am aware that there are already many pioneer-3dx gazebo models out there, yet,
> they are not very well structured. Therefore, my attempt is to follow the [husky](https://github.com/husky)
> simulation structure to re-structure the simulation for pioneer-3dx.

## Build

Create new workspace and clone p3dx package.  
```bash
mkdir sim_ws/src -p
cd sim_ws/src
git clone git@github.com:BruceChanJianLe/p3dx.git
```

Install dependencies and build it!  
```bash
cd sim_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Usage

Start the simulation  
```bash
ros2 launch p3dx_gazebo p3dx_warehouse_world.launch.py
```

Start amcl demo  
``` bash
ros2 launch p3dx_navigation amcl_demo.launch.py
```

Start slam 2d demo, note that you can either launch amcl demo or slam 2d demo. But not both.  
``` bash
ros2 launch p3dx_navigation slam_2d_demo.launch.py
```

Start visualization for p3dx  
``` bash
ros2 launch p3dx_viz view_robot.launch.py
```

## Multi Robots

Great news multi-robot is now supported!!!  
However, only `amcl_demo.launch` is supported, the `slam_2d_demo.launch` is still on its way.  


Use the launch files starting with `multi` in `p3dx_gazebo` to launch multiple pioneer robots in gazebo.  
Note that the robot_namespace should not have any numbers like `1` but use alphabets instead.  

Starting multi-robot simulation  
```bash
ros2 launch p3dx_gazebo multi_p3dx_warehouse_world.launch.py
```

Starting amcl demo for pioneer_a  
``` bash
ros2 launch p3dx_navigation amcl_demo.launch.py robot_namespace:=pioneer_a
```

Starting amcl demo for pioneer_b  
``` bash
ros2 launch p3dx_navigation amcl_demo.launch.py robot_namespace:=pioneer_b
```

start visualization for pionner_a  
``` bash
ros2 launch p3dx_viz view_robot.launch.py robot_namespace:=pioneer_a
```

start visualization for pionner_b  
``` bash
ros2 launch p3dx_viz view_robot.launch.py robot_namespace:=pioneer_b
```

## TODO

- Fix Warning
```bash
Warning [Utils.cc:132] [/sdf/model[@name="pioneer3dx"]/link[@name="base_link"]/sensor[@name="front_camera"]/gz_frame_id:<urdf-string>:L0]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF. Copying[gz_frame_id] as children of [sensor].
```

## Reference

- Robot Namespace:   
- Diff_drive_controller parameters (gazebo plugin): http://wiki.ros.org/diff_drive_controller  
- Multi robot references: https://answers.ros.org/question/267332/how-to-change-the-robot_description-paramter-for-the-joint_trajectory_controller/https://answers.ros.org/question/263415/simplest-multiple-robot-scenario/ https://answers.gazebosim.org//question/16497/spawning-multiple-robots-each-with-a-controller-in-the-same-namespace-as-the-controller_manager/  
- libgazebo_ros_joint_state_publisher: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/melodic-devel/gazebo_plugins/src/gazebo_ros_joint_state_publisher.cpp  
- tf prefix: look at control.launch for more information
