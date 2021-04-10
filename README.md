# Pioneer-3dx Simulator

Pioneer-3dx robot has been [discontinued](https://www.generationrobots.com/en/402395-robot-mobile-pioneer-3-dx.html), however, many universities and institutes are still using them for experiments. To facilitate the simulation in Gazebo for pioneer-3dx, I have decided to make some changes to [sidneyrdc](https://github.com/sidneyrdc/p3dx_gazebo) p3dx_gazebo. The purpose is to make it better and easier for other users to utilize it. I am aware that there are already many pioneer-3dx gazebo models out there, yet, they are not very well structured. Therefore, my attempt is to follow the [husky](https://github.com/husky) simulation structure to re-structure the simulation for pioneer-3dx.

## Multi Robots

Use the launch files starting with `multi` in `p3dx_gazebo` to launch multiple pioneer robots in gazebo. Note that the namespace should not have any numbers like `1` but use alphabets instead.

## Reference

- Robot Namespace:   
- Diff_drive_controller parameters (gazebo plugin): http://wiki.ros.org/diff_drive_controller  
- Multi robot references: https://answers.ros.org/question/267332/how-to-change-the-robot_description-paramter-for-the-joint_trajectory_controller/https://answers.ros.org/question/263415/simplest-multiple-robot-scenario/ https://answers.gazebosim.org//question/16497/spawning-multiple-robots-each-with-a-controller-in-the-same-namespace-as-the-controller_manager/  
- libgazebo_ros_joint_state_publisher: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/melodic-devel/gazebo_plugins/src/gazebo_ros_joint_state_publisher.cpp  
- tf prefix: look at control.launch for more information