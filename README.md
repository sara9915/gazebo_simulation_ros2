# Motoman Yaskawa Gazebo Simulation with ROS2 
This is a ROS2 package to visualize and simulate the Yaskawa SIA5F robot, equipped with a WSG-32 gripper with tactile extensions and a Realsense D435i camera. Both simulation on Gazebo and visualization on Rviz are available. 

## Installing
Follow the steps below to set up the package. The suggested ROS2 version is ros-humble on Ubuntu 22.04.

1. **Add the repository to your ROS2 workspace**
    ```bash
    cd ~/my_ros2_ws/src
    git clone https://github.com/sara9915/uclv_yaskawa_simulation.git
    ```

2. **Install ROS dependencies**

   For ROS2 Humble (otherwise, change the version using ${ROS_DISTRO})
    ```
    sudo apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
    sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
    ```
    The packages listed in `https.rosinstall` have to be installed. You can use `wstool` as follows:
   ```bash
    # In the src of your ros2 ws
    wstool init #if not already initialized
    wstool merge https://raw.githubusercontent.com/sara9915/uclv_yaskawa_simulation/main/https.rosinstall
    wstool update
    ```
   
    If you prefer to clone via ssh, merge with
    ```bash
    wstool merge https://raw.githubusercontent.com/sara9915/uclv_yaskawa_simulation/main/ssh.rosinstall
    ```

4. **Build**
    ```bash
    cd ~/my_ros2_ws
    colcon build --symlink-install

    ```

5. **How to launch twist controller**
    ```bash
    ros2 launch uclv_moveit_planner_ros2 visualize.launch simulation:=true
    ros2 launch uclv_moveit_planner_ros2 planner_srv.launch simulation:=true
    ros2 run uclv_yaskawa_simulation twist_ref_pub
    
    ```


