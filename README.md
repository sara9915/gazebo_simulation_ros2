# Motoman Yaskawa Gazebo Simulation with ROS2 
## Installing
Follow the steps below to set up the package. The suggested ROS2 version is ros-humble on Ubuntu 22.04.

1. **Add the repository to your ROS2 workspace**
    ```
    $ cd ~/my_ros2_ws/src
    $ git clone https://github.com/sara9915/gazebo_simulation_ros2.git
    ```

2. **Install ROS dependencies**
    ```
    $ sudo apt install ros-${ROS_DISTRO}-joint-state-publisher ros-${ROS_DISTRO}-joint-state-publisher-gui gazebo-ros2-control ros2_controllers ros2_control
    
    ```
    The packages listed in `https.rosinstall` have to be installed. You can use `wstool` as follows:
   ```bash
    # In the src of your ros2 ws
    wstool init #if not already initialized
    wstool merge https://raw.githubusercontent.com/sara9915/ros2_gazebo_simulation/main/https.rosinstall
    wstool update
    ```

3. **Build**
    ```
    $ cd ~/my_ros2_ws
    $ colcon build --symlink-install
    ```


