# offboard_cpp
This `package` contains a c++ example for offboard control on ROS2 with [PX4](https://px4.io/).The source code is released under a BSD 3-Clause license.


## Instructions

### Setup
Add this `repository` to your RO2 workspace.
 ```
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git
```

Make sure that the newest versions of [px4_msgs](https://github.com/PX4/px4_msgs) and [px4_ros_com](https://github.com/PX4/px4_ros_com) are in the same workspace.

 ```
git clone --recursive https://github.com/PX4/px4_ros_com.git
git clone --recursive https://github.com/PX4/px4_msgs.git
```

In a different folder, outside of your ros2 workspace clone the  [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot). At the time of writing this was commit [f2607335ac](https://github.com/PX4/PX4-Autopilot/commit/f2607335ac80dbd0fe151ba8fde122e91d776fc7).

 ```
git clone --recursive https://github.com/PX4/PX4-Autopilot.git

```

Finally you need to make sure that you have installed the `micro-ros-agent`. You can use `snap` to install it or [build it from source](https://github.com/micro-ROS/micro_ros_setup#building-micro-ros-agent).

### Build
1. Go to your ros2 workspace
2. Source your ros2 installation. For me this was:
```
source /opt/ros/galactic/setup.zsh 
```
3. Build your workspace, which includes the 4 packages (px4_msgs, px4_ros_com and the two offboard packages from this repo..
```
colcon build --symlink-install
```
### Run
1. Open a new terminal and go to your PX4-Autopilot folder. Run:
```
make px4_sitl gazebo
```

2. Open a new terminal and source your ros2 workspace. Then run the micro-ros-agent:
```
micro-ros-agent udp4 --port 8888
```

3. Open a new terminal and source your ros2 workspace. Then run the c++ offboard node:
```
ros2 run offboard_cpp offboard_cpp_node
```

4. The SITL drone should have taken-off.

### Troubleshoot
* I don't know why but sometimes the offboard node does not manage to change the `NavState` of the SITL drone to offboard (14) and it remains on hold mode (4). To change that use QGC or use your nutt-shell:
```
commander mode offboard
```
* Once the offboard mode is set the SITL drone must be armed using QGC or the nutt-shell:
```
commander arm
```

