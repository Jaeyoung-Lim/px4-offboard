
## Overview

This tutorial explains how to interface ROS2 with PX4 (SITL) using DDS.

### Prerequisites

   * [ROS2 Installed](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2), and setup for your operating system (e.g. [Linux Ubuntu](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)) with Gazebo
   * [FastDDS Installed](https://docs.px4.io/main/en/dev_setup/fast-dds-installation.html)
   * [PX4-Autopilot downloaded](https://docs.px4.io/main/en/dev_setup/building_px4.html)
   * [QGroundControl installed](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)

### Example System Details

  * Ubuntu 22.02
  * ROS2 Humble
  * Python 3.10

## Install PX4 Offboard and dependencies (one time setup)

### Install the px4-offboard example from Jaeyoung-Lim

```
cd ~
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git
```

### Install PX4 ROS Com and PX4 msg

The `px4-offboard` example requires the `px4_ros_com` bridge and `px4_msgs` definitions:

```
cd ~
mkdir px4_ros_com_ws
cd px4_ros_com_ws
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
```

Build them:

```
colcon build
```

This should build, and you should see evidence of it building both repos:

```
Starting >>> px4_msgs
Finished <<< px4_msgs [6.42s]                     
Starting >>> px4_ros_com
Finished <<< px4_ros_com [0.13s]                  

Summary: 2 packages finished [6.71s]
```

You may see some warnings interspered with the output.  As long as there are no __*errors*__ you should be OK..

## Install the micro_ros_agent  (one time setup)

#### Check ROS Distro

When building micro_ros_agent, you need to build it for the particular ROS version you are using.  

Ensure your ROS_DISTRO environment variable is set

```
env | grep ROS
```

You should see this variable with some other ROS settings:

```
ROS_DISTRO=humble
```

If it's not set you can try to update:

```
export ROS_DISTRO=humble
```


*Note: If the ROS_DISTRO isn't set in your enviornment you probably don't have a clean ROS2 installation.  You may have issues with the rest of this tutorial.  Recommend reinstalling and checking your ROS2 installation*


#### Building the micro_ros_agent

From:  [Building micro-ROS-Agent](https://github.com/micro-ROS/micro_ros_setup#building-micro-ros-agent)

```
cd ~
mkdir ~/microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git 
mkdir src
mv micro_ros_setup/ src/
```

*Note how the `ROS_DISTRO` environment variable is used to specify the correct branch...

Now build


```
colcon build
source install/local_setup.sh
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```


This should build and may flash a few warnings:

```
colcon build
[0.584s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/analyst/microros_ws/install/micro_ros_setup' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.584s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/analyst/microros_ws/install/micro_ros_agent' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.584s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/analyst/microros_ws/install/micro_ros_msgs' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.585s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/analyst/microros_ws/install/micro_ros_setup' in the environment variable CMAKE_PREFIX_PATH doesn't exist
[0.585s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/analyst/microros_ws/install/micro_ros_agent' in the environment variable CMAKE_PREFIX_PATH doesn't exist
[0.585s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/analyst/microros_ws/install/micro_ros_msgs' in the environment variable CMAKE_PREFIX_PATH doesn't exist
Starting >>> micro_ros_setup
Finished <<< micro_ros_setup [1.52s]                  

Summary: 1 package finished [1.80s]
analyst@casa:~/microros_ws$ source install/local_setup.sh 
analyst@casa:~/microros_ws$ ros2 run micro_ros_setup create_agent_ws.sh
..
=== ./uros/micro-ROS-Agent (git) ===
Cloning into '.'...
=== ./uros/micro_ros_msgs (git) ===
Cloning into '.'...
#All required rosdeps installed successfully
analyst@casa:~/microros_ws$ ros2 run micro_ros_setup build_agent.sh
Building micro-ROS Agent
[0.593s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/analyst/microros_ws/install/micro_ros_agent' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.593s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/analyst/microros_ws/install/micro_ros_msgs' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.593s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/analyst/microros_ws/install/micro_ros_agent' in the environment variable CMAKE_PREFIX_PATH doesn't exist
[0.593s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/analyst/microros_ws/install/micro_ros_msgs' in the environment variable CMAKE_PREFIX_PATH doesn't exist
Starting >>> micro_ros_msgs
Finished <<< micro_ros_msgs [5.51s]                     
Starting >>> micro_ros_agent
--- stderr: micro_ros_agent                                
Cloning into 'xrceagent'...
Switched to a new branch 'ros2'
HEAD is now at 3eb56b5 Release v2.3.0
CMake Warning (dev) at /usr/share/cmake-3.22/Modules/FindPackageHandleStandardArgs.cmake:438 (message):
  The package name passed to `find_package_handle_standard_args` (tinyxml2)
  does not match the name of the calling package (TinyXML2).  This can lead
  to problems in calling code that expects `find_package` result variables
  (e.g., `_FOUND`) to follow a certain pattern.
Call Stack (most recent call first):
  cmake/modules/FindTinyXML2.cmake:40 (find_package_handle_standard_args)
  /opt/ros/humble/share/fastrtps/cmake/fastrtps-config.cmake:51 (find_package)
  CMakeLists.txt:153 (find_package)
This warning is for project developers.  Use -Wno-dev to suppress it.

---
Finished <<< micro_ros_agent [23.2s]

Summary: 2 packages finished [29.0s]
  1 package had stderr output: micro_ros_agent

```

Try running the agent:

```
source install/local_setup.sh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

It should respond with:

```
[1670101284.566587] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
[1670101284.566822] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
```

## Run the Demo

### Overview

You will need 4 terminal windows  ([gnome terminator](https://gnome-terminator.org/) is a great for this!), one for each of the following components:

   * micro-ros-agent
   * Gazebo   
   * px4-offboard example
   * QGroundControl

In each terminal window, we will first source any workspace settings required for a particular component and also set the `ROS_DOMAIN_ID` and `PYTHONOPTIMIZE`  in each window

### Start the micro-ros-agent

In the terminal you designated as the `micro-ros-agent` terminal:

```
cd ~/microros_ws
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0
```

This should start with a few initial messages:

```
[1680365404.306615] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
[1680365404.306837] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
```

### Start Gazebo

In the terminal you designated for gazebo:

```
cd ~/PX4-Autopilot
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
make px4_sitl gazebo
```

Gazebo should start and you will see a big PX4 ascii art banner in the gazebo terminal and the GUI will launch.

Back in the  `micro-ros-agent` terminal, you should see the Micro ROS Agent start to receive DDS messages:


```
[1680365951.442844] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0xAAAABBBB, topic_id: 0x0C7(2), participant_id: 0x001(1)
[1680365951.442905] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0xAAAABBBB, publisher_id: 0x0C7(3), participant_id: 0x001(1)
[1680365951.443306] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0xAAAABBBB, datawriter_id: 0x0C7(5), publisher_id: 0x0C7(3)
[1680365956.248671] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0xAAAABBBB, topic_id: 0x0CE(2), participant_id: 0x001(1)
[1680365956.248727] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0xAAAABBBB, publisher_id: 0x0CE(3), participant_id: 0x001(1)
[1680365956.249012] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0xAAAABBBB, datawriter_id: 0x0CE(5), publisher_id: 0x0CE(3)
```

Leave the agent running for the rest of the tutorial

If you scroll up in the Gazebo terminal window, you should see logs indicating it set up the microdds_client:

```
INFO  [microdds_client] synchronized with time offset 1680366210257897us
INFO  [microdds_client] successfully created rt/fmu/out/failsafe_flags data writer, topic id: 79
INFO  [microdds_client] successfully created rt/fmu/out/sensor_combined data writer, topic id: 162
INFO  [microdds_client] successfully created rt/fmu/out/timesync_status data writer, topic id: 182
INFO  [microdds_client] successfully created rt/fmu/out/vehicle_control_mode data writer, topic id: 205
INFO  [microdds_client] successfully created rt/fmu/out/vehicle_status data writer, topic id: 222
INFO  [microdds_client] successfully created rt/fmu/out/vehicle_gps_position data writer, topic id: 208
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
INFO  [microdds_client] successfully created rt/fmu/out/vehicle_local_position data writer, topic id: 212
INFO  [microdds_client] successfully created rt/fmu/out/vehicle_odometry data writer, topic id: 217
```

### Start QGround Controller and Take Off

In the terminal you designated as the `QGroundControl` terminal, launch the app:

```
cd /dir/where/qgroundcontroller/is/installed
./QGroundControl.AppImage
```

Click `Takeoff` from left hand menum, then slide to confirm

The simulated drone should takeoff and climb to an altitude of 10m (~32ft)


### Start the px4-offboard example

#### Check ROS Messages

Before we start the example, lets check ROS2 topics.

In the window you have designated for the `px4-offboard` example:


```
cd ~/px4-offboard
source ../px4_ros_com_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
ros2 topic list
```

You should see a list of topics that match the ones sent from Gazebo.:

```
/fmu/in/obstacle_distance
/fmu/in/offboard_control_mode
/fmu/in/onboard_computer_status
/fmu/in/sensor_optical_flow
/fmu/in/telemetry_status
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_attitude_setpoint
/fmu/in/vehicle_command
/fmu/in/vehicle_mocap_odometry
/fmu/in/vehicle_rates_setpoint
/fmu/in/vehicle_trajectory_bezier
/fmu/in/vehicle_trajectory_waypoint
/fmu/in/vehicle_visual_odometry
/fmu/out/failsafe_flags
/fmu/out/sensor_combined
/fmu/out/timesync_status
/fmu/out/vehicle_attitude
/fmu/out/vehicle_control_mode
/fmu/out/vehicle_global_position
/fmu/out/vehicle_gps_position
/fmu/out/vehicle_local_position
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status
/parameter_events
/rosout
```

If you do not see the topics:

  * Check that `ROS_DOMAIN_ID=0`  in all the terminals

Let's echo a topic:

```
ros2 topic echo /fmu/out/vehicle_odometry
```

The terminal should echo some rapidly updating details about the simulated drone.  If you look at position, you can see it matches the height of our drone:

```
timestamp: 1680367557988752
timestamp_sample: 1680367557988752
pose_frame: 1
position:
- 0.023736324161291122
- -0.007955201901495457
- -9.922133445739746
q:
- 0.6887969374656677
- 0.002538114320486784
- -0.007746106944978237
- 0.7249085307121277
velocity_frame: 1
velocity:
- -0.007310825865715742
- -0.032901208847761154
- 0.010859847068786621
angular_velocity:
- -0.0012153536081314087
- -0.01004723273217678
- -0.0018821069970726967
position_variance:
- 0.12015653401613235
- 0.1201564222574234
- 0.11353325098752975
orientation_variance:
- 0.00042706530075520277
- 0.00021347538859117776
- 0.000316519319312647
velocity_variance:
- 0.014987492933869362
- 0.01498822309076786
- 0.014155800454318523
reset_counter: 7
quality: 0
```

*In this case the `-9.922133445739746` value indicates the drone in the gazebo sim is in the air.*

Now that we verfied the DDS-ROS subscription communication link, we can start the demo

CTRL-C to stop the topic echo and then:

```
source ../px4_ros_com_ws/install/setup.bash
source install/setup.bash
```

The first source reensures the dependencies are loaded for the demo.
The second is for the demo itself.

Now launch the demo:

```
ros2 launch px4_offboard offboard_position_control.launch.py
```

If things work, the demo should immediately launch an RViz window with the 3D axis indicator (red, green blue color) at the top of the window above the grid.  This indicates the drone's position in the gazebo sim/

The terminal should show:

```
[INFO] [launch]: All log files can be found below /home/analyst/.ros/log/2023-04-01-16-50-55-835808-casa-32775
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [visualizer-1]: process started with pid [32776]
[INFO] [offboard_control-2]: process started with pid [32778]
[INFO] [rviz2-3]: process started with pid [32780]
[rviz2-3] Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
[offboard_control-2] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT is deprecated. Use ReliabilityPolicy.BEST_EFFORT instead.
[offboard_control-2]   warnings.warn(
[offboard_control-2] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL is deprecated. Use DurabilityPolicy.TRANSIENT_LOCAL instead.
[offboard_control-2]   warnings.warn(
[offboard_control-2] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST is deprecated. Use HistoryPolicy.KEEP_LAST instead.
[offboard_control-2]   warnings.warn(
[rviz2-3] [INFO] [1680367856.767339081] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-3] [INFO] [1680367856.767961910] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[visualizer-1] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT is deprecated. Use ReliabilityPolicy.BEST_EFFORT instead.
[visualizer-1]   warnings.warn(
[visualizer-1] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST is deprecated. Use HistoryPolicy.KEEP_LAST instead.
[visualizer-1]   warnings.warn(
[rviz2-3] [INFO] [1680367856.847242023] [rviz2]: Stereo is NOT SUPPORTED
```

Now head back to QGroundControl and enable offboard control.  Click the current mode "HOLD" in upper left, then in the menu, select `Offboard`:

After a 1-2 sec pause, the demo should take control and you should see the 3d indicator in Rviz drawing circles.


