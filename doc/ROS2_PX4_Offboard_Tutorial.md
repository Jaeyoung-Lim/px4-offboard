
## Overview

This tutorial explains how to interface ROS2 with PX4 (SITL) using DDS.

### Prerequisites

   * [ROS2 Installed](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2), and setup for your operating system (e.g. [Linux Ubuntu](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)) with Gazebo
   * [FastDDS Installed](https://docs.px4.io/v1.13/en/dev_setup/fast-dds-installation.html#fast-dds-installation)
   * [PX4-Autopilot downloaded](https://docs.px4.io/main/en/dev_setup/building_px4.html)
   * [QGroundControl installed](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
   * Ubuntu 22.04
   * ROS2 Humble
   * Python 3.10

## Install PX4 Offboard and dependencies (one time setup)

### Install the px4-offboard example from Jaeyoung-Lim

```
cd ~
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git
```

### Install PX4 msg

The `px4-offboard` example requires `px4_msgs` definitions:

```
mkdir -p ~/px4_ros_com_ws/src && cd ~/px4_ros_com_ws/src
git clone https://github.com/PX4/px4_msgs.git
```

Build:

```
colcon build
```

This should build. You may see some warnings interspered with the output.  As long as there are no __*errors*__ you should be OK..

## Install the micro_ros_agent  (one time setup)
Follow these instructions to build the micro_ros_setup:  [Building micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup#building)
```
source /opt/ros/$ROS_DISTRO/setup.bash

mkdir microros_ws && cd microros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep update && rosdep install --from-paths src --ignore-src -y

colcon build

source install/local_setup.bash
```

Follow these instructions to build the micro_ros_agent:  [Building micro-ROS-Agent](https://github.com/micro-ROS/micro_ros_setup#building-micro-ros-agent)
```
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
```

Try running the agent (assuming the agent is installed at `~/microros_ws`):

```
cd ~/microros_ws
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

Now head back to QGroundControl and enable offboard control.  Click the current mode "HOLD" in upper left, then in the menu, select "Offboard":

After a 1-2 sec pause, the demo should take control and you should see the 3d indicator in Rviz drawing circles.
