# px4-offboard
This `repository` contains a python examples for offboard control on ROS2 with [PX4](https://px4.io/)

The `px4_offboard` package contains the following nodes
- `offboard_control.py`: Example of offboard position control using position setpoints
- `visualizer.py`: Used for visualizing vehicle states in Rviz

The source code is released under a BSD 3-Clause license.

- **Author**: Jaeyoung Lim
- **Affiliation**: Autonomous Systems Lab, ETH Zurich

## Setup
Add the repository to the ros2 workspace
```
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git
```

If you are running this on a companion computer to PX4, you will need to build the package on the companion computer directly. 

## Running

### Software in the Loop
You will make use of 3 different terminals to run the offboard demo.

On the first terminal, run a SITL instance from the PX4 Autopilot firmware.
```
make px4_sitl gazebo
```

On the second terminal terminal, run the micro-ros-agent which will perform the mapping between Micro XRCE-DDS and RTPS. So that ROS2 Nodes are able to communicate with the PX4 micrortps_client.
```
micro-ros-agent udp4 --port 8888
```

In order to run the offboard position control example, open a third terminal and run the the node.
This runs two ros nodes, which publishes offboard position control setpoints and the visualizer.
```
ros2 launch px4_offboard offboard_position_control.launch.py
```
![offboard](https://user-images.githubusercontent.com/5248102/194742116-64b93fcb-ec99-478d-9f4f-f32f7f06e9fd.gif)

In order to just run the visualizer,
```
ros2 launch px4_offboard visualize.launch.py
```
### Hardware

This section is intended for running the offboard control node on a companion computer, such as a Raspberry Pi or Nvidia Jetson/Xavier. You will either need an SSH connection to run this node, or have a shell script to run the nodes on start up. 

If you are running this through a UART connection into the USB port, start the micro-ros agent with the following command

```
micro-ros-agent serial --dev /dev/ttyUSB0 -b 921600 -v
```
If you are using a UART connection which goes into the pinouts on the board, start the micro-ros agent with the following comand
```
micro-ros-agent serial --dev /dev/ttyTHS1 -b 921600 -V
```

To run the offboard position control example, run the node on the companion computer
```
ros2 launch px4_offboard offboard_hardware_position_control.launch.py
```

### Visualization parameters

The following section describes ROS2 parameters that alter behavior of visualization tool.


#### Automatic path clearing between consequent simulation runs

Running visualization node separately from the simulation can be desired to iteratively test how variety of PX4 (or custom offboard programs) parameters adjustments influence the system behavior. 

In such case RVIZ2 is left opened and in separate shell simulation is repeadetly restarted. This process causes paths from consequent runs to overlap with incorrect path continuity where end of previous path is connected to the beginning of new path from new run. To prevent that and clear old path automatically on start of new simulation, set `path_clearing_timeout` to positive float value which corresponds to timeout seconds after which, upon starting new simulation, old path is removed.

As an example, setting the parameter to `1.0` means that one second of delay between position updates through ROS2 topic will schedule path clearing right when next position update comes in (effectively upon next simulation run).


To enable automatic path clearing without closing visualization node set the param to positive floating point value:
```
ros2 param set /px4_offboard/visualizer path_clearing_timeout 1.0
```

To disable this feature set timeout to any negative floating point value (the feature is disabled by default):
```
ros2 param set /px4_offboard/visualizer path_clearing_timeout -1.0
```