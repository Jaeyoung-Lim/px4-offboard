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

## Running
Run SITL instance using from the PX4 Autopilot firmware.
> :warning: Currently need to be run from this [Pull Request](https://github.com/PX4/PX4-Autopilot/pull/20227)

```
make px4_sitl gazebo
```

On a different terminal, run the micro ros agent
```
micro-ros-agent udp4 --port 8888
```
In order to run the offboard position control example, open another terminal and run the the node.
This runs two ros nodes, which publishes offboard position control setpoints and the visualizer.
```
ros2 launch px4_offboard offboard_position_control.launch.py
```
![offboard](https://user-images.githubusercontent.com/5248102/194742116-64b93fcb-ec99-478d-9f4f-f32f7f06e9fd.gif)

In order to just run the visualizer,
```
ros2 launch px4_offboard visualize.launch.py
```
