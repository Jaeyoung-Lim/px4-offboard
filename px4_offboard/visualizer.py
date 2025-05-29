#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

class PX4Visualizer(Node):
    def __init__(self):
        super().__init__("visualizer")

        # Declare and retrieve the namespace parameter
        self.declare_parameter('namespace', '')  # Default to empty namespace
        self.namespace = self.get_parameter('namespace').value
        self.namespace_prefix = f'/{self.namespace}' if self.namespace else ''

        # QoS profiles
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            f"{self.namespace_prefix}/fmu/out/vehicle_attitude",
            self.vehicle_attitude_callback,
            qos_profile_sub,
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f"{self.namespace_prefix}/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            qos_profile_sub,
        )
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            f"{self.namespace_prefix}/fmu/in/trajectory_setpoint",
            self.trajectory_setpoint_callback,
            qos_profile_sub,
        )

        self.vehicle_pose_pub = self.create_publisher(
            PoseStamped, f"{self.namespace_prefix}/px4_visualizer/vehicle_pose", 10
        )
        self.vehicle_vel_pub = self.create_publisher(
            Marker, f"{self.namespace_prefix}/px4_visualizer/vehicle_velocity", 10
        )
        self.vehicle_path_pub = self.create_publisher(
            Path, f"{self.namespace_prefix}/px4_visualizer/vehicle_path", 10
        )
        self.setpoint_path_pub = self.create_publisher(
            Path, f"{self.namespace_prefix}/px4_visualizer/setpoint_path", 10
        )

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_path_msg = Path()
        self.setpoint_path_msg = Path()

        # trail size
        self.trail_size = 1000

        # time stamp for the last local position update received on ROS2 topic
        self.last_local_pos_update = 0.0
        # time after which existing path is cleared upon receiving new
        # local position ROS2 message
        self.declare_parameter("path_clearing_timeout", -1.0)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

    def vector2PoseMsg(self, frame_id, position, attitude):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.orientation.w = attitude[0]
        pose_msg.pose.orientation.x = attitude[1]
        pose_msg.pose.orientation.y = attitude[2]
        pose_msg.pose.orientation.z = attitude[3]
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        return pose_msg

    def vehicle_attitude_callback(self, msg):
        # NED-> ENU transformation
        # Receives quaternion in NED frame as (qw, qx, qy, qz)
        q_enu = 1/np.sqrt(2) * np.array([msg.q[0] + msg.q[3], msg.q[1] + msg.q[2], msg.q[1] - msg.q[2], msg.q[0] - msg.q[3]])
        q_enu /= np.linalg.norm(q_enu)
        self.vehicle_attitude = q_enu.astype(float)

    def vehicle_local_position_callback(self, msg):
        path_clearing_timeout = (
            self.get_parameter("path_clearing_timeout")
            .get_parameter_value()
            .double_value
        )
        if path_clearing_timeout >= 0 and (
            (self.get_clock().now() / 1e9 - self.last_local_pos_update)
            > path_clearing_timeout
        ):
            self.vehicle_path_msg.poses.clear()
        self.last_local_pos_update = Clock().now().nanoseconds / 1e9

        # TODO: handle NED->ENU transformation
        self.vehicle_local_position[0] = msg.y
        self.vehicle_local_position[1] = msg.x
        self.vehicle_local_position[2] = -msg.z
        self.vehicle_local_velocity[0] = msg.vy
        self.vehicle_local_velocity[1] = msg.vx
        self.vehicle_local_velocity[2] = -msg.vz

    def trajectory_setpoint_callback(self, msg):
        self.setpoint_position[0] = msg.position[1]
        self.setpoint_position[1] = msg.position[0]
        self.setpoint_position[2] = -msg.position[2]

    def create_arrow_marker(self, id, tail, vector):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = "map"
        # msg.header.stamp = Clock().now().nanoseconds / 1000
        msg.ns = "arrow"
        msg.id = id
        msg.type = Marker.ARROW
        msg.scale.x = 0.1
        msg.scale.y = 0.2
        msg.scale.z = 0.0
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.0
        msg.color.a = 1.0
        dt = 0.3
        tail_point = Point()
        tail_point.x = tail[0]
        tail_point.y = tail[1]
        tail_point.z = tail[2]
        head_point = Point()
        head_point.x = tail[0] + dt * vector[0]
        head_point.y = tail[1] + dt * vector[1]
        head_point.z = tail[2] + dt * vector[2]
        msg.points = [tail_point, head_point]
        return msg

    def append_vehicle_path(self, msg):
        self.vehicle_path_msg.poses.append(msg)
        if len(self.vehicle_path_msg.poses) > self.trail_size:
            del self.vehicle_path_msg.poses[0]

    def append_setpoint_path(self, msg):
        self.setpoint_path_msg.poses.append(msg)
        if len(self.setpoint_path_msg.poses) > self.trail_size:
            del self.setpoint_path_msg.poses[0]

    def cmdloop_callback(self):
        vehicle_pose_msg = self.vector2PoseMsg(
            "map", self.vehicle_local_position, self.vehicle_attitude
        )
        self.vehicle_pose_pub.publish(vehicle_pose_msg)

        # Publish time history of the vehicle path
        self.vehicle_path_msg.header = vehicle_pose_msg.header
        self.append_vehicle_path(vehicle_pose_msg)
        self.vehicle_path_pub.publish(self.vehicle_path_msg)

        # Publish time history of the vehicle path
        setpoint_pose_msg = self.vector2PoseMsg("odom", self.vehicle_local_position, self.vehicle_attitude)
        self.setpoint_path_msg.header = setpoint_pose_msg.header
        self.append_setpoint_path(setpoint_pose_msg)
        self.setpoint_path_pub.publish(self.setpoint_path_msg)

        # Publish arrow markers for velocity
        velocity_msg = self.create_arrow_marker(1, self.vehicle_local_position, self.vehicle_local_velocity)
        self.vehicle_vel_pub.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)

    px4_visualizer = PX4Visualizer()

    rclpy.spin(px4_visualizer)

    px4_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
