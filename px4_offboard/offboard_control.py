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

#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus


class OffboardControl(Node):
    def __init__(self):
        super().__init__("offboard_control")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(VehicleStatus,"/fmu/out/vehicle_status",self.vehicle_status_callback,qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.theta = 0.0
        self.radius = 10.0
        self.omega = 0.5
        self.arming_state = 0.0

        self.counter = 0

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg=msg)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand.ARMING_ACTION_ARM,
        )
        self.get_logger().info("Arming vehicle!")

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand.ARMING_ACTION_DISARM,
        )
        self.get_logger().info("Disarming vehicle!")


    def set_mode_offboard(self):
        self.get_logger().info("Setting mode to offboard!")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    def publish_offboard_control_mode(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

    def publish_trajectory_setpoint(self):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position[0] = self.radius * np.cos(self.theta)
        trajectory_msg.position[1] = self.radius * np.sin(self.theta)
        trajectory_msg.position[2] = -5.0
        trajectory_msg.yaw = 0.0
        self.publisher_trajectory.publish(trajectory_msg)
        self.theta = self.theta + self.omega * self.dt

    def cmdloop_callback(self):
        # Publish offboard control modes
        self.publish_offboard_control_mode()
        if self.arming_state == 0.0:
            self.get_logger().debug("Vehicle is unarmed!")
            # self.arm()
        elif self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_trajectory_setpoint()


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
