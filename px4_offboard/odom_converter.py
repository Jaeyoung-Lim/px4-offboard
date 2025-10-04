#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from scipy.spatial.transform import Rotation as R
import numpy as np

class OdomConverter(Node):
    def __init__(self):
        super().__init__('odom_converter')

        # Ensure sim time is used
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )])

        # Subscribe to PX4 odometry (NED frame)
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.listener_callback,
            qos_profile_sensor_data
        )

        # Publish ROS odometry (ENU frame)
        self.publisher = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Frame names
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'

        # =====================================================
        # COORDINATE TRANSFORMATION SETTINGS
        # =====================================================
        # PX4 uses NED frame: X=North, Y=East, Z=Down; Body: FRD (Forward-Right-Down)
        # ROS uses ENU frame: X=East, Y=North, Z=Up; Body: FLU (Forward-Left-Up)
        
        # World frame transformation (NED to ENU)
        self.world_transform = np.array([
            [0, 1, 0],  # X_enu = Y_ned (East)
            [1, 0, 0],  # Y_enu = X_ned (North)
            [0, 0, -1]  # Z_enu = -Z_ned (Up)
        ])

        # Body frame transformation (FRD to FLU)
        self.body_transform = np.array([
            [1, 0, 0],   # X_flu = X_frd (Forward)
            [0, -1, 0],  # Y_flu = -Y_frd (Left = -Right)
            [0, 0, -1]   # Z_flu = -Z_frd (Up = -Down)
        ])

        # For rate limiting
        self.last_publish_time = self.get_clock().now()
        self.publish_rate_hz = 30  # Throttle to 30 Hz

    def create_covariance_matrix(self, diagonal_values):
        """Create a proper 6x6 covariance matrix with only diagonal elements."""
        cov = np.zeros((6, 6))
        np.fill_diagonal(cov, diagonal_values)
        return cov.flatten().tolist()

    def transform_vector(self, px4_vector, transform_matrix):
        """Transform a 3D vector using the given matrix."""
        return transform_matrix @ px4_vector

    def transform_orientation(self, px4_quaternion):
        """Transform orientation from PX4 NED/FRD to ROS ENU/FLU."""
        # Convert PX4 quaternion (w, x, y, z) to scipy format (x, y, z, w)
        q_scipy = [px4_quaternion[1], px4_quaternion[2], px4_quaternion[3], px4_quaternion[0]]
        
        # Get rotation matrix from quaternion
        r_ned_frd = R.from_quat(q_scipy)
        R_ned_frd = r_ned_frd.as_matrix()
        
        # Apply transformation: R_enu_flu = world_transform @ R_ned_frd @ body_transform
        R_enu_flu = self.world_transform @ R_ned_frd @ self.body_transform
        
        # Convert back to quaternion
        r_enu_flu = R.from_matrix(R_enu_flu)
        q_enu = r_enu_flu.as_quat()  # [x, y, z, w]
        
        # Ensure quaternion has positive w (convention)
        if q_enu[3] < 0:
            q_enu = [-x for x in q_enu]
        
        return q_enu

    def listener_callback(self, msg: VehicleOdometry):
        current_time = self.get_clock().now()
        if (current_time - self.last_publish_time).nanoseconds / 1e9 < 1.0 / self.publish_rate_hz:
            return
        self.last_publish_time = current_time

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # --- Position transformation (NED to ENU) ---
        position_ned = np.array([msg.position[0], msg.position[1], msg.position[2]])
        position_enu = self.transform_vector(position_ned, self.world_transform)
        odom.pose.pose.position.x = float(position_enu[0])
        odom.pose.pose.position.y = float(position_enu[1])
        odom.pose.pose.position.z = float(position_enu[2])

        # --- Orientation transformation (NED/FRD to ENU/FLU) ---
        q_enu = self.transform_orientation(msg.q)
        odom.pose.pose.orientation.x = q_enu[0]
        odom.pose.pose.orientation.y = q_enu[1]
        odom.pose.pose.orientation.z = q_enu[2]
        odom.pose.pose.orientation.w = q_enu[3]

        # --- Velocity transformation (based on velocity_frame) ---
        velocity_ned = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
        if msg.velocity_frame == 1:  # VELOCITY_FRAME_NED
            velocity_enu = self.transform_vector(velocity_ned, self.world_transform)
        elif msg.velocity_frame == 3:  # VELOCITY_FRAME_BODY_FRD
            velocity_enu = self.transform_vector(velocity_ned, self.body_transform)
        else:
            velocity_enu = velocity_ned  # Fallback, though unlikely
        odom.twist.twist.linear.x = float(velocity_enu[0])
        odom.twist.twist.linear.y = float(velocity_enu[1])
        odom.twist.twist.linear.z = float(velocity_enu[2])

        # --- Angular velocity transformation (FRD to FLU) ---
        av_frd = np.array(msg.angular_velocity)
        av_flu = self.transform_vector(av_frd, self.body_transform)
        odom.twist.twist.angular.x = float(av_flu[0])
        odom.twist.twist.angular.y = float(av_flu[1])
        odom.twist.twist.angular.z = float(av_flu[2])

        # --- Covariances ---
        pose_diagonal = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        twist_diagonal = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        
        odom.pose.covariance = self.create_covariance_matrix(pose_diagonal)
        odom.twist.covariance = self.create_covariance_matrix(twist_diagonal)

        # Publish /odom
        self.publisher.publish(odom)

        # Broadcast TF (odom → base_link)
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()