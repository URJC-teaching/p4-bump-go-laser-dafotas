# Copyright 2025 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

from rclpy.duration import Duration
import time

class BumpGoLaserNode(Node):
    def __init__(self):
        super().__init__('bump_go_laser_node')

        self.declare_parameter('min_distance', 0.5)
        self.declare_parameter('base_frame', 'base_footprint')

        self.min_distance = self.get_parameter('min_distance').value
        self.base_frame = self.get_parameter('base_frame').value

        self.get_logger().info(f'BumpGoLaserNode set to {self.min_distance:.2f} m')

        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan_filtered',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = 'init'
        self.distance_obstacle = 10.0
        self.angle_obstacle = 0.0

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.turning_time = Duration(seconds = 1.0)

        self.state_ts = self.get_clock().now()

    def laser_callback(self, scan: LaserScan):

        #self.get_logger().debug(f'Received LaserScan with {len(scan.ranges)} ranges')
        if not scan.ranges:
            #self.get_logger().debug('No laser measurements received')
            return
        
        ranges = [r if math.isfinite(r) else float('inf') for r in scan.ranges] # all NaN to inf so they are ignored by min()
        if not ranges:
            self.get_logger().debug('No valid laser measurements after filtering')
            return
        
        distance_min = min(ranges) # closest obstacle
        min_idx = ranges.index(distance_min)

        #self.get_logger().info(f'Min distance from laser: {distance_min:.2f} m at index {min_idx} Min distance defined at beginning: {self.min_distance:.2f} m')

        if distance_min < self.min_distance and distance_min > 0.34: # if obstacle detected within min_distance

            angle = scan.angle_min + scan.angle_increment * min_idx # relative to the sensor frame
            x = distance_min * math.cos(angle)
            y = distance_min * math.sin(angle)

            pt = PointStamped()
            pt.header = scan.header
            pt.point.x = x
            pt.point.y = y
            pt.point.z = 0.0

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    scan.header.frame_id,
                    rclpy.time.Time()
                ) # get latest available transform between the laser frame and the base frame
                pt_base = do_transform_point(pt, transform) # transform point to base frame

                angle_base = math.atan2(pt_base.point.y, pt_base.point.x)
                distance_base = math.hypot(pt_base.point.x, pt_base.point.y)
                self.get_logger().info(f'Obstacle @ {self.base_frame}: x={angle_base:.2f} rad, distance={distance_base:.2f} m')
                #self.get_logger().info(
                #    f'Obstacle @ {self.base_frame}: x={pt_base.point.x:.2f}, y={pt_base.point.y:.2f}, '
                #    f'distance={distance_base:.2f} m, angle={math.degrees(angle_base):.2f} deg'
                #)

                self.distance_obstacle = distance_base
                self.angle_obstacle = angle_base

            except Exception as e:
                self.get_logger().warn(f'No TF from {scan.header.frame_id} to {self.base_frame}: {e}')
            
        else:
            #self.get_logger().debug(f'No obstacle closer than {self.min_distance:.2f} m (min={distance_min:.2f} m)')
            if (self.get_clock().now() - self.state_ts) > self.turning_time:
                self.state = 'forward'
                self.distance_obstacle = 10.0

    def control_loop(self):
        if self.state == 'init':
            self.state = 'forward'

        if (self.angle_obstacle >= -math.pi / 2) and (self.angle_obstacle <= -math.pi / 12):
            self.state = 'right_obstacle'
            self.get_logger().info("Obstáculo detectado a la derecha (entre -90º y -15º).")

        elif (self.angle_obstacle > -math.pi / 12) and (self.angle_obstacle < math.pi / 12):
            self.state = 'center_obstacle'
            self.get_logger().info(f"Obstáculo detectado en el medio (entre -15º y +15º). {self.angle_obstacle:.2f} rad.{self.distance_obstacle:.2f} m.")

        elif (self.angle_obstacle >= math.pi / 12) and (self.angle_obstacle <= math.pi / 2):
            self.state = 'left_obstacle'
            self.get_logger().info(f"Obstáculo detectado a la izquierda (entre +15º y +90º). {self.angle_obstacle:.2f} rad. ")

        twist = Twist()
            
        if self.state == 'forward':
            self.state_ts = self.get_clock().now()
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        elif (self.state == 'right_obstacle') or (self.state =='center_obstacle'):
            twist.linear.x = -0.2
            twist.angular.z = 0.6
            self.get_logger().info(f'{self.get_clock().now() - self.state_ts}')
            if (self.get_clock().now() - self.state_ts) > self.turning_time:
                self.state = 'forward'

        elif self.state == 'left_obstacle':
            twist.linear.x = -0.2
            twist.angular.z = -0.6
            self.get_logger().info(f'{self.get_clock().now() - self.state_ts}')
            if (self.get_clock().now() - self.state_ts) > self.turning_time:
                self.state = 'forward'

        self.vel_publisher.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    node = BumpGoLaserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

