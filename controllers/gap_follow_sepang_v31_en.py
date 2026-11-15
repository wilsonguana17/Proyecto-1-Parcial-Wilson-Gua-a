#!/usr/bin/env python3
"""Follow the Gap V3.1 (Sepang, ROS 2 Humble) - bounded frontal speed boost.

Preserves V2 gap selection. When steering angle and yaw rate indicate stable
straight-line driving, estimates visible clearance along a finite-width LiDAR
corridor following the current vehicle curvature. Falls back to the V2
conservative front speed limit when turning or when confidence is low.
The corridor estimate can only allow a bounded speed increase; it cannot
reduce the V2 limit by itself. Intended for F1TENTH simulation.

Reactive navigation: no global map or predefined path is used by the controller.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class FollowTheGapSepangV31(Node):
    def __init__(self):
        super().__init__('follow_the_gap_sepang_v31')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Gap selection unchanged from V2.
        self.BUBBLE_RADIUS = 0.6
        self.PREPROC_MAX_DIST = 4.5
        self.SAFE_DIST_THRESHOLD = 1.2
        self.MAX_STEER_ANGLE = 0.4189
        self.MIN_SPEED = 1.0
        self.WHEELBASE_M = 0.33

        # Speed and handling limits retained from V2.
        self.max_speed = float(self.declare_parameter('max_speed', 9.0).value)
        self.lateral_accel_max = float(self.declare_parameter('lat_accel_max', 8.0).value)
        self.brake_accel = float(self.declare_parameter('brake_accel', 5.0).value)
        self.accel_limit = float(self.declare_parameter('accel_limit', 3.0).value)
        self.decel_limit = float(self.declare_parameter('decel_limit', 12.0).value)
        self.steer_alpha = float(self.declare_parameter('steer_alpha', 0.65).value)
        self.front_margin = float(self.declare_parameter('front_margin', 1.0).value)

        # Parameters affecting only the straight-line speed enhancement.
        self.front_boost_max = float(self.declare_parameter('front_boost_max', 1.0).value)
        self.corridor_half_width = float(self.declare_parameter('corridor_half_width', 0.45).value)
        self.reaction_time = float(self.declare_parameter('reaction_time', 0.12).value)
        self.straight_full_deg = float(self.declare_parameter('straight_full_deg', 0.8).value)
        self.straight_off_deg = float(self.declare_parameter('straight_off_deg', 2.0).value)
        self.yaw_full_deg_s = float(self.declare_parameter('yaw_full_deg_s', 12.0).value)
        self.yaw_off_deg_s = float(self.declare_parameter('yaw_off_deg_s', 30.0).value)

        if not 1.0 <= self.max_speed <= 12.0:
            raise ValueError('max_speed must be between 1.0 and 12.0 m/s')
        if not 1.0 <= self.lateral_accel_max <= 30.0:
            raise ValueError('lat_accel_max must be between 1.0 and 30.0')
        if not 0.5 <= self.brake_accel <= 20.0:
            raise ValueError('brake_accel must be between 0.5 and 20.0')
        if not 0.0 < self.steer_alpha <= 1.0:
            raise ValueError('steer_alpha must be in (0, 1]')
        if (self.accel_limit <= 0 or self.decel_limit <= 0 or
                self.front_margin < 0 or self.front_boost_max < 0 or
                not 0.25 <= self.corridor_half_width <= 1.5 or
                not 0.0 <= self.reaction_time <= 1.0):
            raise ValueError('Braking or corridor parameters are out of range')
        if not (0 <= self.straight_full_deg < self.straight_off_deg <= 15):
            raise ValueError('Invalid straight-line steering thresholds')
        if not (0 <= self.yaw_full_deg_s < self.yaw_off_deg_s <= 90):
            raise ValueError('Invalid straight-line yaw-rate thresholds')

        self.last_command_speed = 0.0
        self.last_steer = None
        self.last_update = time.monotonic()
        self.last_log = time.monotonic()
        self.actual_speed = 0.0
        self.actual_yaw_rate = 0.0
        self.actual_x = float('nan')
        self.actual_y = float('nan')
        self.has_odom = False
        self.last_odom_update = 0.0
        self.straight_confidence = 0.0
        self.get_logger().info(
            f'SEPANG V3.1 | MaxSpeed={self.max_speed:.1f} m/s | '
            f'BrakingDecel={self.brake_accel:.1f} m/s2 | '
            f'CorridorHalfWidth=+/-{self.corridor_half_width:.2f} m | '
            f'MaxFrontBoost={self.front_boost_max:.1f} m/s'
        )

    def odom_callback(self, msg):
        self.actual_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.actual_yaw_rate = msg.twist.twist.angular.z
        self.actual_x = msg.pose.pose.position.x
        self.actual_y = msg.pose.pose.position.y
        self.has_odom = True
        self.last_odom_update = time.monotonic()

    def preprocess_lidar(self, msg):
        raw = np.asarray(msg.ranges, dtype=float)
        if raw.size == 0 or msg.angle_increment <= 0:
            return None
        start_idx = max(0, int((-math.pi / 2 - msg.angle_min) / msg.angle_increment))
        end_idx = min(raw.size, int((math.pi / 2 - msg.angle_min) / msg.angle_increment))
        if end_idx - start_idx < 5:
            return None
        fragment = raw[start_idx:end_idx]
        valid = np.isfinite(fragment) & (fragment > max(msg.range_min, 0.02))
        proc = np.where(valid, fragment, self.PREPROC_MAX_DIST)
        proc = np.clip(proc, 0.0, self.PREPROC_MAX_DIST)
        proc = np.convolve(proc, np.ones(5) / 5.0, mode='same')
        proc[:2] = np.where(valid[:2], np.minimum(fragment[:2], self.PREPROC_MAX_DIST), self.PREPROC_MAX_DIST)
        proc[-2:] = np.where(valid[-2:], np.minimum(fragment[-2:], self.PREPROC_MAX_DIST), self.PREPROC_MAX_DIST)
        return proc, start_idx, raw

    def choose_gap(self, processed):
        closest_idx = int(np.argmin(processed))
        closest_dist = float(processed[closest_idx])
        if closest_dist > 0.01:
            span = int(math.ceil(
                math.atan2(self.BUBBLE_RADIUS, closest_dist) / self.scan_angle_increment
            ))
        else:
            span = 10
        processed[max(0, closest_idx - span):min(len(processed), closest_idx + span + 1)] = 0.0
        masked = processed > self.SAFE_DIST_THRESHOLD
        edges = np.diff(np.pad(masked.astype(np.int8), (1, 1)))
        starts = np.flatnonzero(edges == 1)
        ends = np.flatnonzero(edges == -1)
        if starts.size == 0:
            return int(np.argmax(processed))
        widths = ends - starts
        best = int(np.argmax(widths))
        gap_start = int(starts[best])
        gap_end = int(ends[best]) - 1
        best_rel = int(np.argmax(processed[gap_start:gap_end + 1]))
        center = (gap_start + gap_end) // 2
        return int(0.6 * (gap_start + best_rel) + 0.4 * center)

    def front_distance_v2(self, raw, msg):
        # Same frontal clearance estimator used in V2.
        indices = np.arange(raw.size)
        angles = msg.angle_min + indices * msg.angle_increment
        sector = raw[np.abs(angles) <= math.radians(12)]
        sector = np.where(np.isposinf(sector), msg.range_max, sector)
        valid = sector[np.isfinite(sector) & (sector > max(msg.range_min, 0.02))]
        if valid.size == 0:
            return 0.0
        return float(np.percentile(valid, 20))

    def corridor_clearance(self, raw, msg, steering_angle):
        """Estimate visible clearance along a finite-width Ackermann corridor.

        Samples a locally constant-curvature path every 0.35 m, using the
        current steering angle. At each step checks the left edge, center,
        and right edge against LiDAR returns. The first blocked sample
        determines the available clearance. This is not global planning.
        """
        max_range = float(msg.range_max)
        if not math.isfinite(max_range) or max_range < 2.0:
            return 0.0
        max_range = min(max_range, 30.0)
        if msg.angle_increment <= 0 or len(raw) < 3:
            return 0.0
        data = np.asarray(raw, dtype=float)
        # Positive infinity means no return: free space up to range_max.
        data = np.where(np.isposinf(data), max_range, data)
        # NaN, negative infinity, zero, negative range: unknown => no speed boost.
        data = np.where(np.isfinite(data) & (data > max(msg.range_min, 0.02)),
                        np.minimum(data, max_range), 0.0)
        kappa = math.tan(steering_angle) / self.WHEELBASE_M
        s = np.arange(0.7, max_range - 0.35, 0.35)
        if s.size == 0:
            return 0.0
        psi = kappa * s
        if abs(kappa) < 1e-6:
            x = s
            y = np.zeros_like(s)
        else:
            x = np.sin(psi) / kappa
            y = (1.0 - np.cos(psi)) / kappa

        theta_min = float(msg.angle_min)
        theta_max = theta_min + (len(data) - 1) * float(msg.angle_increment)
        points_clear = np.ones(s.shape, dtype=bool)
        for lateral in (-self.corridor_half_width, 0.0, self.corridor_half_width):
            xp = x - np.sin(psi) * lateral
            yp = y + np.cos(psi) * lateral
            bearings = np.arctan2(yp, xp)
            radial = np.hypot(xp, yp)
            visible = ((bearings >= theta_min) & (bearings <= theta_max) &
                       (radial < max_range))
            at = np.rint((bearings - theta_min) / msg.angle_increment).astype(np.int64)
            at = np.clip(at, 0, len(data) - 1)
            # Conservative check against the two neighboring rays.
            left = np.maximum(at - 1, 0)
            right = np.minimum(at + 1, len(data) - 1)
            measured = np.minimum(np.minimum(data[left], data[at]), data[right])
            # Small angular/numerical tolerance near obstacle boundaries.
            points_clear &= visible & (measured + 0.08 >= radial)
        blocked = np.flatnonzero(~points_clear)
        if blocked.size > 0:
            return float(s[blocked[0]])
        return float(s[-1])

    @staticmethod
    def confidence_component(value, full, off):
        if value <= full:
            return 1.0
        if value >= off:
            return 0.0
        return (off - value) / (off - full)

    def scan_callback(self, msg):
        prep = self.preprocess_lidar(msg)
        if prep is None:
            return
        ranges, offset, raw = prep
        self.scan_angle_increment = msg.angle_increment
        point = self.choose_gap(ranges.copy())
        proposed = msg.angle_min + (offset + point) * msg.angle_increment
        proposed = float(np.clip(proposed, -self.MAX_STEER_ANGLE, self.MAX_STEER_ANGLE))
        if self.last_steer is None:
            steering = proposed
        else:
            steering = self.steer_alpha * proposed + (1.0 - self.steer_alpha) * self.last_steer
        self.last_steer = steering

        speed_original = max(self.MIN_SPEED, self.max_speed * (
            1.0 - (abs(steering) / self.MAX_STEER_ANGLE) ** 1.8
        ))
        tangent = abs(math.tan(steering))
        speed_curve = math.sqrt(
            self.lateral_accel_max * self.WHEELBASE_M / max(tangent, 1e-5)
        )
        front_wide = self.front_distance_v2(raw, msg)
        speed_front_v2 = math.sqrt(
            2.0 * self.brake_accel * max(0.0, front_wide - self.front_margin)
        )
        now = time.monotonic()
        dt = max(0.001, min(now - self.last_update, 0.2))
        self.last_update = now

        steer_deg = abs(math.degrees(steering))
        yaw_deg_s = abs(math.degrees(self.actual_yaw_rate))
        steer_conf = self.confidence_component(steer_deg, self.straight_full_deg,
                                               self.straight_off_deg)
        yaw_conf = self.confidence_component(yaw_deg_s, self.yaw_full_deg_s,
                                             self.yaw_off_deg_s)
        odom_fresh = self.has_odom and (now - self.last_odom_update <= 0.30)
        raw_conf = min(steer_conf, yaw_conf) if odom_fresh else 0.0
        if front_wide < 4.0:  # Disable boost when a nearby obstacle is detected.
            raw_conf = 0.0
        if raw_conf < self.straight_confidence:
            self.straight_confidence = raw_conf  # Immediate deactivation.
        else:
            self.straight_confidence = min(
                raw_conf, self.straight_confidence + dt / 0.6
            )  # Gradual activation (0.6 s for full boost).

        front_path = float('nan')
        speed_front = speed_front_v2
        speed_front_path = 0.0
        if self.straight_confidence > 0 and self.front_boost_max > 0:
            front_path = self.corridor_clearance(raw, msg, steering)
            # Controller reaction time and longitudinal safety margin.
            usable = max(0.0, front_path - self.front_margin -
                         self.actual_speed * self.reaction_time)
            speed_front_path = math.sqrt(2.0 * self.brake_accel * usable)
            # V3.1 correction: a more conservative local arc estimate must
            # NOT cause extra braking at low confidence. The corridor can
            # ONLY enable a positive increase relative to V2 front speed.
            # If the corridor is short or uncertain, use exactly V2's limit.
            extra = max(0.0, min(speed_front_path - speed_front_v2,
                                 self.front_boost_max))
            speed_front = speed_front_v2 + self.straight_confidence * extra

        target_v2 = max(0.0, min(self.max_speed, speed_original,
                                 speed_curve, speed_front_v2))
        target = max(0.0, min(self.max_speed, speed_original,
                              speed_curve, speed_front))
        improvement = max(0.0, target - target_v2)
        change = target - self.last_command_speed
        if change > 0:
            command = self.last_command_speed + min(change, self.accel_limit * dt)
        else:
            command = self.last_command_speed + max(change, -self.decel_limit * dt)
        self.last_command_speed = command

        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.drive.steering_angle = float(steering)
        drive.drive.speed = float(command)
        self.drive_pub.publish(drive)

        if now - self.last_log >= 1.5:
            self.last_log = now
            candidate_limits = {
                'MAX': self.max_speed,
                'FTG': speed_original,
                'CORNER': speed_curve,
                'FRONT': speed_front,
            }
            reason = min(candidate_limits, key=candidate_limits.get)
            mode = 'STRAIGHT' if self.straight_confidence >= 0.95 else (
                'BLENDED' if self.straight_confidence > 0 else 'V2')
            path_label = f'{front_path:.2f}' if math.isfinite(front_path) else '--'
            self.get_logger().info(
                f'ActualSpeed={self.actual_speed:.2f}m/s | CommandSpeed={command:.2f}m/s | '
                f'TargetSpeed={target:.2f}m/s | CornerLimit={speed_curve:.2f}m/s | '
                f'FrontDistanceV2={front_wide:.2f}m | CorridorDistance={path_label}m | '
                f'FrontSpeedLimit={speed_front:.2f}m/s | V2Target={target_v2:.2f}m/s | ' 
                f'SpeedGain={improvement:.2f}m/s | Confidence={self.straight_confidence:.2f} | '
                f'Mode={mode} | LimitedBy={reason} | '
                f'SteeringAngle={math.degrees(steering):+.1f}deg | '
                f'YawRate={math.degrees(self.actual_yaw_rate):+.1f}deg/s | '
                f'X={self.actual_x:.2f},Y={self.actual_y:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapSepangV31()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
