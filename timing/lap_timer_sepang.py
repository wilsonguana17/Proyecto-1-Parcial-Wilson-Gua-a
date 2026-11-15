#!/usr/bin/env python3
"""10-lap timer for F1TENTH / Sepang (ROS 2 Humble).

Start BEFORE the controller while the car is stationary at the start.
Subscribes to /ego_racecar/odom. Does not send vehicle commands.
Uses a monotonic clock (wall-clock runtime).
"""

import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class LapTimerSepang(Node):
    TOTAL_LAPS = 10
    START_RADIUS_M = 2.0         # Dentro de esta zona se detecta la llegada
    REARM_RADIUS_M = 5.0         # Debe alejarse al menos esto para armar la meta
    MIN_LAP_DISTANCE_M = 80.0    # Evita contabilizar atajos cerca de la salida
    MIN_LAP_TIME_S = 20.0        # Evita contar entradas demasiado rapidas
    MIN_HEADING_DOT = 0.0        # Sentido similar al de la salida (±90 grados)
    TELEPORT_THRESHOLD_M = 20.0 # Descarta saltos de teletransporte/reinicio
    START_MOVEMENT_M = 0.4       # Primer movimiento para iniciar cronometro

    def __init__(self):
        super().__init__('lap_timer_sepang')
        self.subscription = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10
        )

        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.previous_xy = None
        self.armed = False
        self.race_started = False
        self.lap_start_time = None
        self.race_start_time = None
        self.lap_distance = 0.0
        self.total_distance = 0.0
        self.count = 0
        self.best_lap_s = float('inf')
        self.finished = False

        name = 'sepang_lap_results_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'
        self.csv_path = Path.cwd() / name
        self.csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'lap', 'lap_time_s', 'best_lap_s',
            'elapsed_time_s', 'lap_distance_m'
        ])
        self.csv_file.flush()

        self.get_logger().info('SEPANG LAP TIMER | Target: 10 laps')
        self.get_logger().info('Start this node first with the vehicle stationary.')
        self.get_logger().info(f'Results will be saved to: {self.csv_path}')

    @staticmethod
    def yaw_from_quaternion(q):
        # Formula estandar para rotacion yaw (eje vertical z)
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def odom_callback(self, msg):
        if self.finished:
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        now = time.monotonic()

        if self.start_x is None:
            self.start_x, self.start_y, self.start_yaw = x, y, yaw
            self.previous_xy = (x, y)
            if math.hypot(x + 14.14, y + 1.14) > 4.0:
                self.get_logger().warn(
                    'The starting position is far from the expected Sepang spawn '
                    '(-14.14, -1.14). Check sim.yaml and RViz.'
                )
            self.get_logger().info(
                f'Start position detected: x={x:.2f} m, y={y:.2f} m, '
                f'heading={math.degrees(yaw):.1f} degrees'
            )
            return

        last_x, last_y = self.previous_xy
        step_m = math.hypot(x - last_x, y - last_y)
        self.previous_xy = (x, y)

        if step_m > self.TELEPORT_THRESHOLD_M:
            self.get_logger().warn(
                'Position jump detected; ignoring this jump. '
                'Restart the lap timer if you restarted the simulator.'
            )
            self.armed = False
            return

        distance_to_start = math.hypot(x - self.start_x, y - self.start_y)

        # No contar la espera entre iniciar el contador y arrancar el coche.
        if not self.race_started:
            if distance_to_start >= self.START_MOVEMENT_M:
                self.race_started = True
                self.lap_start_time = now
                self.race_start_time = now
                self.get_logger().info('Vehicle is moving: lap timing has started.')
            else:
                return

        self.lap_distance += step_m
        self.total_distance += step_m

        # Histéresis: solo puede contar otra vuelta despues de salir de la zona.
        if distance_to_start >= self.REARM_RADIUS_M:
            self.armed = True

        if not self.armed or distance_to_start > self.START_RADIUS_M:
            return

        elapsed = now - self.lap_start_time
        heading_dot = math.cos(yaw - self.start_yaw)
        if (
            elapsed < self.MIN_LAP_TIME_S
            or self.lap_distance < self.MIN_LAP_DISTANCE_M
            or heading_dot < self.MIN_HEADING_DOT
        ):
            return

        self.count += 1
        self.best_lap_s = min(self.best_lap_s, elapsed)
        accumulated_s = now - self.race_start_time

        self.csv_writer.writerow([
            self.count, f'{elapsed:.3f}', f'{self.best_lap_s:.3f}',
            f'{accumulated_s:.3f}', f'{self.lap_distance:.2f}'
        ])
        self.csv_file.flush()

        self.get_logger().info('==========================================')
        self.get_logger().info(f'LAP {self.count}/{self.TOTAL_LAPS} COMPLETED')
        self.get_logger().info(f'Lap time: {elapsed:.2f} s')
        self.get_logger().info(f'Best lap: {self.best_lap_s:.2f} s')
        self.get_logger().info(f'Total elapsed time: {accumulated_s:.2f} s')
        self.get_logger().info(f'Estimated lap distance: {self.lap_distance:.1f} m')
        self.get_logger().info('==========================================')

        self.lap_start_time = now
        self.lap_distance = 0.0
        self.armed = False

        if self.count >= self.TOTAL_LAPS:
            self.finished = True
            self.get_logger().info(
                f'FINISH: 10/10 laps | Total {accumulated_s:.2f} s | '
                f'Best lap {self.best_lap_s:.2f} s'
            )
            self.get_logger().info('Timing complete. Stop the controller with Ctrl+C.')

    def destroy_node(self):
        try:
            self.csv_file.close()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LapTimerSepang()
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
