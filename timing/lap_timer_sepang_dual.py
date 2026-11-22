#!/usr/bin/env python3
"""Contador independiente de 10 vueltas para un vehiculo F1Tenth.

Se puede ejecutar DOS VECES, con vehicle=ego/opp y odom_topic independiente.
Adaptado del contador original lap_timer_sepang.py: conserva sus criterios
para detectar vueltas. Cada ejecucion escribe su propio CSV.

Iniciar con ambos vehiculos quietos y SIN reiniciar el simulador durante la prueba.
Este cronometro NO detecta colisiones ni detiene los vehiculos.
"""

import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class LapTimerSepangDual(Node):
    TOTAL_LAPS = 10
    START_RADIUS_M = 2.0
    REARM_RADIUS_M = 5.0
    MIN_LAP_DISTANCE_M = 80.0
    MIN_LAP_TIME_S = 20.0
    MIN_HEADING_DOT = 0.0
    TELEPORT_THRESHOLD_M = 20.0
    START_MOVEMENT_M = 0.4

    def __init__(self):
        super().__init__('lap_timer_sepang_dual')
        self.vehicle = str(self.declare_parameter('vehicle', 'ego').value).strip().lower()
        topic_default = '/ego_racecar/odom' if self.vehicle == 'ego' else '/opp_racecar/odom'
        self.odom_topic = str(self.declare_parameter('odom_topic', topic_default).value)
        if self.vehicle not in ('ego', 'opp'):
            raise ValueError("vehicle debe ser 'ego' u 'opp'")
        self.subscription = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)

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

        name = ('sepang_lap_results_' + self.vehicle + '_' +
                datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv')
        self.csv_path = Path.cwd() / name
        self.csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'lap', 'lap_time_s', 'best_lap_s', 'elapsed_time_s', 'lap_distance_m'])
        self.csv_file.flush()

        self.get_logger().info(
            f'{self.vehicle.upper()} | 10-LAP TIMER | topic={self.odom_topic}')
        self.get_logger().info('Inicie el contador con el vehiculo detenido.')
        self.get_logger().info(f'Archivo CSV: {self.csv_path}')

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

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
            if self.vehicle == 'ego' and math.hypot(x + 14.14, y + 1.14) > 4.0:
                self.get_logger().warn('EGO lejos de salida Sepang esperada (-14.14,-1.14).')
            self.get_logger().info(
                f'{self.vehicle.upper()} start x={x:.2f}m y={y:.2f}m yaw={math.degrees(yaw):.1f}deg')
            return

        last_x, last_y = self.previous_xy
        step_m = math.hypot(x - last_x, y - last_y)
        self.previous_xy = (x, y)
        if step_m > self.TELEPORT_THRESHOLD_M:
            self.get_logger().warn('Salto de posicion: reinicie el contador si reinicio el simulador.')
            self.armed = False
            return

        distance_to_start = math.hypot(x - self.start_x, y - self.start_y)
        if not self.race_started:
            if distance_to_start < self.START_MOVEMENT_M:
                return
            self.race_started = True
            self.lap_start_time = now
            self.race_start_time = now
            self.get_logger().info(f'{self.vehicle.upper()} comienza cronometraje.')

        self.lap_distance += step_m
        self.total_distance += step_m

        if distance_to_start >= self.REARM_RADIUS_M:
            self.armed = True
        if not self.armed or distance_to_start > self.START_RADIUS_M:
            return

        elapsed = now - self.lap_start_time
        heading_dot = math.cos(yaw - self.start_yaw)
        if (elapsed < self.MIN_LAP_TIME_S or
                self.lap_distance < self.MIN_LAP_DISTANCE_M or
                heading_dot < self.MIN_HEADING_DOT):
            return

        self.count += 1
        self.best_lap_s = min(self.best_lap_s, elapsed)
        total = now - self.race_start_time
        self.csv_writer.writerow([
            self.count, f'{elapsed:.3f}', f'{self.best_lap_s:.3f}',
            f'{total:.3f}', f'{self.lap_distance:.2f}'])
        self.csv_file.flush()
        self.get_logger().info(
            f'{self.vehicle.upper()} LAP {self.count}/{self.TOTAL_LAPS} | '
            f'Lap={elapsed:.2f}s | Best={self.best_lap_s:.2f}s | '
            f'Total={total:.2f}s | Dist={self.lap_distance:.1f}m')
        self.lap_start_time = now
        self.lap_distance = 0.0
        self.armed = False

        if self.count >= self.TOTAL_LAPS:
            self.finished = True
            self.get_logger().info(
                f'{self.vehicle.upper()} FINISH 10/10 | Total {total:.2f}s | '
                f'Best {self.best_lap_s:.2f}s')

    def destroy_node(self):
        try:
            self.csv_file.close()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LapTimerSepangDual()
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