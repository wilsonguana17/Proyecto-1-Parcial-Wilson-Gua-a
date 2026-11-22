#!/usr/bin/env python3
"""Follow the Gap V3.9 — reactive navigation and overtaking in F1Tenth.

Uses LiDAR to choose free gaps and estimate local corridor clearance, own
odometry for motion feedback, and the opponent's odometry plus LiDAR to evaluate
opportunistic dynamic overtaking. Both robots may run the same controller
through separate ROS topic remappings and different max_speed values.

Intended for simulation; no collision-free guarantee for untested maps.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class FollowTheGapSepangV39(Node):
    WHEELBASE_M = 0.33
    MAX_STEER = 0.4189             # 24 grados

    def __init__(self):
        super().__init__('follow_the_gap_sepang_v39')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        # Parámetros que se pueden ajustar desde la terminal.
        self.max_speed = float(self.declare_parameter('max_speed', 6.0).value)
        self.lat_accel = float(self.declare_parameter('lat_accel_max', 8.0).value)
        self.brake_accel = float(self.declare_parameter('brake_accel', 7.0).value)
        self.accel_limit = float(self.declare_parameter('accel_limit', 5.0).value)
        self.decel_limit = float(self.declare_parameter('decel_limit', 16.0).value)
        self.steer_alpha = float(self.declare_parameter('steer_alpha', 0.65).value)
        self.front_margin = float(self.declare_parameter('front_margin', 0.75).value)
        self.reaction_time = float(self.declare_parameter('reaction_time', 0.15).value)
        self.safety_radius = float(self.declare_parameter('safety_radius', 0.45).value)
        self.safe_dist = float(self.declare_parameter('safe_dist', 1.0).value)
        self.path_half_width = float(self.declare_parameter('path_half_width', 0.38).value)
        self.preproc_max_dist = float(self.declare_parameter('preproc_max_dist', 15.0).value)
        self.front_detection_deg = float(self.declare_parameter('front_detection_deg', 25.0).value)
        self.lookahead_base = float(self.declare_parameter('lookahead_base', 2.8).value)
        self.lookahead_gain = float(self.declare_parameter('lookahead_gain', 0.50).value)
        # Se acepta por compatibilidad con la línea de comandos de la V3.1,
        # pero no se utiliza: la velocidad ahora depende del corredor estimado.
        self.declare_parameter('front_boost_max', 0.0)

        if not 1.0 <= self.max_speed <= 12.0:
            raise ValueError('max_speed debe estar entre 1 y 12 m/s')
        if not 1.0 <= self.lat_accel <= 30.0:
            raise ValueError('lat_accel_max debe estar entre 1 y 30')
        if not 1.0 <= self.brake_accel <= 20.0:
            raise ValueError('brake_accel debe estar entre 1 y 20')
        if self.accel_limit <= 0 or self.decel_limit <= 0:
            raise ValueError('limites de aceleracion/frenado invalidos')
        if not 0.0 < self.steer_alpha <= 1.0:
            raise ValueError('steer_alpha debe estar en (0, 1]')
        if self.front_margin < 0 or not 0.25 <= self.path_half_width <= 0.8:
            raise ValueError('margenes de seguridad invalidos')
        if not 0.20 <= self.safety_radius <= 1.2:
            raise ValueError('safety_radius fuera de rango')
        if self.safe_dist <= 0 or not 8 <= self.preproc_max_dist <= 30:
            raise ValueError('safe_dist o preproc_max_dist invalidos')
        if not 10 <= self.front_detection_deg <= 45:
            raise ValueError('front_detection_deg fuera de rango')
        if self.reaction_time < 0 or self.lookahead_base < 2.0 or self.lookahead_gain < 0:
            raise ValueError('tiempo de reaccion/lookahead invalidos')

        self.last_speed = 0.0
        self.last_steer = None
        self.last_update = time.monotonic()
        self.last_log = time.monotonic()
        self.last_warn = 0.0
        self.actual_speed = 0.0
        self.x = float('nan')
        self.y = float('nan')
        self.odom_time = 0.0
        self.last_target_angle = 0.0
        # Adelantamiento oportunista: activado solo con corredor lateral verificado.
        self.opp_odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/opp_odom', self.opp_callback, 10)
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.opp_x = float('nan')
        self.opp_y = float('nan')
        self.opp_yaw = 0.0
        self.opp_speed = 0.0
        self.opp_last = 0.0
        self.pass_enabled = bool(self.declare_parameter('pass_enabled', True).value)
        self.pass_dist_max = float(self.declare_parameter('pass_max_distance', 14.0).value)
        self.pass_speed_cap = float(self.declare_parameter('pass_speed_cap', 8.0).value)
        self.pass_offset = float(self.declare_parameter('pass_offset', 0.85).value)
        self.pass_side = 0
        self.pass_mode = 'NORMAL'
        self.pass_last_log = 0.0
        self.pass_cooldown = 0.0
        self.pass_angle = 0.0
        self.pass_start_time = 0.0
        self.get_logger().info(
            f'SEPANG V3.9 | MaxSpeed={self.max_speed:.1f}m/s | '
            f'LatAccel={self.lat_accel:.1f} | Brake={self.brake_accel:.1f} | '
            f'Bubble={self.safety_radius:.2f}m | PathHalfWidth={self.path_half_width:.2f}m'
        )

    def odom_callback(self, msg):
        self.actual_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.odom_time = time.monotonic()
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.yaw_rate = float(msg.twist.twist.angular.z)

    def opp_callback(self, msg):
        self.opp_x = float(msg.pose.pose.position.x)
        self.opp_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.opp_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.opp_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.opp_last = time.monotonic()

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2*math.pi) - math.pi

    def _relative(self):
        dx,dy = self.opp_x-self.x,self.opp_y-self.y
        c,s = math.cos(self.yaw), math.sin(self.yaw)
        return c*dx+s*dy, -s*dx+c*dy, self._wrap(self.opp_yaw-self.yaw)

    def _filtered(self, raw, msg, forward, lateral):
        """Filtra solo retornos coincidentes con la silueta conocida del rival.
        El modelo dinámico del rival se comprueba por separado y nunca se ignora."""
        rr=np.asarray(raw, dtype=float)
        data=np.where(np.isposinf(rr),float(msg.range_max),
           np.where(np.isfinite(rr)&(rr>max(0.02,msg.range_min)),rr,0.0))
        r=math.hypot(forward,lateral)
        if not .8<r<self.pass_dist_max+3:
            return data
        ang=math.atan2(lateral,forward)
        bearings=msg.angle_min+np.arange(len(data))*msg.angle_increment
        width=math.atan2(0.28,max(.5,r-.35))
        mask=(np.abs(self._wrap(bearings-ang))<width) & (data>=r-.77)&(data<=r+.12)
        data[mask]=float(msg.range_max)
        return data

    def _pass_path(self, data, msg, f, lat, dhead, side, offset, length):
        """Corredor local suave para alinearse a un lado del oponente.
        Comprueba anchura del carro y envolventes temporales del rival."""
        dist=np.arange(.3,9.81,.2)
        L=length
        dhead=float(np.clip(dhead,-.23,.23))
        slope=math.tan(dhead)
        # Recta paralela al rumbo del rival. La meta lateral permanece fija
        # aunque la orientación del ego cambie durante el desplazamiento.
        end_y=lat+slope*(L-f)+side*offset/max(.8,math.cos(dhead))
        u=np.minimum(dist/L,1.0)
        yy=(3*u*u-2*u*u*u)*end_y+(u*u*u-u*u)*L*slope
        dy=((6*u-6*u*u)*end_y+(3*u*u-2*u)*L*slope)/L
        rest=dist>L
        yy[rest]=end_y+slope*(dist[rest]-L)
        dy[rest]=slope
        denom=np.sqrt(1+dy*dy)
        best=float(dist[-1])
        maxa=msg.angle_min+(len(data)-1)*msg.angle_increment
        for shift in (-self.path_half_width,0,self.path_half_width):
            xp=dist-shift*dy/denom
            yp=yy+shift/denom
            bearings=np.arctan2(yp,xp)
            radial=np.hypot(xp,yp)
            indices=np.clip(np.rint((bearings-msg.angle_min)/msg.angle_increment).astype(int),0,len(data)-1)
            observed=np.minimum.reduce((data[np.maximum(indices-1,0)],data[indices],data[np.minimum(indices+1,len(data)-1)]))
            block=np.where((bearings<msg.angle_min)|(bearings>maxa)|(observed+.04<radial))[0]
            if block.size:best=min(best,float(dist[block[0]]))
        # Dos velocidades plausibles (constante / acelerando) para no
        # declarar libre el espacio que ocupará el rival más adelante.
        for ego_v in (max(3.,self.actual_speed),max(3.,self.actual_speed+2.)):
            tau=dist/ego_v
            opp_f=f+self.opp_speed*math.cos(dhead)*tau
            opp_y=lat+self.opp_speed*math.sin(dhead)*tau
            # Dimensiones combinadas, con margen longitudinal y lateral.
            risk=((dist-opp_f)/.88)**2+((yy-opp_y)/.58)**2
            collide=np.flatnonzero(risk<1.0)
            if collide.size:best=min(best,float(dist[collide[0]]))
        look=min(5.0,max(3.0,self.actual_speed*.55))
        lu=min(look/L,1.0)
        target_y=(3*lu*lu-2*lu*lu*lu)*end_y+(lu*lu*lu-lu*lu)*L*slope
        if look>L:target_y=end_y+slope*(look-L)
        curvature=2*target_y/(look**2+target_y**2)
        steer=float(np.clip(math.atan(self.WHEELBASE_M*curvature),-self.MAX_STEER,self.MAX_STEER))
        return steer,best

    def _overtake(self, raw, msg, now):
        if not self.pass_enabled or now-self.odom_time>.3 or now-self.opp_last>.3:
            self.pass_mode='NORMAL';self.pass_side=0
            return None
        if not (math.isfinite(self.opp_x) and math.isfinite(self.opp_y)):
            return None
        f,lat,head=self._relative()
        if self.pass_mode=='OVERTAKE' and f< -2.0 and abs(lat)>.55:
            self.get_logger().info(f'PASS COMPLETE: rival detras {abs(f):.1f}m | lat={lat:+.2f}m')
            self.pass_mode='NORMAL';self.pass_side=0;self.pass_cooldown=now+4
            return None
        if self.pass_mode=='NORMAL':
            if now<self.pass_cooldown or not (4.5<f<self.pass_dist_max and abs(lat)<.65 and abs(head)<math.radians(10)):
                return None
            if abs(self.yaw_rate)>.22 or self.max_speed<=self.opp_speed+1.5:
                return None
            # No usar odometría externa como si fuera un hueco: confirmar que
            # el LiDAR ve la superficie aproximada del vehículo.
            rawarr=np.asarray(raw,dtype=float)
            beam=math.atan2(lat,f)
            bearings=msg.angle_min+np.arange(len(rawarr))*msg.angle_increment
            relevant=rawarr[(np.abs(self._wrap(bearings-beam))<.055)&np.isfinite(rawarr)]
            r=math.hypot(f,lat)
            if relevant.size==0 or not (r-1.0<=np.min(relevant)<=r+.2):return None
        else:
            if abs(lat)>2.4 or f>self.pass_dist_max+4:
                self.pass_mode='NORMAL';self.pass_side=0;self.pass_cooldown=now+3
                return None
        data=self._filtered(raw,msg,f,lat)
        sides=(self.pass_side,) if self.pass_mode=='OVERTAKE' else (1,-1)
        options=[]
        for side in sides:
            for offset in (self.pass_offset, min(1.05,self.pass_offset+.12)):
                for length in (5.,7.):
                    steer,free=self._pass_path(data,msg,f,lat,head,side,offset,length)
                    # Comprobar el primer giro calculado frente a las PAREDES,
                    # y el rival mediante el modelo dinámico de la trayectoria.
                    arc=self.path_clearance(data,msg,steer)
                    if arc < 2.5:continue
                    required=(min(8.0,max(5.0,f+1.2)) if self.pass_mode=='NORMAL'
                              else 3.8)
                    if free < required:continue
                    score=free+.18*min(arc,6)-3*abs(steer)-.25*abs(offset-self.pass_offset)
                    options.append((score,side,steer,free,arc))
        if not options:
            if self.pass_mode=='OVERTAKE':
                self.pass_mode='NORMAL';self.pass_side=0;self.pass_cooldown=now+3
                if now-self.pass_last_log>2:
                    self.pass_last_log=now
                    self.get_logger().warn(f'PASS ABORT: sin corredor seguro | rival={f:.1f}m')
            elif now-self.pass_last_log>3:
                self.pass_last_log=now
                self.get_logger().info(f'PASS WAIT: corredor lateral insuficiente | rival={f:.1f}m')
            return None
        _,side,steer,clear,arc=max(options)
        if self.pass_mode!='OVERTAKE':
            self.pass_mode='OVERTAKE';self.pass_side=side;self.pass_start_time=now
            self.get_logger().info(f'PASS START: {"LEFT" if side>0 else "RIGHT"} | rival={f:.1f}m | libre={clear:.1f}m')
        return steer,min(clear,arc)

    def preprocess_lidar(self, msg):
        raw = np.asarray(msg.ranges, dtype=float)
        if raw.size < 10 or msg.angle_increment <= 0:
            return None
        start = max(0, int(math.ceil((-math.pi / 2 - msg.angle_min) / msg.angle_increment)))
        end = min(raw.size, int(math.floor((math.pi / 2 - msg.angle_min) / msg.angle_increment)) + 1)
        if end - start < 10:
            return None
        fragment = raw[start:end]
        # +inf: sin retorno -> espacio libre hasta el alcance del sensor.
        # NaN/0/-inf: lectura desconocida -> se trata conservadoramente.
        measured = np.where(
            np.isposinf(fragment), self.preproc_max_dist,
            np.where(np.isfinite(fragment) & (fragment > max(msg.range_min, 0.02)),
                     np.minimum(fragment, self.preproc_max_dist), 0.0))
        # Una mediana preserva mejor bordes que la media de 5 muestras.
        # Para limitar dependencias usamos un mínimo local de 3 rayos.
        proc = np.minimum.reduce([
            measured,
            np.r_[measured[0], measured[:-1]],
            np.r_[measured[1:], measured[-1]]
        ])
        return proc, start, raw

    def choose_gap(self, proc, start, msg, speed):
        """Selecciona un gap usando distancia de anticipación dependiente de velocidad.

        A diferencia de la burbuja sobre el punto más cercano, esto no empuja
        el vehículo hacia una pared lateral si la recta está despejada.
        """
        n = len(proc)
        angles = msg.angle_min + (start + np.arange(n)) * msg.angle_increment
        lookahead = min(12.0, self.lookahead_base + self.lookahead_gain * speed)
        # Marcar como libres rayos con margen suficiente en profundidad.
        # Las paredes laterales se recortan naturalmente al tener menos alcance.
        free = (proc > max(self.safe_dist, lookahead)) & \
               (np.abs(angles) <= math.radians(65.0))

        # Inflar únicamente una discontinuidad frontal bien definida.
        # No inflamos cada lectura de pared: eso anulaba todos los gaps.
        front = np.abs(angles) < math.radians(self.front_detection_deg)
        obstacle_idx = np.flatnonzero(front & (proc > 0.02) & (proc < lookahead))
        if obstacle_idx.size:
            center_obstacle = int(obstacle_idx[np.argmin(proc[obstacle_idx])])
            d = float(proc[center_obstacle])
            # Detectar obstáculo destacado comparándolo con rayos laterales.
            shift = max(1, int(round(math.radians(9.0) / msg.angle_increment)))
            left_ref = float(proc[max(0, center_obstacle - shift)])
            right_ref = float(proc[min(n - 1, center_obstacle + shift)])
            if min(left_ref, right_ref) > d + 1.0:
                # Inflación pequeña: deja un margen lateral al vehículo.
                pad = int(math.ceil(math.atan2(self.safety_radius, max(d, 0.1)) /
                                    msg.angle_increment))
                free[max(0, center_obstacle - pad):min(n, center_obstacle + pad + 1)] = False

        transitions = np.diff(np.pad(free.astype(np.int8), (1, 1)))
        starts = np.flatnonzero(transitions == 1)
        ends = np.flatnonzero(transitions == -1)
        if starts.size == 0:
            return None, lookahead
        widths = ends - starts
        widths_deg = widths * msg.angle_increment * 180.0 / math.pi
        candidates_gaps = np.flatnonzero(widths_deg >= 5.0)
        if candidates_gaps.size == 0:
            return None, lookahead
        # Gap más ancho, con un desempate suave por continuidad y dirección frontal.
        mids = (starts + ends - 1) / 2.0
        mid_angles = msg.angle_min + (start + mids) * msg.angle_increment
        scores = widths.astype(float) * (1.0 + 0.06 * np.cos(mid_angles - self.last_target_angle))
        best = int(candidates_gaps[np.argmax(scores[candidates_gaps])])
        point = int((starts[best] + ends[best] - 1) // 2)
        target_angle = msg.angle_min + (start + point) * msg.angle_increment
        self.last_target_angle = target_angle
        return target_angle, lookahead

    def path_clearance(self, raw, msg, steering):
        """Espacio libre estimado para el vehículo sobre arco Ackermann.

        Inspecciona bordes y centro del corredor, no solo el rayo frontal.
        La primera muestra bloqueada devuelve distancia recorrida sobre el arco.
        Es una aproximación conservadora, NO un verificador de colisiones exacto.
        """
        if msg.angle_increment <= 0 or len(raw) < 10:
            return 0.0
        max_sensor = float(msg.range_max)
        if not math.isfinite(max_sensor) or max_sensor < 2.0:
            return 0.0
        horizon = min(15.0, max_sensor - 0.3)
        if horizon <= 0.5:
            return 0.0
        data = np.asarray(raw, dtype=float)
        data = np.where(
            np.isposinf(data), max_sensor,
            np.where(np.isfinite(data) & (data > max(msg.range_min, 0.02)),
                     np.minimum(data, max_sensor), 0.0))
        kappa = math.tan(steering) / self.WHEELBASE_M
        samples = np.arange(0.45, horizon, 0.25)
        if not samples.size:
            return 0.0
        psi = kappa * samples
        if abs(kappa) < 1e-7:
            x = samples
            y = np.zeros_like(samples)
        else:
            x = np.sin(psi) / kappa
            y = (1.0 - np.cos(psi)) / kappa
        scan_max_angle = msg.angle_min + (len(data) - 1) * msg.angle_increment
        for lateral in (-self.path_half_width, 0.0, self.path_half_width):
            xp = x - np.sin(psi) * lateral
            yp = y + np.cos(psi) * lateral
            bearings = np.arctan2(yp, xp)
            radii = np.hypot(xp, yp)
            visible = ((bearings >= msg.angle_min) &
                       (bearings <= scan_max_angle) & (radii < max_sensor))
            at = np.rint((bearings - msg.angle_min) / msg.angle_increment).astype(np.int64)
            at = np.clip(at, 0, len(data) - 1)
            measured = np.minimum.reduce([
                data[np.maximum(at - 1, 0)], data[at],
                data[np.minimum(at + 1, len(data) - 1)]
            ])
            blocked = np.flatnonzero((~visible) | (measured + 0.06 < radii))
            if blocked.size:
                # Mantener el primer bloqueo encontrado para este borde.
                return_candidate = float(samples[blocked[0]])
                # No devolver todavía: otros bordes podrían bloquear antes.
                if lateral == -self.path_half_width:
                    result = return_candidate
                else:
                    result = min(result, return_candidate)
            elif lateral == -self.path_half_width:
                result = float(samples[-1])
        return float(result)

    def refine_steering(self, raw, msg, steering):
        """Evitar quedarse parado cuando el centro del gap roza un obstáculo."""
        base_clear = self.path_clearance(raw, msg, steering)
        if base_clear >= 2.4:
            return steering, base_clear
        # Revisar algunas curvas factibles antes de declarar que no hay salida.
        candidates = np.radians([-24, -18, -12, -8, -5, 0, 5, 8, 12, 18, 24])
        candidates = np.r_[candidates, steering]
        best_steer = steering
        best_clear = base_clear
        best_score = base_clear
        for candidate in candidates:
            candidate = float(np.clip(candidate, -self.MAX_STEER, self.MAX_STEER))
            clear = self.path_clearance(raw, msg, candidate)
            # Preferir dirección similar cuando ambas están igualmente libres.
            score = clear - 0.35 * abs(candidate - steering)
            if score > best_score + 0.05:
                best_score = score
                best_steer = candidate
                best_clear = clear
        return best_steer, best_clear

    def publish(self, speed, steer):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.speed = float(speed)
        cmd.drive.steering_angle = float(steer)
        self.drive_pub.publish(cmd)

    def stop(self, reason, now):
        self.publish(0.0, 0.0)
        self.last_speed = 0.0
        self.last_steer = None
        if now - self.last_warn > 1.0:
            self.last_warn = now
            self.get_logger().warn(reason)

    def scan_callback(self, msg):
        now = time.monotonic()
        prep = self.preprocess_lidar(msg)
        if prep is None:
            self.stop('SCAN INVALIDO: detenido', now)
            return
        proc, offset, raw = prep
        odom_fresh = (now - self.odom_time) < 0.4
        # Si la odometría no está disponible no acelerar sin referencia.
        if not odom_fresh:
            self.stop('ODOMETRIA NO DISPONIBLE: detenido', now)
            return
        speed_ref = max(self.actual_speed, self.last_speed)
        pass_solution = self._overtake(raw,msg,now)
        if pass_solution is not None:
            # Solo se aplica la intervención mientras hay un corredor válido.
            steer, path_dist = pass_solution
            if self.last_steer is not None:
                steer=self.steer_alpha*steer+(1-self.steer_alpha)*self.last_steer
            # El suavizado no debe invalidar la comprobación del giro.
            geo=self._relative()
            filt=self._filtered(raw,msg,geo[0],geo[1])
            arc=self.path_clearance(filt,msg,steer)
            if arc>=2.0:
                self.last_steer=steer
                max_v=min(self.max_speed,self.pass_speed_cap)
                steer_v=max(1.0,max_v*(1-(abs(steer)/self.MAX_STEER)**1.8))
                curve_v=math.sqrt(self.lat_accel*self.WHEELBASE_M/max(abs(math.tan(steer)),1e-5))
                clear=min(path_dist,arc)
                usable=max(0.,clear-self.front_margin-self.actual_speed*self.reaction_time)
                path_v=math.sqrt(2*self.brake_accel*usable)
                target=min(max_v,steer_v,curve_v,path_v)
                dt=max(.001,min(.20,now-self.last_update));self.last_update=now
                dv=target-self.last_speed
                self.last_speed=max(0.,self.last_speed+(min(dv,self.accel_limit*dt) if dv>=0 else max(dv,-self.decel_limit*dt)))
                self.publish(self.last_speed,steer)
                if now-self.last_log>=1.:
                    self.last_log=now
                    self.get_logger().info(f'Mode=OVERTAKE | ActualSpeed={self.actual_speed:.2f}m/s | CommandSpeed={self.last_speed:.2f}m/s | PathClear={clear:.2f}m | Steering={math.degrees(steer):+.1f}deg')
                return
            self.pass_mode='NORMAL';self.pass_side=0;self.pass_cooldown=now+3
        target_angle, lookahead = self.choose_gap(proc, offset, msg, speed_ref)
        if target_angle is None:
            self.stop('NO SAFE GAP: detenido (revisar espacio libre)', now)
            return

        desired_steer = float(np.clip(target_angle, -self.MAX_STEER, self.MAX_STEER))
        if self.last_steer is None:
            steering = desired_steer
        else:
            steering = (self.steer_alpha * desired_steer +
                        (1.0 - self.steer_alpha) * self.last_steer)
        steering, path_dist = self.refine_steering(raw, msg, steering)
        self.last_steer = steering

        steer_fraction = min(1.0, abs(steering) / self.MAX_STEER)
        speed_steer = max(1.0, self.max_speed * (1.0 - steer_fraction ** 1.8))
        speed_curve = math.sqrt(
            self.lat_accel * self.WHEELBASE_M /
            max(abs(math.tan(steering)), 1e-5))

        # Frenar según trayectoria, no solo obstáculo frontal. La reacción
        # usa la velocidad medida, y añade margen de seguridad longitudinal.
        usable = max(0.0, path_dist - self.front_margin -
                     self.actual_speed * self.reaction_time)
        speed_path = math.sqrt(2.0 * self.brake_accel * usable)

        limits = {
            'MAX': self.max_speed,
            'STEER': speed_steer,
            'CURVE': speed_curve,
            'PATH': speed_path,
        }
        target = max(0.0, min(limits.values()))
        dt = max(0.001, min(0.20, now - self.last_update))
        self.last_update = now
        dv = target - self.last_speed
        if dv >= 0:
            command = self.last_speed + min(dv, self.accel_limit * dt)
        else:
            command = self.last_speed + max(dv, -self.decel_limit * dt)
        self.last_speed = max(0.0, command)
        self.publish(self.last_speed, steering)

        if now - self.last_log >= 1.0:
            self.last_log = now
            limiting = min(limits, key=limits.get)
            self.get_logger().info(
                f'ActualSpeed={self.actual_speed:.2f}m/s | '
                f'CommandSpeed={self.last_speed:.2f}m/s | '
                f'TargetSpeed={target:.2f}m/s | '
                f'PathClear={path_dist:.2f}m | Lookahead={lookahead:.1f}m | '
                f'Steering={math.degrees(steering):+.1f}deg | '
                f'LimitBy={limiting} | X={self.x:.2f},Y={self.y:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapSepangV39()
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