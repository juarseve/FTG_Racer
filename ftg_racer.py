#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap_node')
        
        # --- Suscriptores y Publicadores ---
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # --- Parámetros del Controlador ---
        self.bubble_radius = 0.50  # Mantenemos la burbuja alta para las curvas
        self.max_speed = 9.0       # Reducir de 6.0 para ganar estabilidad en recta
        self.med_speed = 3.5       
        self.min_speed = 2.2       # Aumentar ligeramente de 1.8 para curvas
        self.max_steering_angle = math.radians(25) # Reducir de 30 para giros más controlados
        
        # --- Variables de Telemetría y Cronometraje ---
        self.lap_count = 0
        self.lap_times = []
        self.start_pos = None
        self.lap_start_time = None
        self.car_left_start = False
        self.finish_line_radius = 1.5  # Radio (m) para considerar que cruzó la meta
        self.leave_threshold = 4.0     # Distancia (m) para considerar que salió de la meta
        self.race_finished = False
        
        self.prev_steering_angle = 0.0

    def preprocess_lidar(self, ranges, angle_min, angle_increment, max_view_distance):
        """ Filtra valores, recorta el FOV trasero y aplica un horizonte dinámico """
        proc_ranges = np.array(ranges)
        proc_ranges = np.nan_to_num(proc_ranges, nan=0.0, posinf=10.0, neginf=0.0)
        
        # Aplicamos el horizonte de visión dinámico calculado
        proc_ranges = np.clip(proc_ranges, 0.0, max_view_distance)
        
        # Ignorar lo que está detrás del auto (-90 a +90 grados)
        fov_min = -math.pi / 2.0
        fov_max = math.pi / 2.0
        
        start_idx = int((fov_min - angle_min) / angle_increment)
        end_idx = int((fov_max - angle_min) / angle_increment)
        
        proc_ranges[:start_idx] = 0.0
        proc_ranges[end_idx:] = 0.0
        
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Encuentra la secuencia consecutiva más larga de valores no nulos """
        # Retorna el índice de inicio y fin del mayor gap
        non_zero = np.where(free_space_ranges > 0.0)[0]
        if len(non_zero) == 0:
            return 0, len(free_space_ranges) - 1
            
        # Agrupar índices consecutivos
        splits = np.split(non_zero, np.where(np.diff(non_zero) != 1)[0] + 1)
        max_gap = max(splits, key=len)
        return max_gap[0], max_gap[-1]

    def find_best_point(self, start_i, end_i, ranges):
        """ Encuentra el centroide de la zona más profunda del gap """
        gap_slice = ranges[start_i:end_i+1]
        
        # 1. Encontrar la profundidad máxima en este gap
        max_val = np.max(gap_slice)
        
        # 2. Encontrar TODOS los índices que alcanzan esta profundidad máxima
        # Usamos un pequeño margen (0.05) para absorber el ruido del sensor LiDAR
        max_indices = np.where(gap_slice >= max_val - 0.05)[0]
        
        # 3. El mejor punto es el centro exacto de esta "planicie" profunda
        center_of_maxes = int(np.mean(max_indices))
        
        return start_i + center_of_maxes
	
    def scan_callback(self, msg):
        if self.race_finished:
            return 
            
        # 1. Encontrar la distancia al obstáculo frontal (Usando promedio para estabilizar en rectas)
        raw_ranges = np.array(msg.ranges)
        raw_ranges = np.nan_to_num(raw_ranges, nan=0.0, posinf=10.0, neginf=0.0)
        
        # NUEVO: En lugar del mínimo absoluto, usar el promedio de un cono frontal de +/- 15 grados
        cone_start_angle = math.radians(-15)
        cone_end_angle = math.radians(15)
        cone_start_idx = int((cone_start_angle - msg.angle_min) / msg.angle_increment)
        cone_end_idx = int((cone_end_angle - msg.angle_min) / msg.angle_increment)
        
        # Filtrar valores válidos dentro del cono frontal
        frontal_ranges = raw_ranges[cone_start_idx:cone_end_idx]
        valid_frontal_ranges = frontal_ranges[frontal_ranges > 0.1]
        
        # Si no hay datos frontales, usar un valor por defecto seguro
        closest_dist = np.mean(valid_frontal_ranges) if len(valid_frontal_ranges) > 0 else 3.5
        
        # --- ESTRATEGIA ADAPTATIVA (Mismo concepto, valores ajustados) ---
        dynamic_max_view = max(1.7, min(3.8, closest_dist + 0.6))
        
        ranges = self.preprocess_lidar(msg.ranges, msg.angle_min, msg.angle_increment, dynamic_max_view)
        
        # 2 y 3. Burbuja (Se mantiene igual)
        closest_idx = np.argmin(ranges[ranges > 0.0])
        if ranges[closest_idx] > 0:
            theta = math.atan(self.bubble_radius / ranges[closest_idx])
            num_indices = int(theta / msg.angle_increment)
            start_bubble = max(0, closest_idx - num_indices)
            end_bubble = min(len(ranges), closest_idx + num_indices)
            ranges[start_bubble:end_bubble] = 0.0
            
        # 4. Encontrar el Max Gap (Se mantiene igual)
        start_gap, end_gap = self.find_max_gap(ranges)
        
        # 5. Encontrar el centroide de la zona más profunda (Se mantiene igual)
        best_idx = self.find_best_point(start_gap, end_gap, ranges)
        
        # 6. Calcular ángulo de dirección (Aumentar Suavizado: REDUCIR ALPHA)
        raw_steering_angle = msg.angle_min + (best_idx * msg.angle_increment)
        raw_steering_angle = np.clip(raw_steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        # NUEVO: Implementar Deadband (Zona Muerta) de ~2.5 grados
        deadzone = math.radians(2.5)
        if abs(raw_steering_angle) < deadzone:
            raw_steering_angle = 0.0  # Forzar línea recta absoluta
            
        # Aplicar el filtro de suavizado
        alpha = 0.08 
        steering_angle = (alpha * raw_steering_angle) + ((1.0 - alpha) * self.prev_steering_angle)
        self.prev_steering_angle = steering_angle
        
        # 7. Control de Velocidad Continuo (Interpolación Lineal)
        # Calculamos qué porcentaje del giro máximo estamos utilizando (0.0 a 1.0)
        steering_ratio = abs(steering_angle) / self.max_steering_angle
        
        # Reducimos la velocidad proporcionalmente al ángulo de giro
        speed = self.max_speed - (steering_ratio * (self.max_speed - self.min_speed))
        
        # Aseguramos que un cálculo extremo no baje de la velocidad mínima
        speed = max(self.min_speed, speed)
            
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Inicializar posición de inicio
        if self.start_pos is None:
            self.start_pos = (x, y)
            self.lap_start_time = current_time
            self.get_logger().info("🏁 Posición de inicio registrada. Cronómetro iniciado.")
            return

        # Calcular distancia a la posición de inicio
        dist_to_start = math.sqrt((x - self.start_pos[0])**2 + (y - self.start_pos[1])**2)

        # Lógica de conteo de vueltas
        if self.car_left_start and dist_to_start < self.finish_line_radius:
            # ¡Vuelta completada!
            lap_time = current_time - self.lap_start_time
            self.lap_times.append(lap_time)
            self.lap_count += 1
            
            self.get_logger().info(f"✅ Vuelta {self.lap_count}/10 completada en {lap_time:.3f} segundos.")
            
            self.lap_start_time = current_time # Resetear cronómetro
            self.car_left_start = False        # Esperar a que el auto vuelva a salir
            
            if self.lap_count == 10:
                self.race_finished = True
                best_lap = min(self.lap_times)
                self.get_logger().info(f"🏆 ¡COMPETENCIA FINALIZADA! 🏆")
                self.get_logger().info(f"⏱️ Mejor tiempo: {best_lap:.3f} segundos.")
                self.stop_car()

        # Detectar que el auto se ha alejado lo suficiente de la meta
        elif not self.car_left_start and dist_to_start > self.leave_threshold:
            self.car_left_start = True
            
    def stop_car(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.speed = 0.0
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
