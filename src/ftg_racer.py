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
        
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('max_speed', 10.5)
        self.declare_parameter('is_ego', True) 

        scan_topic = str(self.get_parameter('scan_topic').value)
        drive_topic = str(self.get_parameter('drive_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        
        self.max_speed = float(self.get_parameter('max_speed').value)
        ego_param = str(self.get_parameter('is_ego').value).strip().lower()
        self.is_ego = ego_param in ['true', '1', 't', 'y', 'yes']
        
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        
        self.min_speed = 5.0       
        self.max_steering_angle = math.radians(30) 
        
        self.lap_count = 0
        self.lap_times = []
        self.start_pos = None
        self.lap_start_time = None
        self.car_left_start = False
        self.finish_line_radius = 1.5  
        self.leave_threshold = 4.0     
        self.race_finished = False
        self.prev_steering_angle = 0.0
        
        self.init_time = None
        
        role = "PILOTO PRINCIPAL" if self.is_ego else "OPONENTE"
        self.get_logger().info(f"Iniciado como: {role} | Vel Max: {self.max_speed} m/s")

    def preprocess_lidar(self, ranges, angle_min, angle_increment, max_view):
        proc_ranges = np.array(ranges)
        proc_ranges = np.nan_to_num(proc_ranges, nan=0.0, posinf=10.0, neginf=0.0)
        proc_ranges = np.clip(proc_ranges, 0.0, max_view)
        
        fov_min = -math.pi / 2.0
        fov_max = math.pi / 2.0
        start_idx = int((fov_min - angle_min) / angle_increment)
        end_idx = int((fov_max - angle_min) / angle_increment)
        
        proc_ranges[:start_idx] = 0.0
        proc_ranges[end_idx:] = 0.0
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        non_zero = np.where(free_space_ranges > 0.0)[0]
        if len(non_zero) == 0:
            return 0, len(free_space_ranges) - 1
            
        splits = np.split(non_zero, np.where(np.diff(non_zero) != 1)[0] + 1)
        max_gap = max(splits, key=len)
        return max_gap[0], max_gap[-1]

    def find_best_point(self, start_i, end_i, ranges):
        gap_slice = ranges[start_i:end_i+1]
        max_val = np.max(gap_slice)
        
        max_indices = np.where(gap_slice >= max_val - 0.05)[0]
        center_idx = int(np.mean(max_indices))
        return start_i + center_idx
    
    def scan_callback(self, msg):
        if self.race_finished:
            return 
            
        raw_ranges = np.array(msg.ranges)
        raw_ranges = np.nan_to_num(raw_ranges, nan=0.0, posinf=10.0, neginf=0.0)
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.init_time is None:
            self.init_time = current_time
        is_start_of_race = (current_time - self.init_time) < 3.0
        
        # Radar frontal
        cone_start = int((math.radians(-15) - msg.angle_min) / msg.angle_increment)
        cone_end = int((math.radians(15) - msg.angle_min) / msg.angle_increment)
        frontal_rays = raw_ranges[cone_start:cone_end]
        valid_front = frontal_rays[frontal_rays > 0.1]
        front_dist = np.min(valid_front) if len(valid_front) > 0 else 6.0
        
        # --- INTELIGENCIA DINÁMICA ---
        if self.is_ego:
            if front_dist < 4.5:
                # MODO MISIL
                view_distance = 3.5
                base_bubble = 0.25 
                base_padding = 0.20  
                alpha = 0.55         
            else:
                # MODO CRUCERO
                view_distance = max(3.5, min(6.5, front_dist + 1.0))
                base_bubble = 0.40
                base_padding = 0.35
                alpha = 0.30        
        else:
            # Comportamiento estable e inmutable para los oponentes
            view_distance = 2.2
            base_bubble = 0.35
            base_padding = 0.35 
            alpha = 0.70  
            
        ranges = self.preprocess_lidar(msg.ranges, msg.angle_min, msg.angle_increment, view_distance)
        
        valid_indices = np.where(ranges > 0.0)[0]
        if len(valid_indices) > 0:
            closest_idx = valid_indices[np.argmin(ranges[valid_indices])]
            closest_dist = ranges[closest_idx]
            
            if is_start_of_race:
                current_bubble = min(base_bubble, closest_dist * 0.7)
                current_padding = min(base_padding, closest_dist * 0.6)
            else:
                current_bubble = base_bubble
                current_padding = base_padding
                
            ranges[ranges < current_padding] = 0.0
            
            if closest_dist > 0.05:
                theta = math.atan(current_bubble / closest_dist)
                num_indices = int(theta / msg.angle_increment)
                start_bubble = max(0, closest_idx - num_indices)
                end_bubble = min(len(ranges), closest_idx + num_indices)
                ranges[start_bubble:end_bubble] = 0.0
            
        start_gap, end_gap = self.find_max_gap(ranges)
        best_idx = self.find_best_point(start_gap, end_gap, ranges)
        
        raw_steering_angle = msg.angle_min + (best_idx * msg.angle_increment)
        raw_steering_angle = np.clip(raw_steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        # Si el volante está casi derecho, pisa a fondo ignorando el ruido
        if abs(raw_steering_angle) < math.radians(8):
            speed = self.max_speed
        else:
            braking_ratio = abs(raw_steering_angle) / self.max_steering_angle
            speed = self.max_speed - (braking_ratio * (self.max_speed - self.min_speed))
        
        # Frenado en Modo Misil
        if self.is_ego and front_dist < 3.5 and speed > 6.5:
            speed = 6.5  # Baja la velocidad solo un instante para asegurar la maniobra de rebase
                
        speed = max(self.min_speed, speed)

        deadzone = math.radians(2.5)
        if abs(raw_steering_angle) < deadzone:
            raw_steering_angle = 0.0  
            
        steering_angle = (alpha * raw_steering_angle) + ((1.0 - alpha) * self.prev_steering_angle)
        self.prev_steering_angle = steering_angle
            
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

    def odom_callback(self, msg):
        if not self.is_ego:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.start_pos is None:
            self.start_pos = (x, y)
            self.lap_start_time = current_time
            self.get_logger().info("Posición de inicio registrada. Cronómetro iniciado.")
            return

        dist_to_start = math.sqrt((x - self.start_pos[0])**2 + (y - self.start_pos[1])**2)

        if self.car_left_start and dist_to_start < self.finish_line_radius:
            lap_time = current_time - self.lap_start_time
            self.lap_times.append(lap_time)
            self.lap_count += 1
            
            self.get_logger().info(f"Vuelta {self.lap_count}/10 completada en {lap_time:.3f} s.")
            self.lap_start_time = current_time 
            self.car_left_start = False        
            
            if self.lap_count == 10:
                self.race_finished = True
                best_lap = min(self.lap_times)
                self.get_logger().info(f"MEJOR TIEMPO: {best_lap:.3f} s.")
                self.stop_car()

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
