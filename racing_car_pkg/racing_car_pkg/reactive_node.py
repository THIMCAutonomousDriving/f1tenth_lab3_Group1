#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool


class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive_gf'

        # Publisher und Subscriber
        self.subscription = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.debug_publisher = self.create_publisher(LaserScan, 'debug_lidar', 10)
        self.subscriber_aeb = self.create_subscription(Bool, 'aeb_stop', self.stop_callback, 10)

        # Modus-Auswahl: Simulation oder echtes Auto
        self.declare_parameter("sim_or_real", "sim")
        self.status = False # AEB Status (False = Fahrt frei, True = Notbremsung)

        # Gemeinsame, neue Parameter (bisher "Magic Numbers")
        self.declare_parameter('disparity_threshold', 0.3)
        self.declare_parameter('steering_penalty', 300.0)
        self.declare_parameter('min_safe_depth', 1.5)
        self.declare_parameter('fov_angle', 100.0) # Sichtfeld in Grad (z.B. 100° für bessere Kurven)

        # Profil-spezifische Parameter laden
        if self.get_parameter("sim_or_real").get_parameter_value().string_value == 'sim':
            self.declare_parameter('bubble_radius', 0.19)
            self.declare_parameter('max_range', 5.11) 
            self.declare_parameter('window_size', 5)
            self.declare_parameter('weight_far', 0.4)
            self.declare_parameter('weight_center', 0.6)
            self.declare_parameter('speed_fast', 1.4)
            self.declare_parameter('speed_medium_fast', 1.0)
            self.declare_parameter('speed_medium', 0.6)
            self.declare_parameter('speed_medium_slow', 0.4)
            self.declare_parameter('speed_slow', 0.3)
            self.declare_parameter('min_gap_size', 30)
            self.get_logger().info('ReactiveFollowGap Node im SIM-Modus initialisiert.')

        else:
            self.declare_parameter('bubble_radius', 0.19)
            self.declare_parameter('max_range', 5.30) 
            self.declare_parameter('window_size', 7)
            self.declare_parameter('weight_far', 0.4)
            self.declare_parameter('weight_center', 0.6)
            self.declare_parameter('speed_fast', 2.5)
            self.declare_parameter('speed_medium_fast', 2.0)
            self.declare_parameter('speed_medium', 1.8)
            self.declare_parameter('speed_medium_slow', 1.6)
            self.declare_parameter('speed_slow', 1.4)
            self.declare_parameter('min_gap_size', 25)
            self.get_logger().info('ReactiveFollowGap Node im REAL-Modus initialisiert.')


    def stop_callback(self, msg):
        # Aktualisiert den Status für die Notbremse
        self.status = msg.data


    def preprocess_lidar(self, ranges):
        # LiDAR Rohdaten glätten
        max_range = self.get_parameter('max_range').get_parameter_value().double_value
        window_size = self.get_parameter('window_size').get_parameter_value().integer_value

        proc = np.array(ranges, dtype=float)
        proc = np.where(np.isfinite(proc), proc, max_range)
        proc = np.clip(proc, 0.0, max_range)

        # Gleitender Mittelwert (Moving Average Filter)
        kernel = np.ones(window_size) / window_size
        pad_width = window_size // 2
        proc_padded = np.pad(proc, (pad_width, pad_width), mode='edge')
        proc = np.convolve(proc_padded, kernel, mode='valid')

        return proc


    def disparity_extender(self, ranges, angle_increment):
        # Verbreitert Hindernisse an abrupten Kanten (Disparitäten), um die Autobreite zu kompensieren.
        extended = ranges.copy()
        bubble_radius = self.get_parameter('bubble_radius').get_parameter_value().double_value
        disparity_threshold = self.get_parameter('disparity_threshold').get_parameter_value().double_value

        # Optimierung: Finde Sprünge via NumPy statt durch 1000+ Iterationen zu loopen
        diffs = np.diff(ranges)
        disparity_indices = np.where(np.abs(diffs) > disparity_threshold)[0]

        for i in disparity_indices:
            left_idx = i
            right_idx = i + 1

            # Künstliche Kanten durch das 0.0 Cropping ignorieren
            if ranges[left_idx] == 0.0 or ranges[right_idx] == 0.0:
                continue

            # Rechts ist näher als links -> Nach links erweitern
            if ranges[right_idx] < ranges[left_idx]:
                close_dist = max(ranges[right_idx], 0.01)
                extend_size = int(np.ceil(np.arctan((bubble_radius / 2.0) / close_dist) / angle_increment))
                start = max(0, right_idx - extend_size)
                # Nur in 'extended' schreiben, Original 'ranges' als Referenz behalten!
                extended[start:right_idx] = np.minimum(extended[start:right_idx], ranges[right_idx])
            
            # Links ist näher als rechts -> Nach rechts erweitern
            else:
                close_dist = max(ranges[left_idx], 0.01)
                extend_size = int(np.ceil(np.arctan((bubble_radius / 2.0) / close_dist) / angle_increment))
                end = min(len(ranges), left_idx + 1 + extend_size)
                extended[left_idx + 1:end] = np.minimum(extended[left_idx + 1:end], ranges[left_idx])

        return extended


    def apply_bubble(self, proc_ranges, angle_increment):
        # Setzt eine Sicherheitsblase um den Punkt, der dem Auto am allernächsten ist.
        bubble_radius = self.get_parameter('bubble_radius').get_parameter_value().double_value

        closest_idx = int(np.argmin(
            np.where(proc_ranges > 0, proc_ranges, np.inf)
        ))
        closest_dist = proc_ranges[closest_idx]

        if closest_dist == 0.0:
            return proc_ranges, closest_idx

        half_angle = bubble_radius / max(closest_dist, 0.01)
        half_steps = int(np.ceil(half_angle / angle_increment))

        bubble_start = max(0, closest_idx - half_steps)
        bubble_end   = min(len(proc_ranges) - 1, closest_idx + half_steps)

        proc_ranges[bubble_start:bubble_end + 1] = 0.0

        return proc_ranges, closest_idx


    def find_max_gap(self, free_space_ranges, angles):
        # Findet die beste fahrbare Lücke basierend auf Breite, Tiefe und Winkel
        steering_penalty = self.get_parameter('steering_penalty').get_parameter_value().double_value

        best_start, best_end = 0, 0
        cur_start            = None
        best_score           = -np.inf

        for i, val in enumerate(free_space_ranges):
            if val > 0.0:
                if cur_start is None:
                    cur_start = i
            else:
                if cur_start is not None:
                    cur_end      = i - 1
                    cur_len      = cur_end - cur_start + 1
                    center_idx   = (cur_start + cur_end) // 2
                    center_angle = abs(angles[center_idx])
                    
                    gap_ranges = free_space_ranges[cur_start:cur_end + 1]
                    max_depth  = np.max(gap_ranges) if len(gap_ranges) > 0 else 0.0

                    # Die überarbeitete Bewertungsfunktion (Score)
                    score = (cur_len * max_depth) - (steering_penalty * (center_angle ** 2))

                    if score > best_score:
                        best_score = score
                        best_start = cur_start
                        best_end   = cur_end

                    cur_start = None

        # Randfall: Lücke geht bis ans Ende des Arrays
        if cur_start is not None:
            cur_end      = len(free_space_ranges) - 1
            cur_len      = cur_end - cur_start + 1
            center_idx   = (cur_start + cur_end) // 2
            center_angle = abs(angles[center_idx])
            
            gap_ranges = free_space_ranges[cur_start:cur_end + 1]
            max_depth  = np.max(gap_ranges) if len(gap_ranges) > 0 else 0.0
            
            score = (cur_len * max_depth) - (steering_penalty * (center_angle ** 2))

            if score > best_score:
                best_start = cur_start
                best_end   = cur_end

        return best_start, best_end


    def find_best_point(self, start_i, end_i, ranges):
        # Bestimmt den exakten Zielpunkt innerhalb der besten Lücke
        weight_far    = self.get_parameter('weight_far').get_parameter_value().double_value
        weight_center = self.get_parameter('weight_center').get_parameter_value().double_value

        gap_ranges = ranges[start_i:end_i + 1]
        furthest_i = start_i + int(np.argmax(gap_ranges))
        center_i   = (start_i + end_i) // 2

        best_i = int(round(weight_far * furthest_i + weight_center * center_i))
        best_i = int(np.clip(best_i, start_i, end_i))

        return best_i


    def lidar_callback(self, data: LaserScan):
        # Notbrems-Check: Steht der AEB auf "Stop", tun wir sofort nichts anderes als bremsen
        if self.status:
            self.get_logger().warn('AEB AKTIV - Notbremsung!', throttle_duration_sec=1.0)
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.speed = 0.0
            self.publisher.publish(drive_msg)
            return

        angle_min       = data.angle_min
        angle_increment = data.angle_increment
        min_gap_size    = self.get_parameter('min_gap_size').get_parameter_value().integer_value
        fov_angle       = self.get_parameter('fov_angle').get_parameter_value().double_value

        # 1. Daten glätten
        proc_ranges = self.preprocess_lidar(data.ranges)

        # 2. Sichtfeld (FoV) einschränken (z.B. auf 100° anstatt starr 90°)
        n      = len(proc_ranges)
        angles = angle_min + np.arange(n) * angle_increment
        proc_ranges[np.abs(angles) > np.radians(fov_angle)] = 0.0

        # 3. Kanten verbreitern (Fahrzeugbreite kompensieren)
        proc_ranges = self.disparity_extender(proc_ranges, angle_increment)

        # 4. Sicherheitsblase um nahestes Objekt
        proc_ranges, _ = self.apply_bubble(proc_ranges, angle_increment)

        # 5. Beste Lücke finden
        start_i, end_i = self.find_max_gap(proc_ranges, angles)

        gap_ranges = proc_ranges[start_i:end_i + 1]
        max_depth = np.max(gap_ranges) if len(gap_ranges) > 0 else 0.0
        min_safe_depth = self.get_parameter('min_safe_depth').get_parameter_value().double_value

        # Sicherheits-Check: Ist die Lücke zu schmal oder eine Sackgasse?
        if end_i - start_i < min_gap_size or (max_depth < min_safe_depth):
            self.get_logger().warn('Lücke zu eng oder Sackgasse - bremse ab.', throttle_duration_sec=0.5)
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.speed          = 0.0
            self.publisher.publish(drive_msg)
            return

        # 6. Exakten Zielpunkt berechnen
        best_i = self.find_best_point(start_i, end_i, proc_ranges)

        # --- Debug Publisher (korrigiert) ---
        # Neues Message-Objekt erzeugen, um Pointer-Probleme in Python zu vermeiden
        debug_msg = LaserScan()
        debug_msg.header = data.header
        debug_msg.angle_min = data.angle_min
        debug_msg.angle_max = data.angle_max
        debug_msg.angle_increment = data.angle_increment
        debug_msg.time_increment = data.time_increment
        debug_msg.scan_time = data.scan_time
        debug_msg.range_min = data.range_min
        debug_msg.range_max = data.range_max
        
        debug_ranges = [0.0] * len(data.ranges)
        debug_ranges[best_i] = float(data.ranges[best_i]) # Nur den gewählten Punkt anzeigen
        debug_msg.ranges = debug_ranges
        self.debug_publisher.publish(debug_msg)
        # --- Ende Debug Publisher ---

        # 7. Lenkwinkel und Geschwindigkeit berechnen
        best_angle = angle_min + best_i * angle_increment
        steer = float(np.clip(best_angle, -0.35, 0.35))
        abs_steer = abs(steer)

        speed_fast        = self.get_parameter('speed_fast').get_parameter_value().double_value
        speed_medium_fast = self.get_parameter('speed_medium_fast').get_parameter_value().double_value
        speed_medium      = self.get_parameter('speed_medium').get_parameter_value().double_value
        speed_medium_slow = self.get_parameter('speed_medium_slow').get_parameter_value().double_value
        speed_slow        = self.get_parameter('speed_slow').get_parameter_value().double_value

        # Dynamisches Geschwindigkeitsprofil basierend auf dem Lenkwinkel
        if abs_steer < 0.15:
            velocity = speed_fast
        elif abs_steer < 0.2:
            velocity = speed_medium_fast
        elif abs_steer < 0.25:
            velocity = speed_medium
        elif abs_steer < 0.3:
            velocity = speed_medium_slow
        else:
            velocity = speed_slow

        # Befehl absenden
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steer
        drive_msg.drive.speed          = velocity
        self.publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("ReactiveFollowGap Node gestartet...")
    reactive_node = ReactiveFollowGap()
    
    try:
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        pass
    finally:
        reactive_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()