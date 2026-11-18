import rospy
import math
import numpy as np
import time

class LineFollower:
    def __init__(self):
        self.time_h = 0
        self.history = None
        # State variables
        self.lines_detected = False
        self.line_data = None
        self.line_angle = -math.pi/2
        self.current_line_center = 0.0
        self.current_line_distance = 0.0

        # Parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.1)  # m/s
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.1)  # rad/s
        self.control_rate = rospy.get_param('~control_rate', 20.0)  # Hz

        # Controller gains
        self.position_gain = rospy.get_param('~position_gain', 0.0005)
        self.position_ki     = rospy.get_param('~position_ki', 0.00012)
        self.orientation_gain = rospy.get_param('~orientation_gain',15)       
        self.orientation_ki  = rospy.get_param('~orientation_ki', 10)

        self.image_width = rospy.get_param('~image_width', 480)
        self.image_height = rospy.get_param('~image_height', 270)
        self.min_line_length = rospy.get_param('~min_line_length', 30)

        self.target_line_center = self.image_width / 2  # Center between lines
        self.target_line_distance = rospy.get_param('~target_line_distance', 0.15)  # Distance between lines
        self.line_tolerance = rospy.get_param('~line_tolerance', 0.03)
        self.min_line_length = rospy.get_param('~min_line_length', 30)  # Minimum line length to consider

    def lines_processing(self, msg):
        """
        process lines get from line_detection topic using Lines message with Line2D[].
        Line2D has: float32 x1, float32 y1, float32 x2, float32 y2
        All coordinates are in pixels.
        """
        try:    
            # Get image dimensions for normalization (if available)
            # You might need to get these from camera info or set as parameters
            
            # Filter lines by minimum length to avoid noise (in pixels)
            valid_lines = []
            for line in msg.lines:
                length = math.sqrt((line.x2 - line.x1)**2 + (line.y2 - line.y1)**2)
                if length >= self.min_line_length:
                    valid_lines.append(line)
            
            if len(valid_lines) == 1:
                rospy.logwarn_throttle(1.0, "Only one valid line after filtering")
                left_line = valid_lines
                right_line = valid_lines
                self.lines_detected = True
                
            elif len(valid_lines) < 1:
                if not self.history:
                    rospy.logwarn_throttle(1.0, "Less than 2 valid lines after filtering")
                    self.lines_detected = False
                    return
                if self.time_h <= 100000:
                    left_line = self.history[0]
                    right_line = self.history[1]
                else: 
                    self.time_h = self.time_h + 1

            else:
                # Sort lines by x position to get left and right lines
                sorted_lines = sorted(valid_lines, key=lambda l: l.center_x)
            
                # Take the two most extreme lines (left and right)
                left_line = sorted_lines[0]
                right_line = sorted_lines[-1]

                self.history = [left_line,right_line]
            # Calculate center between top lines in pixels
            center_pixel = (left_line.center_x + right_line.center_x) / 2.0
            
            # Normalize center to range [-1, 1] where 0 is image center
            # This makes it independent of image resolution
            self.current_line_center = (center_pixel - self.image_width/2.0) / (self.image_width/2.0)
            
            # Calculate distance between lines in pixels
            distance_pixels = math.sqrt(
                (right_line.center_x - left_line.center_x)**2 + 
                (right_line.center_y - left_line.center_y)**2
            )
            
            # Normalize distance relative to image width
            # This gives a value between 0 and 1
            self.current_line_distance = distance_pixels / self.image_width
            
            # Get orientation angle of the lines in radians
            # Use the left line for orientation
            left_angle = left_line.theta
            right_angle = right_line.theta
            
            # Average the angles (handle angle wrapping)
            self.line_angle = math.atan2(
                (math.sin(left_angle) + math.sin(right_angle)) / 2.0,
                (math.cos(left_angle) + math.cos(right_angle)) / 2.0
            )
            print(math.degrees(self.line_angle))
            
            # # Alternative: Use angle between line centers for orientation
            # # This is more robust for parallel lines and gives direction perpendicular to lines
            # center_angle = math.atan2(
            #     right_line.center_y - left_line.center_y,
            #     right_line.center_x - left_line.center_x
            # )
            # # The perpendicular to the center line gives the line orientation
            # self.line_angle = center_angle + math.pi/2.0  # Perpendicular direction
            
            # Store line data for debugging (keep pixel coordinates for reference)
            self.line_data = {
                'left': left_line,
                'right': right_line,
                'center_pixel': center_pixel,
                'center_normalized': self.current_line_center,
                'distance_pixels': distance_pixels,
                'distance_normalized': self.current_line_distance,
                'angle': self.line_angle
            }
            
            self.lines_detected = True
            
            # Log debug info
            rospy.logdebug(f"Lines detected: center={self.current_line_center:.3f} (norm), "
                        f"distance={self.current_line_distance:.3f} (norm), "
                        f"angle={math.degrees(self.line_angle):.2f}°")
            
        except Exception as e:
            rospy.logwarn(f"Error processing line data: {e}")
            self.lines_detected = False

    def filter_lines(self):
        pass

    def compute_control(self):
        """
        Compute control commands for omni-directional robot.
        Returns: (vx, vy, omega) in robot frame
        """
        if not self.lines_detected or self.line_data is None:
            rospy.logwarn_throttle(1.0, "No lines detected, stopping...")
            # Reset integradores ao perder linha
            self._vy_integral = 0.0
            self._omega_integral = 0.0
            self._last_ctrl_time = None
            return 0.0, 0.0, 0.0

        try:
            # ---------- Cálculo de dt ----------
            now = time.time()
            if getattr(self, '_last_ctrl_time', None) is None:
                dt = 1.0 / getattr(self, 'control_rate', 20.0)
            else:
                dt = now - self._last_ctrl_time
                # proteção contra dt inválido
                if dt <= 0.0 or dt > 1.0:
                    dt = 1.0 / getattr(self, 'control_rate', 20.0)
            self._last_ctrl_time = now

            # ---------- Erros ----------
            # Ângulo-alvo (0 = robô alinhado com as linhas)
            desired_angle = self.line_angle
            angle_error = desired_angle - self.line_angle
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            position_error = self.current_line_center  # já normalizado (0 = centro)

            # ---------- Integradores com anti-windup ----------
            # Limites do integrador = output_max / ki
            vy_int_limit    = 0.1  / max(self.position_ki,    1e-6)
            omega_int_limit = 0.15 / max(self.orientation_ki, 1e-6)

            vy_integral    = getattr(self, '_vy_integral',    0.0)
            omega_integral = getattr(self, '_omega_integral', 0.0)

            vy_integral    += position_error * dt
            omega_integral += angle_error    * dt

            vy_integral    = max(-vy_int_limit,    min(vy_integral,    vy_int_limit))
            omega_integral = max(-omega_int_limit, min(omega_integral, omega_int_limit))

            # ---------- Saídas PI ----------
            # vy: kp*erro + ki*integral, saturado em ±0.1
            vy_unsat = -self.position_gain * position_error - self.position_ki * vy_integral
            vy = max(-0.1, min(vy_unsat, 0.1))

            # omega: kp*erro + ki*integral, saturado em ±0.15
            omega_unsat = self.orientation_gain * angle_error + self.orientation_ki * omega_integral
            omega = max(-0.15, min(omega_unsat, 0.15))

            # Anti-windup condicional: só integra se NÃO saturou
            if vy == vy_unsat:
                self._vy_integral = vy_integral
            else:
                self._vy_integral = getattr(self, '_vy_integral', 0.0)

            if omega == omega_unsat:
                self._omega_integral = omega_integral
            else:
                self._omega_integral = getattr(self, '_omega_integral', 0.0)

            # ---------- Velocidade linear frontal ----------
            vx = max(0.1, self.linear_speed)

            print([vx, vy, omega])

            # Ajuste de vx com base na distância entre linhas (como no original)
            if abs(self.current_line_distance - self.target_line_distance) > self.line_tolerance:
                distance_error = self.current_line_distance - self.target_line_distance
                vx *= max(0.1, 1.0 - 0.5 * abs(distance_error) / self.target_line_distance)

            return vx, vy, omega

        except Exception as e:
            rospy.logwarn(f"Error computing control: {e}")
            return 0.0, 0.0, 0.0
    
    
