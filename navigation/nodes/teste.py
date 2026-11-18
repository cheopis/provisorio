#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, Pose2D
from line_detection.msg import Lines
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Float32

class OmniLineFollower:
    def __init__(self):
        rospy.init_node('omni_line_follower', anonymous=True)
        
        # Parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.1)  # m/s
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.1)  # rad/s
        self.orientation_gain = rospy.get_param('~orientation_gain', 1.5)
        self.position_gain = rospy.get_param('~position_gain', 0.05)
        self.control_rate = rospy.get_param('~control_rate', 20.0)  # Hz

        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 360)
        
        # Line parameters
        self.target_line_center = self.image_width / 2  # Center between lines
        self.target_line_distance = rospy.get_param('~target_line_distance', 0.15)  # Distance between lines
        self.line_tolerance = rospy.get_param('~line_tolerance', 0.03)
        self.min_line_length = rospy.get_param('~min_line_length', 30)  # Minimum line length to consider
        
        # State variables
        self.current_angle = 0.0  # Robot orientation from odometry
        self.current_position = Pose2D()
        self.lines_detected = False
        self.line_data = None
        self.line_angle = 0.0
        self.current_line_center = 0.0
        self.current_line_distance = 0.0
        self.last_time = rospy.Time.now()
        
        # Publishers and Subscribers
        # self.cmd_pub = rospy.Publisher('/sim_ros_interface/AMR/cmd_vel', Twist, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel_ita', Twist, queue_size=10)
        
        # Subscribe to line detection topic (adjust message type as needed)
        # Assumed message format: [line1_x, line1_y, line2_x, line2_y] or similar
        rospy.Subscriber('/lines_detection/lines', Lines, self.line_callback)
        
        # Subscribe to odometry for robot orientation
        rospy.Subscriber('/sim_ros_interface/AMR/odom', Odometry, self.odom_callback)
        
        # Control loop timer
        self.control_timer = rospy.Timer(rospy.Duration(1.0/self.control_rate), self.control_loop)
        
        rospy.loginfo("Omni Line Follower Node initialized")
        rospy.loginfo(f"Linear speed: {self.linear_speed} m/s, Max angular speed: {self.max_angular_speed} rad/s")
    
    def line_callback(self, msg):
        """
        Callback for line detection data using Lines message with Line2D[].
        Line2D has: float32 x1, float32 y1, float32 x2, float32 y2
        All coordinates are in pixels.
        """
        try:
            # Check if we have at least 2 lines (vertical parallel lines on floor)
            if not hasattr(msg, 'lines') or len(msg.lines) < 2:
                rospy.logwarn_throttle(1.0, "Insufficient lines detected")
                self.lines_detected = False
                return
            
            # Get image dimensions for normalization (if available)
            # You might need to get these from camera info or set as parameters
            image_width = rospy.get_param('~image_width', 640)  # Default 640 pixels
            image_height = rospy.get_param('~image_height', 360)  # Default 480 pixels
            
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
                
            elif len(valid_lines) < 1:
                rospy.logwarn_throttle(1.0, "Less than 2 valid lines after filtering")
                self.lines_detected = False
                return

            elif len(valid_lines) >1:
                # Sort lines by x position to get left and right lines
                sorted_lines = sorted(valid_lines, key=lambda l: l.center_x)
            
                # Take the two most extreme lines (left and right)
                left_line = sorted_lines[0]
                right_line = sorted_lines[-1]
            else: 
                sorted_lines = sorted(valid_lines, key=lambda l: l.center_x)
                [left_line, right_line] = self.filter_lines(sorted_lines)
            
            # Calculate center between lines in pixels
            center_pixel = (left_line.center_x + right_line.center_x) / 2.0
            
            # Normalize center to range [-1, 1] where 0 is image center
            # This makes it independent of image resolution
            self.current_line_center = (center_pixel - image_width/2.0) / (image_width/2.0)
            
            # Calculate distance between lines in pixels
            distance_pixels = math.sqrt(
                (right_line.center_x - left_line.center_x)**2 + 
                (right_line.center_y - left_line.center_y)**2
            )
            
            # Normalize distance relative to image width
            # This gives a value between 0 and 1
            self.current_line_distance = distance_pixels / image_width
            
            # Calculate orientation angle of the lines in radians
            # Use the left line for orientation
            left_angle = math.atan2(left_line.y2 - left_line.y1, left_line.x2 - left_line.x1)
            right_angle = math.atan2(right_line.y2 - right_line.y1, right_line.x2 - right_line.x1)
            
            # Average the angles (handle angle wrapping)
            self.line_angle = math.atan2(
                (math.sin(left_angle) + math.sin(right_angle)) / 2.0,
                (math.cos(left_angle) + math.cos(right_angle)) / 2.0
            )
            
            # Alternative: Use angle between line centers for orientation
            # This is more robust for parallel lines and gives direction perpendicular to lines
            center_angle = math.atan2(
                right_line.center_y - left_line.center_y,
                right_line.center_x - left_line.center_x
            )
            # The perpendicular to the center line gives the line orientation
            self.line_angle = center_angle + math.pi/2.0  # Perpendicular direction
            
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

    def filter_lines(self,valid_lines):
        left_line = None
        right_line = None
        for i in range(0,len(valid_lines)-2):
            distance_pixels = math.sqrt(
                            (valid_lines[i].center_x - valid_lines[i+1].center_x)**2 + 
                            (valid_lines[i].center_y - valid_lines[i+1].center_y)**2)
            if (distance_pixels >= 10) and (distance_pixels <= 40):
                left_line = valid_lines[i]
                right_line = valid_lines[i+1]
            else: 
                continue

        return [left_line, right_line]
    
    def odom_callback(self, msg):
        """Get robot orientation from odometry"""
        # Extract quaternion from odometry
        quat = msg.pose.pose.orientation
        # Convert to euler angles
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.current_angle = math.atan2(siny_cosp, cosy_cosp)
        
        # Update position (optional, for position-based control)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        
    def compute_control(self):
        """
        Compute control commands for omni-directional robot.
        Returns: (vx, vy, omega) in robot frame
        """
        if not self.lines_detected or self.line_data is None:
            rospy.logwarn_throttle(1.0, "No lines detected, stopping...")
            return 0.0, 0.0, 0.0
        
        try:
            # Calculate orientation error (desired angle vs current angle)
            # The robot should align with the lines
            desired_angle = 0.0 #self.line_angle
            
            # Normalize angle error
            angle_error = desired_angle - self.current_angle
            #angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            # Calculate position error (distance from line center)
            position_error = self.current_line_center #- self.target_line_center
            
            # Compute control outputs
            # Linear velocity forward
            vx = max(0.1,self.linear_speed)
            
            # Lateral velocity to correct position
            vy = max(-0.1, min(-self.position_gain * position_error,0.1))
            
            # Angular velocity to correct orientation
            omega = max(-0.15, min(self.orientation_gain * angle_error,0.15))
            
            # Limit angular velocity
            omega = np.clip(omega, -self.max_angular_speed, self.max_angular_speed)
            
            # If lines are too close or too far, adjust forward speed
            if abs(self.current_line_distance - self.target_line_distance) > self.line_tolerance:
                # Adjust speed based on distance error (optional)
                distance_error = self.current_line_distance - self.target_line_distance
                vx *= max(0.1, 1.0 - 0.5 * abs(distance_error) / self.target_line_distance)
            
            return vx, vy, omega
            
        except Exception as e:
            rospy.logwarn(f"Error computing control: {e}")
            return 0.0, 0.0, 0.0
    
    def control_loop(self, event):
        """Main control loop executed at fixed rate"""
        # Compute control commands
        vx, vy, omega = self.compute_control()
        
        # Create and publish Twist message
        cmd_msg = Twist()
        cmd_msg.linear.x = vx *0
        cmd_msg.linear.y = vy *0
        cmd_msg.linear.z = 0.0
        cmd_msg.angular.x = 0.0
        cmd_msg.angular.y = 0.0
        cmd_msg.angular.z = omega *0
        
        # Log debug info (optional)
        if self.lines_detected:
            cmd_msg.linear.x = -0.1 #vx
            cmd_msg.linear.y = vy
            cmd_msg.angular.z = omega * 1.5
            rospy.logdebug(f"vx: {vx:.2f}, vy: {vy:.2f}, omega: {omega:.2f}")

        self.cmd_pub.publish(cmd_msg)
    
    def shutdown(self):
        """Stop robot on shutdown"""
        rospy.loginfo("Shutting down, stopping robot...")
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.linear.y = 0.0
        cmd_msg.angular.z = 0.0
        self.cmd_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        follower = OmniLineFollower()
        rospy.on_shutdown(follower.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass