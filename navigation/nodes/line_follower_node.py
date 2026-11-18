#!/usr/bin/env python3
import rospy
import math

from geometry_msgs.msg import Twist, Pose2D
from line_detection.msg import Lines
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Float32

from navigation.line_follower import LineFollower

class LineFollowerNode:
    def __init__(self):
        rospy.init_node('line_follower_node', anonymous=True)

        # ===================================================================
        # CONFIGURATION
        # ===================================================================
        coppeliaSim = False

        # Parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.1)  # m/s
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.1)  # rad/s
        self.orientation_gain = rospy.get_param('~orientation_gain', 0.0015)
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
        self.last_time = rospy.Time.now()

        self.follower = LineFollower()

        # ===================================================================
        # PUBLISHERS
        # ===================================================================
        if coppeliaSim:
            self.cmd_pub = rospy.Publisher(
                '/sim_ros_interface/AMR/cmd_vel', 
                Twist, 
                queue_size=10)
        else:    
            self.cmd_pub = rospy.Publisher(
                '/cmd_vel_ita', 
                Twist, 
                queue_size=10)
        

        # ===================================================================
        # SUBSCRIBERS
        # ===================================================================
        self.lines_sub = rospy.Subscriber(
            '/lines_detection/lines', 
            Lines,
            self.line_detection_callback)

        if coppeliaSim:
            self.odometry = rospy.Subscriber(
                '/sim_ros_interface/AMR/odom', 
                Odometry, 
                self.odom_callback)

        rospy.loginfo("Navigation Node: Line Follower Mode Node Ready.")

    def line_detection_callback(self, msg):
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
            
            self.follower.lines_processing(msg)
            self.lines_detected = True
            self.publish_velocity()
            
        except Exception as e:
            rospy.logwarn(f"Error processing line data: {e}")
            self.lines_detected = False

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

    def publish_velocity(self):
        """Main control loop executed at fixed rate"""
        # Compute control commands
        vx, vy, omega = self.follower.compute_control()
        
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
            cmd_msg.linear.x = -vx #vx
            cmd_msg.linear.y = -vy
            cmd_msg.angular.z = omega
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
        node = LineFollowerNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass