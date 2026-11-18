#!/usr/bin/env python3
import rospy
import numpy as np

from math import pi, degrees

from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from iiwa_motion.iiwa_controller import TrajectoryEngine
from gap_detection.msg import GapProfile
# We use a standard service for simplicity, or create a custom .srv
from std_srvs.srv import Trigger, TriggerResponse 
from kuka_ros_node import kuka_iiwa_ros_node

class IiwaControlNode:
    def __init__(self):
        self.client = kuka_iiwa_ros_node()
        #rospy.init_node('iiwa_control')

        self.flow = 0.00002

        # ===================================================================
        # CONFIGURATION
        # ===================================================================
        self.steps = rospy.get_param('~interpolation_steps', 100)
        self.enabled = rospy.get_param('~enabled', True)
        self.pub_rate = rospy.get_param('~publish_rate_hz', 20)
        

        # ===================================================================
        # PUBLISHERS
        # ===================================================================
        self.joint_pub = None

        # ===================================================================
        # SUBSCRIBERS
        # ===================================================================
        # self.joint_sub = rospy.Subscriber(
        #     "/kuka_iiwa/JointPosition", 
        #     String,)
        self.gap_sub = rospy.Subscriber(
            '/gap_detection/profile',
            GapProfile,
            self.gap_callback
        )
        
        self.current_q = None
        self.engine = TrajectoryEngine()
        self.engine.start_client(self.client)

        # Service to trigger the motion (The 'Mission' script will call this or similar)
        # For this specific requirement, we'll listen to parameters updated by the mission script
        self.handle_motion()
        #rospy.Service('~execute_linear_motion', Trigger, self.handle_motion)
        
        rospy.loginfo("IIWA Control Node Ready.")

    def gap_callback(self, profile):
        self.area = profile.depth * profile.current_width

    def handle_motion(self):

        self.current_q = self.client.JointPosition[0]
        
        if self.current_q[0] is None:
            return TriggerResponse(success=False, message="No joint states received yet")

        # Get start/end points from Parameter Server (set by mission script)

        base = np.array([-0.85514, 1.3735, 0.31063])

        start_p = np.array([-0.32, 1.3735, 1.2]) - base
        end_p = np.array([-0.32, 1.3735, 0.6]) - base

        q = [0, pi/2, -pi]

        path, status = self.engine.generate_straight_line(start_p, end_p, 100, self.current_q, q)

        if path is None:
            return TriggerResponse(success=False, message=status)

        # Execute
        rate = rospy.Rate(self.pub_rate)
        for joints in path:
            if rospy.is_shutdown(): break

            vel = 0.1 * self.flow / self.area

            for i in range(0,len(joints)):
                joints[i] = degrees(joints[i])
            msg = f"{' '.join(map(str, joints))}"
            self.client.send_command(f'setPosition {msg}')
            self.client.send_command(f'setJointVelocity {vel}')
            rate.sleep()

        return TriggerResponse(success=True, message="Motion completed")

if __name__ == '__main__':
    node = IiwaControlNode()
    rospy.spin()