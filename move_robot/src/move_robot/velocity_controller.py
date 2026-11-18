#!/usr/bin/env python3
"""
Módulo principal de movimentação da base móvel
"""

import rospy
import numpy as np
from typing import Optional, Tuple, Dict, List

from geometry_msgs.msg import Twist, Pose, Vector3
from gap_detection.msg import GapProfile
from .processors.gapprofile_processor import GapDetectionMsgUnpacker, GapData

from .config import VelocityControlConfig

class VelocityController:
    """
    Classe principal movimentação da base móvel no ambiente
    Processa GapProfile contendo dados para estimativa da distância da parede e corrige sua movimentação
    de modo a manter uma distância constante da fuselagem
    """
    def __init__(self, config: Optional[VelocityControlConfig] = None):
        """
        Inicializa o controle da base móvel
        """

        self.vazao = 0

        self.config = config or VelocityControlConfig()

        rospy.loginfo(f"✅ RobotControl inicializado")
        rospy.loginfo(f"   Velocidades nominais: eixo x:  0.0 m/s \n")
        rospy.loginfo(f"                         eixo y: -0.5 m/s \n")
        rospy.loginfo(f"                          theta:  0.0 m/s \n")
                       

    def estimate_distance(self, msg: GapProfile):
        
        profile = msg #self.process_gap_profile(msg)

        distance = profile.distance
        area = profile.depth * profile.current_width # in m²
        #print([distance, area])

        return distance, area
    
    def process_gap_profile(self, msg: GapProfile):
        # Create unpacker instance
        unpacker = GapDetectionMsgUnpacker(verbose=False)
        
        # Set get msg
        unpacker._gap_callback(msg)
        data = unpacker.get_latest_gap_data()

        return data

    def calculate_velocity(self, distance, area, flow):
        linear_vel = Vector3()
        angular_vel = Vector3()

        if type(distance) == Pose:
            linear_vel.x = -(distance.position.y - self.config.nominal_distance) * 0.01 #self.config.P_x
        else:
            linear_vel.x = -(distance - self.config.nominal_distance) * 0.01 #self.config.P_x
        
        #print(linear_vel.x)

        linear_vel.y = -0.1 * self.config.nominal_velocity * flow / (area * 0.2)
        linear_vel.z = 0.0

        angular_vel.x = 0.0
        angular_vel.y = 0.0
        angular_vel.z = 0.0

        return linear_vel, angular_vel

    def update_velocities(self, profile: GapProfile, pose: Pose, sealant_flow) -> Twist:

        distance, area = self.estimate_distance(profile)
        
        linear_vel, angular_vel = self.calculate_velocity(pose.position.y -1, area, sealant_flow.flow)

        velocities = Twist()
        velocities.linear = linear_vel
        velocities.angular = angular_vel

        return velocities
