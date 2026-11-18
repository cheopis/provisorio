#!/usr/bin/env python3
"""
Node ROS principal para movimentação do robô
"""

import rospy
from time import time
from datetime import datetime
from statistics import mean, median, stdev

from collections import deque
from geometry_msgs.msg import Twist, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry

# Importar do pacote move_robot
from move_robot.velocity_controller import VelocityController
from move_robot.config import VelocityControlConfig
from gap_detection.msg import GapProfile, GapDiscontinuity
from sealant_control.msg import SealingStatus

class MoveRobotNode:

    def __init__(self):
        rospy.init_node('move_robot_node', anonymous=True)

        # ===================================================================
        # CONFIGURAÇÃO DO NÓ
        # ===================================================================

        # Inicializar pose vazia
        self.pose = None

        # Inicializar gap profile vazio
        self.gap_profile = None
        self.discontinuity = False

        # Ler parâmetros ROS
        nominal_velocity = rospy.get_param('~nominal_velocity', 0.3)  # 3 m/s
        
        # Criar configuração
        config = VelocityControlConfig(
            nominal_velocity=nominal_velocity,
            nominal_distance=rospy.get_param('~nominal_distance', 0.025),
            P_x=rospy.get_param('~P_x', 0.5),
            I_x=rospy.get_param('~I_x', 0.0),
            vazao_nominal=rospy.get_param('~vazao_nominao',0.3)
        )

        # Inicializar controle
        self.vel_hist = deque(maxlen=10)
        self.controller = VelocityController()
        self.file = datetime.now().strftime("benchmark_%Y-%m-%d_%H-%M-%S.log")

        # ===================================================================
        # PUBLISHERS
        # ===================================================================
        self.cmd_vel = rospy.Publisher(
            '/sim_ros_interface/AMR/cmd_vel',
            Twist,
            queue_size = 10
        )

        # ===================================================================
        # SUBSCRIBERS
        # ===================================================================
        rospy.Subscriber(
            '/gap_detection/profile',
            GapProfile,
            self.gap_callback
        )

        rospy.Subscriber(
            '/gap_detection/discontinuity',
            GapDiscontinuity,
            self.discontinuity_callback
        )

        rospy.Subscriber(
            '/sealing_status',
            SealingStatus,
            self.sealing_callback
        )

        rospy.Subscriber(
            '/sim_ros_interface/AMR/odom',
            Odometry,
            self.odometry_callback
        )

        # ===================================================================
        # VARIÁVEIS DE ESTADO
        # ===================================================================
        self.publish_rate = rospy.Rate(rospy.get_param('~publish_rate', 60))


    def gap_callback(self, msg: GapProfile):
        """Callback para processamento do perfil do gap"""
        if not self.pose:
            return

        self.gap_profile = msg

    def discontinuity_callback(self, msg: GapDiscontinuity):
        if not self.pose:
            return
        
        if msg.type == "excessive_gap":
            self.discontinuity = True
        else:
            self.discontinuity = False
        
    def sealing_callback(self, msg):

        inicio = time()

        if self.discontinuity:
            with open("positions.txt", "w") as f:
                f.write(f"{self.pose.position.x},{self.pose.position.y}\n")
            
            self.cmd_vel.publish(self.vel_hist[-1])
            self.salvar_linha(100, self.vel_hist[-1].linear.y)
            return

        if not self.gap_profile:
            return

        velocities = self.controller.update_velocities(self.gap_profile, self.pose, msg)

        if velocities:
            self.cmd_vel.publish(velocities)

            if self.vel_hist and abs(velocities.linear.y - self.vel_hist[-1].linear.y) > 0.005:
                velocities = self.vel_hist[-1]

        self.vel_hist.append(velocities)

        # print(self.vel_hist[-1].linear.y)

        fim = time()
        elapsed = fim - inicio
        self.salvar_linha(elapsed, velocities.linear.y)

    def salvar_linha(self, tempo: float, vel) -> None:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with open(self.file, "a", encoding="utf-8") as f:
            f.write(f"{tempo:.6f}; {self.pose.position.y}, {self.gap_profile.current_width}; {vel}\n")


    def odometry_callback(self, msg: Odometry):
        """Callback para atualização da pose do AMR"""
        
        self.pose = msg.pose.pose
        

    def run(self):
        """Loop principal do node"""
        rospy.loginfo("Node Move Robot em execução...")
        
        # Manter node ativo
        while not rospy.is_shutdown():
            self.publish_rate.sleep()


if __name__ == '__main__':
    try:
        node = MoveRobotNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node Move Robot finalizado")