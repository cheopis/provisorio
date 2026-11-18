#!/usr/bin/env python3
"""
Node ROS principal para controle e supervisão do selante
"""

import rospy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PointStamped

# Importar do pacote gap_detection
from sealant_control.sealant_controller import SealantController
from sealant_control.config import SealantControlConfig
from sealant_control.msg import SealingStatus


class SealantControlNode:
    """ROS Node for sealant control"""
    
    def __init__(self):
        rospy.init_node('sealant_control_node', anonymous=True)
        
        # ===================================================================
        # CONFIGURAÇÃO DO DETECTOR
        # ===================================================================
        
        # Criar configuração
        # config = SealantControlConfig(
        #     K=rospy.get_param('~K'),
        #     delta_P=rospy.get_param('~delta_P', 0.002),
        #     s=rospy.get_param('~smoothing_sigma', 2.0),
        #     min_points_for_detection=rospy.get_param('~min_points_for_detection', 50)
        # )
        
        # Inicializar detector
        self.controller = SealantController()
        
        # ===================================================================
        # PUBLISHERS
        # ===================================================================
        
        # Tópicos de saída
        self.sealant_status_pub = rospy.Publisher(
            '/sealing_status',
            SealingStatus,
            queue_size=10
        )
        
        # self.servo_cmd_pub = rospy.Publisher(
        #     '/servo_cmd',
        #     MsgType,
        #     queue_size=10
        # )
        
        # ===================================================================
        # SUBSCRIBERS
        # ===================================================================
        
        # Subscrever tópico do servo
        # rospy.Subscriber(
        #     '',
        #     ,
        #     self.servo_callback
        # )
        
        # ===================================================================
        # VARIÁVEIS DE ESTADO
        # ===================================================================
        
        self.enabled = rospy.get_param('~enabled', True)
        self.publish_rate = rospy.Rate(rospy.get_param('~publish_rate', 30))
        
        # Publicar status inicial
        self._publish_status()
        
        rospy.loginfo(f"🚀 Sealing Controler Node Started")
        
    def servo_callback(self, msg):
        pass
    
    def _publish_status(self):
        """Publica posições das bordas para visualização"""
        status = self.controller.process_sealing()
        self.sealant_status_pub.publish(status)
        
    
    def enable_callback(self, msg: Bool):
        """Habilita/desabilita o processamento"""
        if self.enabled != msg.data:
            self.enabled = msg.data
            self._publish_status()
            
            status_text = "✅ HABILITADO" if self.enabled else "⛔ DESABILITADO"
            rospy.loginfo(f"Gap Detection {status_text}")
    
    
    def run(self):
        """Loop principal do node"""
        rospy.loginfo("Node Gap Detection em execução...")
        
        # Manter node ativo
        while not rospy.is_shutdown():
            self._publish_status()
            self.publish_rate.sleep()


if __name__ == '__main__':
    try:
        node = SealantControlNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node Gap Detection finalizado")