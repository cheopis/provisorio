#!/usr/bin/env python3
"""
Node ROS principal para detecção de gaps aeronáuticos
"""

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PointStamped

# Importar do pacote gap_detection
from gap_detection.gap_detector import GapDetector
from gap_detection.config import GapDetectionConfig, GapDetectionMode
from gap_detection.msg import GapProfile, GapDiscontinuity


class GapDetectionNode:
    """Node ROS para detecção de gaps em tempo real"""
    
    def __init__(self):
        rospy.init_node('gap_detection_node', anonymous=True)
        
        # ===================================================================
        # CONFIGURAÇÃO DO DETECTOR
        # ===================================================================
        
        # Ler parâmetros ROS
        nominal_gap = rospy.get_param('~nominal_gap', 0.003)  # 3mm
        detection_mode_str = rospy.get_param('~detection_mode', 'standard')
        
        # Converter string para enum
        if detection_mode_str == 'discontinuous':
            detection_mode = GapDetectionMode.DISCONTINUOUS
        elif detection_mode_str == 'adaptive':
            detection_mode = GapDetectionMode.ADAPTIVE
        else:
            detection_mode = GapDetectionMode.STANDARD
        
        # Criar configuração
        config = GapDetectionConfig(
            nominal_gap_width=nominal_gap,
            detection_mode=detection_mode,
            gap_tolerance=rospy.get_param('~gap_tolerance', 0.001),
            discontinuity_threshold=rospy.get_param('~discontinuity_threshold', 0.002),
            smoothing_sigma=rospy.get_param('~smoothing_sigma', 2.0),
            min_points_for_detection=rospy.get_param('~min_points_for_detection', 50)
        )
        
        # Inicializar detector
        self.detector = GapDetector(config)
        
        # ===================================================================
        # PUBLISHERS
        # ===================================================================
        
        # Tópicos de saída
        self.gap_profile_pub = rospy.Publisher(
            '/gap_detection/profile',
            GapProfile,
            queue_size=10
        )
        
        # self.gap_width_pub = rospy.Publisher(
        #     '/gap_detection/current_width',
        #     Float32,
        #     queue_size=10
        # )
        
        self.discontinuity_pub = rospy.Publisher(
            '/gap_detection/discontinuity',
            GapDiscontinuity,
            queue_size=10
        )
        
        # self.left_edge_pub = rospy.Publisher(
        #     '/gap_detection/left_edge',
        #     PointStamped,
        #     queue_size=10
        # )
        
        # self.right_edge_pub = rospy.Publisher(
        #     '/gap_detection/right_edge',
        #     PointStamped,
        #     queue_size=10
        # )
        
        # Tópico de status
        self.status_pub = rospy.Publisher(
            '/gap_detection/status',
            Bool,
            queue_size=1,
            latch=True
        )
        
        # ===================================================================
        # SUBSCRIBERS
        # ===================================================================
        
        # Subscrever tópico do sensor folha de luz
        pointcloud_topic = rospy.get_param('~pointcloud_topic', '/pointcloud')
        rospy.Subscriber(
            pointcloud_topic,
            PointCloud2,
            self.pointcloud_callback,
            queue_size=1
        )
        
        # Subscrever comando de enable/disable
        rospy.Subscriber(
            '/gap_detection/enable',
            Bool,
            self.enable_callback
        )
        
        # ===================================================================
        # VARIÁVEIS DE ESTADO
        # ===================================================================
        
        self.enabled = rospy.get_param('~enabled', True)
        self.publish_rate = rospy.Rate(rospy.get_param('~publish_rate', 30))
        
        # Publicar status inicial
        self._publish_status()
        
        rospy.loginfo(f"🚀 Gap Detection Node iniciado")
        rospy.loginfo(f"   Tópico PointCloud: {pointcloud_topic}")
        rospy.loginfo(f"   Gap nominal: {nominal_gap*1000:.1f}mm")
        rospy.loginfo(f"   Modo: {detection_mode_str}")
        
    def pointcloud_callback(self, msg: PointCloud2):
        """Callback para processamento de PointCloud2"""
        if not self.enabled:
            return
        
        # Processar PointCloud com o detector
        gap_profile, discontinuity = self.detector.process_pointcloud(msg)
        
        if gap_profile:
            # Publicar resultados
            self.gap_profile_pub.publish(gap_profile)
            
            # # Publicar largura separadamente
            # width_msg = Float32()
            # width_msg.data = gap_profile.current_width
            # self.gap_width_pub.publish(width_msg)
            
            # # Publicar bordas para visualização
            # self._publish_edges(gap_profile, msg.header)
            
            # Log informativo periódico
            if rospy.get_time() % 5 < 0.1:  # A cada ~5 segundos
                rospy.loginfo(
                    f"📏 Gap: {gap_profile.current_width*1000:.2f}mm | "
                    #f"Conf: {gap_profile.confidence:.0%} | "
                    #f"Contínuo: {not gap_profile.is_discontinuous}"
                )
        
        if discontinuity:
            self.discontinuity_pub.publish(discontinuity)
            # rospy.logwarn(
            #     f"⚠️  Descontinuidade detectada: {discontinuity.type} | "
            #     f"Severidade: {discontinuity.severity:.1f}"
            # )
    
    def _publish_edges(self, gap_profile: GapProfile, header):
        """Publica posições das bordas para visualização"""
        # Borda inferior (esquerda)
        left_edge_msg = PointStamped()
        left_edge_msg.header = header
        left_edge_msg.point = gap_profile.left_edge
        self.left_edge_pub.publish(left_edge_msg)
        
        # Borda superior (direita)
        right_edge_msg = PointStamped()
        right_edge_msg.header = header
        right_edge_msg.point = gap_profile.right_edge
        self.right_edge_pub.publish(right_edge_msg)
    
    def enable_callback(self, msg: Bool):
        """Habilita/desabilita o processamento"""
        if self.enabled != msg.data:
            self.enabled = msg.data
            self._publish_status()
            
            status_text = "✅ HABILITADO" if self.enabled else "⛔ DESABILITADO"
            rospy.loginfo(f"Gap Detection {status_text}")
    
    def _publish_status(self):
        """Publica status atual do node"""
        status_msg = Bool()
        status_msg.data = self.enabled
        self.status_pub.publish(status_msg)
    
    def run(self):
        """Loop principal do node"""
        rospy.loginfo("Node Gap Detection em execução...")
        
        # Manter node ativo
        while not rospy.is_shutdown():
            self.publish_rate.sleep()


if __name__ == '__main__':
    try:
        node = GapDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node Gap Detection finalizado")