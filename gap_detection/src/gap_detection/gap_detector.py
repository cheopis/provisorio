#!/usr/bin/env python3
"""
Módulo principal de detecção de gaps para selagem aeronáutica
"""

import rospy
import numpy as np
from typing import Optional, Tuple, Dict, List
from dataclasses import dataclass
from enum import Enum

from sensor_msgs.msg import PointCloud2 # type: ignore
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Point

from gap_detection.msg import GapProfile, GapDiscontinuity
from .config import GapDetectionConfig, GapDetectionMode
from .processors.pointcloud_processor import PointCloudProcessor
from .processors.edge_detector import EdgeDetector, EdgeDetectionResult
from .processors.gap_analyzer import GapAnalyzer, GapAnalysis


class GapDetector:
    """
    Classe principal para detecção de gaps em estruturas aeronáuticas
    Processa PointCloud2 do sensor folha de luz e detecta gaps de ~3mm
    """
    
    def __init__(self, config: Optional[GapDetectionConfig] = None):
        """
        Inicializa o detector de gaps
        
        Args:
            config: Configuração do detector (opcional)
        """
        self.config = config or GapDetectionConfig()
        
        # Inicializar processadores
        self.pointcloud_processor = PointCloudProcessor(
            apply_outlier_filter=self.config.apply_outlier_filter,
            outlier_std_dev=self.config.outlier_std_dev
        )
        
        self.edge_detector = EdgeDetector(
            sensitivity=self.config.edge_detection_sensitivity,
            smoothing_sigma=self.config.smoothing_sigma if self.config.enable_smoothing else 0
        )
        
        self.gap_analyzer = GapAnalyzer(
            nominal_gap=self.config.nominal_gap_width,
            tolerance=self.config.gap_tolerance,
            discontinuity_threshold=self.config.discontinuity_threshold
        )
        
        # Histórico para análise temporal
        self.gap_history: List[float] = []
        self.confidence_history: List[float] = []
        
        # Estatísticas
        self.processing_times: List[float] = []
        self.detection_count = 0
        self.error_count = 0
        
        rospy.loginfo(f"✅ GapDetector inicializado - Modo: {self.config.detection_mode.value}")
        rospy.loginfo(f"   Gap nominal: {self.config.nominal_gap_width*1000:.1f}mm")
    
    def process_pointcloud(self, pointcloud_msg: PointCloud2) -> Tuple[Optional[GapProfile], Optional[GapDiscontinuity]]:
        """
        Processa uma mensagem PointCloud2 e retorna o gap detectado
        
        Args:
            pointcloud_msg: Mensagem ROS PointCloud2 do sensor
            
        Returns:
            Tuple[Optional[GapProfile], Optional[GapDiscontinuity]]: 
                Perfil do gap e descontinuidade (se detectada)
        """
        start_time = rospy.Time.now().to_sec()
        
        try:
            self.detection_count += 1
            
            # 1. Extrair e pré-processar pontos
            points = self.pointcloud_processor.extract_points(pointcloud_msg)
            
            if points is None or len(points) < self.config.min_points_for_detection:
                rospy.logwarn(f"Pontos insuficientes: {len(points) if points else 0}")
                self.error_count += 1
                return None, None
            
            # 2. Ordenar e filtrar pontos para detecção do gap
            sorted_points = self.pointcloud_processor.prepare_for_gap_detection(points)
            
            # 3. Detectar bordas do gap
            edges_result = self.edge_detector.detect_gap_edges(sorted_points)
            
            if not edges_result.success:
                rospy.logwarn(f"Falha na detecção de bordas: {edges_result.message}")
                self.error_count += 1
                return None, None
            
            # 4. Calcular largura do gap
            gap_width = self.calculate_gap_width(edges_result.left_edge, edges_result.right_edge)
            
            # 5. Analisar qualidade e detectar descontinuidades
            gap_analysis = self.gap_analyzer.analyze_gap(
                width=gap_width,
                left_edge=edges_result.left_edge,
                right_edge=edges_result.right_edge,
                points=sorted_points
            )
            
            # 6. Atualizar histórico
            self.update_history(gap_width, gap_analysis.confidence)
            
            # 7. Criar mensagem GapProfile
            gap_profile = self.create_gap_profile_message(
                header=pointcloud_msg.header,
                width=gap_width,
                analysis=gap_analysis,
                edges_result=edges_result
            )
            
            # 8. Criar mensagem de descontinuidade (se aplicável)
            gap_discontinuity = None
            if gap_analysis.is_discontinuous:
                gap_discontinuity = self.create_discontinuity_message(
                    header=pointcloud_msg.header,
                    analysis=gap_analysis
                )
            
            # 9. Registrar estatísticas
            processing_time = rospy.Time.now().to_sec() - start_time
            self.processing_times.append(processing_time)
            
            if len(self.processing_times) > 100:
                self.processing_times.pop(0)
            
            # Log periódico
            if self.detection_count % 100 == 0:
                avg_time = np.mean(self.processing_times) if self.processing_times else 0
                success_rate = ((self.detection_count - self.error_count) / 
                              self.detection_count * 100)
                rospy.loginfo(
                    f"📊 Estatísticas: "
                    f"Processamento: {avg_time*1000:.1f}ms avg | "
                    f"Sucesso: {success_rate:.1f}% | "
                    f"Gap atual: {gap_width*1000:.2f}mm"
                )
            
            return gap_profile, gap_discontinuity
            
        except Exception as e:
            rospy.logerr(f"❌ Erro no processamento do PointCloud: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())
            self.error_count += 1
            return None, None
    
    def calculate_gap_width(self, left_edge: Point, right_edge: Point) -> float:
        """
        Calcula a largura real do gap considerando geometria 3D 
        -> situação real de selagem em fuselagem (geometria não é plana como a parede utilizada nas simulações)
        
        Args:
            left_edge: Ponto da borda esquerda
            right_edge: Ponto da borda direita
            
        Returns:
            float: Largura do gap em metros
        """
        # Para gap horizontal na fuselagem, calcular distância em X
        dx = abs(right_edge.x - left_edge.x)
        dy = abs(right_edge.y - left_edge.y)
        dz = 0.0#abs(right_edge.z - left_edge.z)

        # Distância euclidiana 3D
        width = np.sqrt(dx**2 + dy**2 + dz**2)

        return width
    
    def update_history(self, width: float, confidence: float) -> None:
        """Atualiza o histórico de detecções"""
        self.gap_history.append(width)
        self.confidence_history.append(confidence)
        
        # Manter tamanho limitado do histórico
        if len(self.gap_history) > self.config.max_history_size:
            self.gap_history.pop(0)
            self.confidence_history.pop(0)
    
    def create_gap_profile_message(self, header: Header, width: float, 
                                  analysis: GapAnalysis,
                                  edges_result: EdgeDetectionResult) -> GapProfile:
        """Cria mensagem GapProfile a partir dos resultados da análise"""
        msg = GapProfile()
        msg.header = header
        msg.current_width = width
        msg.nominal_width = self.config.nominal_gap_width
        msg.point_step = 640
        msg.depth = edges_result.depth
        msg.distance = edges_result.distance
        # msg.is_discontinuous = analysis.is_discontinuous
        # msg.discontinuity_length = analysis.discontinuity_length
        msg.left_edge = edges_result.left_edge
        msg.right_edge = edges_result.right_edge
        # msg.confidence = analysis.confidence
        # msg.profile_uniformity = analysis.profile_uniformity
        # msg.edge_sharpness = edges_result.edge_sharpness
        
        return msg
    
    def create_discontinuity_message(self, header: Header, 
                                    analysis: GapAnalysis) -> GapDiscontinuity:
        """Cria mensagem de descontinuidade"""
        msg = GapDiscontinuity()
        msg.header = header
        msg.type = analysis.discontinuity_type.value
        msg.length = analysis.discontinuity_length
        msg.position_x = analysis.discontinuity_position
        msg.severity = analysis.discontinuity_severity
        
        return msg
    
    def get_statistics(self) -> Dict:
        """Retorna estatísticas do detector
        * Utilizar para análise da selagem e artigo do ICAS *
        """
        if not self.gap_history:
            return {}
        
        return {
            'total_detections': self.detection_count,
            'error_rate': self.error_count / max(self.detection_count, 1),
            'avg_gap_width': np.mean(self.gap_history),
            'std_gap_width': np.std(self.gap_history) if len(self.gap_history) > 1 else 0,
            #'avg_confidence': np.mean(self.confidence_history),
            'avg_processing_time': np.mean(self.processing_times) if self.processing_times else 0,
            'history_size': len(self.gap_history)
        }
    
    def reset_statistics(self) -> None:
        """Reseta as estatísticas do detector"""
        self.detection_count = 0
        self.error_count = 0
        self.processing_times.clear()
        self.gap_history.clear()
        self.confidence_history.clear()
        rospy.loginfo("Estatísticas do GapDetector resetadas")