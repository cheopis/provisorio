"""
Processador de PointCloud2 para extração e pré-processamento de pontos
"""

import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point


@dataclass
class PointCloudProcessingResult:
    """Resultado do processamento de PointCloud"""
    points: np.ndarray
    num_points: int
    bounding_box: Tuple[float, float, float, float, float, float]  # min/max x,y,z
    success: bool
    message: str


class PointCloudProcessor:
    """Processa mensagens PointCloud2 para detecção de gaps"""
    
    def __init__(self, apply_outlier_filter: bool = True, outlier_std_dev: float = 3.0):
        self.apply_outlier_filter = apply_outlier_filter
        self.outlier_std_dev = outlier_std_dev
        
    def extract_points(self, cloud_msg: PointCloud2) -> Optional[np.ndarray]:
        """
        Extrai pontos de uma mensagem PointCloud2 para array numpy
        
        Args:
            cloud_msg: Mensagem ROS PointCloud2
            
        Returns:
            Optional[np.ndarray]: Array de pontos [N, 3] ou None se falhar
        """
        try:
            points_list = []
            
            # Extrair pontos usando pc2.read_points (eficiente)
            for point in pc2.read_points(cloud_msg, 
                                        field_names=("x", "y", "z"), 
                                        skip_nans=True):
                points_list.append([point[0], point[1], point[2]])
            
            if not points_list:
                return None
            
            points_array = np.array(points_list)
            
            # Verificar dimensões
            if points_array.shape[1] != 3:
                rospy.logwarn(f"Dimensões inválidas: {points_array.shape}")
                return None
            
            return points_array
            
        except Exception as e:
            rospy.logerr(f"Erro na extração de pontos: {e}")
            return None
    
    def prepare_for_gap_detection(self, points: np.ndarray) -> np.ndarray:
        """
        Pré-processa pontos para detecção de gaps
        
        Args:
            points: Array de pontos [N, 3]
            
        Returns:
            np.ndarray: Pontos pré-processados e ordenados
        """
        if points is None or len(points) == 0:
            return points
        
        # 1. Remover NaN e infinitos
        valid_mask = ~(np.isnan(points).any(axis=1) | np.isinf(points).any(axis=1))
        points = points[valid_mask]
        
        if len(points) < 10:
            return points

        # 2. Remover outliers estatisticamente (se habilitado)
        #if self.apply_outlier_filter and len(points) > 20:
        #    points = self._remove_statistical_outliers(points)
        
        return points
    
    def _remove_statistical_outliers(self, points: np.ndarray) -> np.ndarray:
        """Remove outliers usando filtro estatístico"""
        # Calcular médias e desvios padrão para cada eixo
        means = np.mean(points, axis=0)
        stds = np.std(points, axis=0)
        
        # Criar máscara para pontos dentro de N desvios padrão
        mask = np.all(
            np.abs(points - means) < (self.outlier_std_dev * stds),
            axis=1
        )
        
        filtered_points = points[mask]
        
        if len(filtered_points) < len(points) * 0.9:  # Se removeu mais de 50%
            rospy.logwarn(f"Filtro removeu muitos pontos: {len(points)} -> {len(filtered_points)}")
            return points  # Retornar original se filtro muito agressivo
        
        return filtered_points