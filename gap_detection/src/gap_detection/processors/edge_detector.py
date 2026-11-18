"""
Detector de bordas para identificação dos limites do gap
"""

import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass
from scipy.signal import find_peaks
from scipy.ndimage import gaussian_filter1d
from geometry_msgs.msg import Point


@dataclass
class EdgeDetectionResult:
    """Resultado da detecção de bordas"""
    left_edge: Point
    right_edge: Point
    depth: float
    distance: float
    edge_sharpness: float
    edge_confidence: float
    success: bool
    message: str


class EdgeDetector:
    """Detecta bordas do gap usando processamento de sinal"""
    
    def __init__(self, sensitivity: float = 0.0005, smoothing_sigma: float = 2.0):
        self.sensitivity = sensitivity  # Sensibilidade para detecção de bordas
        self.smoothing_sigma = smoothing_sigma
        
    def detect_gap_edges(self, points: np.ndarray) -> EdgeDetectionResult:
        """
        Detecta as bordas esquerda e direita do gap
        
        Args:
            points: Pontos ordenados por X [N, 3]
            
        Returns:
            EdgeDetectionResult: Resultado da detecção
        """
        if points is None or len(points) < 20:
            return EdgeDetectionResult(
                left_edge=Point(),
                right_edge=Point(),
                depth=0,
                distance=0,
                edge_sharpness=0.0,
                edge_confidence=0.0,
                success=False,
                message="Pontos insuficientes"
            )
        
        try:
            x_coords = points[:, 0]
            z_coords = points[:, 2]  # Usar Z para detecção (altura/perfil)
            
            # 1. Suavizar sinal Z para reduzir ruído
            if self.smoothing_sigma > 0:
                z_smooth = gaussian_filter1d(z_coords, sigma=self.smoothing_sigma)
            else:
                z_smooth = z_coords

            # 2. Calcular gradiente (mudança de Z em X)
            dz_dx = np.gradient(z_smooth, x_coords)
            
            # 3. Detectar picos no gradiente (bordas)
            edge_indices = self._find_edge_peaks(dz_dx, x_coords)

            # print(edge_indices) #[227, 262]
            
            if len(edge_indices) < 2:
                # Fallback: usar método baseado em threshold
                return self._fallback_edge_detection(points)
                
            elif len(edge_indices) == 2:
                # Indices obtidos pelo detector de picos
                left_idx, right_idx = [edge_indices[0]-1, edge_indices[1]]
                confidence = 0.8
            else:
                # Selecionar melhor par de bordas
                left_idx, right_idx, confidence = self._select_best_edge_pair(
                    edge_indices, dz_dx, x_coords, z_smooth
                )

            # print(points[left_idx,:])
            # print(points[right_idx,:])

            # 4. Calcular nitidez das bordas
            edge_sharpness = self._calculate_edge_sharpness(
                dz_dx[left_idx], dz_dx[right_idx]
            )
            
            # 5. Criar objetos Point para as bordas
            y_mean = np.mean(points[:, 1])  # Y médio (posição na fuselagem)
            
            
            left_edge = Point(
                x=x_coords[left_idx],
                y=y_mean,
                z=z_coords[left_idx]
            )
            
            right_edge = Point(
                x=x_coords[right_idx],
                y=y_mean,
                z=z_coords[right_idx]
            )

            return EdgeDetectionResult(
                left_edge=left_edge,
                right_edge=right_edge,
                depth=self._calculate_gap_depth(points),
                distance=self._calculate_wall_median(points),
                edge_sharpness=edge_sharpness,
                edge_confidence=confidence,
                success=True,
                message="Bordas detectadas com sucesso"
            )
            
        except Exception as e:
            rospy.logerr(f"Erro na detecção de bordas: {e}")
            return EdgeDetectionResult(
                left_edge=Point(),
                right_edge=Point(),
                depth=0.0,
                distance=0.0,
                edge_sharpness=0.0,
                edge_confidence=0.0,
                success=False,
                message=f"Erro: {str(e)}"
            )
    
    def _find_edge_peaks(self, gradient: np.ndarray, x_coords: np.ndarray) -> np.ndarray:
        """Encontra picos no gradiente que correspondem a bordas"""
        if len(gradient) < 10:
            return np.array([])
        
        # Normalizar sensibilidade pelo espaçamento em X
        avg_dx = np.mean(np.diff(x_coords))

        if avg_dx > 0:
            normalized_sensitivity = self.sensitivity / avg_dx
        else:
            normalized_sensitivity = self.sensitivity / 0.0001  # Fallback
        
        # Encontrar picos no sinal
        positive_peaks, _ = find_peaks(
            gradient,
            height=None,
            distance=10,
            prominence=normalized_sensitivity/2
        )
        
        negative_peaks, _ = find_peaks(
            -gradient,
            height=None,
            distance=5,
            prominence=normalized_sensitivity/2
        )
        
        # Combinar todos os picos
        all_peaks = np.sort(np.concatenate([positive_peaks, negative_peaks]))
        
        return all_peaks
    
    def _select_best_edge_pair(self, edge_indices: np.ndarray, gradient: np.ndarray,
                              x_coords: np.ndarray, z_smooth: np.ndarray) -> Tuple[int, int, float]:
        """Seleciona o melhor par de bordas para o gap"""
        
        # Calcular força de cada borda
        edge_strengths = np.abs(gradient[edge_indices])
        
        # Tentar encontrar par com distância próxima do esperado (~3mm)
        expected_gap_mm = 3.0  # 3mm nominal
        expected_gap_m = expected_gap_mm / 1000.0
        
        best_pair = None
        best_score = 0.0
        
        for i in range(len(edge_indices)):
            for j in range(i+1, len(edge_indices)):
                idx1, idx2 = edge_indices[i], edge_indices[j]
                
                # Calcular largura deste par
                width = abs(x_coords[idx2] - x_coords[idx1])
                
                # Calcular simetria (alturas similares)
                height_diff = abs(z_smooth[idx2] - z_smooth[idx1])
                avg_height = (abs(z_smooth[idx1]) + abs(z_smooth[idx2])) / 2
                symmetry = 1.0 - (height_diff / max(avg_height, 0.001))
                
                # Calcular força combinada das bordas
                combined_strength = (edge_strengths[i] + edge_strengths[j]) / 2
                
                # Pontuação baseada na precisão da largura
                width_error = abs(width - expected_gap_m)
                width_accuracy = max(0, 1.0 - (width_error / (expected_gap_m * 2)))
                
                # Pontuação total
                score = (combined_strength * 0.0 + 
                        symmetry * 0.7 + 
                        width_accuracy * 0.3)
                
                if score > best_score:
                    best_score = score
                    best_pair = (idx1, idx2)
        
        if best_pair:
            return best_pair[0], best_pair[1], min(1.0, best_score)
        else:
            # Fallback: usar bordas mais fortes
            strongest_indices = np.argsort(edge_strengths)[-2:]
            return (edge_indices[strongest_indices[0]], 
                   edge_indices[strongest_indices[1]], 
                   0.7)
    
    def _fallback_edge_detection(self, points: np.ndarray) -> EdgeDetectionResult:
        """Método fallback para quando a detecção por picos falha"""
        # Usar threshold baseado na mediana
        median = np.median(points[:, 0])
        std = np.std(points[:, 0])
        
        # Identificar região do gap (X diferente da média)
        threshold = (median + std * 0.5)*0.1
        # print(threshold)

        # Calcula diferenças absolutas em x
        diff = np.abs(np.diff(points[:, 0]))
        # print(max(diff))
        
        # Encontrar transições: índices onde a diferença supera o limiar 
        transitions = np.where(diff > threshold)[0]
        
        if len(transitions) >= 2:
            left_idx = transitions[0]
            right_idx = transitions[-1] + 1
            confidence = 0.7
        else:
            # Último recurso: usar 25% e 75% percentis
            left_idx = int(len(points[:, 0]) * 0.25)
            right_idx = int(len(points[:, 0]) * 0.75)
            confidence = 0.4
        
        y_mean = np.mean(points[:, 1])

        # print([left_idx, right_idx])
        # print(points[left_idx])
        # print(points[right_idx])


        return EdgeDetectionResult(
            left_edge=Point(x=points[left_idx, 0], y=y_mean, z=points[left_idx, 2]),
            right_edge=Point(x=points[right_idx, 0], y=y_mean, z=points[right_idx, 2]),
            depth=self._calculate_gap_depth(points),
            distance=median,
            edge_sharpness=0.5,
            edge_confidence=confidence,
            success=True,
            message="Bordas detectadas via método fallback"
        )
    
    def _calculate_wall_median(self, points: np.ndarray) -> float:
        return np.median(points[:, 0])

    def _calculate_gap_depth(self, points: np.ndarray) -> float:
        diff = np.abs(np.diff(points[:, 0]))
        return max(diff)

    def _calculate_edge_sharpness(self, left_gradient: float, right_gradient: float) -> float:
        """Calcula a nitidez média das bordas (0.0 a 1.0)"""
        avg_gradient = (abs(left_gradient) + abs(right_gradient)) / 2
        # Normalizar para faixa razoável (assumindo que 0.01 é muito nítido)
        sharpness = min(1.0, avg_gradient * 100)
        return sharpness