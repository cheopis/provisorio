"""
Analisador de gaps para validação e detecção de descontinuidades
"""

import numpy as np
from typing import Optional
from dataclasses import dataclass
from enum import Enum
from geometry_msgs.msg import Point


class DiscontinuityType(Enum):
    """Tipos de descontinuidades em gaps aeronáuticos"""
    NONE = "none"
    PHYSICAL_INTERRUPTION = "physical_interruption"  # Gap fecha completamente
    ABRUPT_CHANGE = "abrupt_change"                   # Mudança brusca de largura
    EXCESSIVE_GAP = "excessive_gap"                  # Gap muito largo
    DATA_GAP = "data_gap"                            # Falta de dados do sensor


@dataclass
class GapAnalysis:
    """Resultado da análise do gap"""
    width: float
    is_valid: bool
    is_discontinuous: bool
    discontinuity_type: DiscontinuityType
    discontinuity_length: float
    discontinuity_position: float
    discontinuity_severity: float
    confidence: float
    profile_uniformity: float
    validation_message: str


class GapAnalyzer:
    """Analisa gaps detectados para validação e detecção de problemas"""
    
    def __init__(self, nominal_gap: float = 0.003, tolerance: float = 0.003,
                 discontinuity_threshold: float = 0.01):
        self.nominal_gap = nominal_gap
        self.tolerance = tolerance
        self.discontinuity_threshold = discontinuity_threshold
        
        # Histórico para detecção temporal
        self.width_history: list = []
        self.max_history_size = 20
        
    def analyze_gap(self, width: float, left_edge: Point, right_edge: Point,
                   points: Optional[np.ndarray] = None) -> GapAnalysis:
        """
        Analisa um gap detectado
        
        Args:
            width: Largura do gap em metros
            left_edge: Borda esquerda do gap
            right_edge: Borda direita do gap
            points: Pontos usados na detecção (opcional)
            
        Returns:
            GapAnalysis: Resultado da análise
        """
        # Atualizar histórico
        self.width_history.append(width)
        if len(self.width_history) > self.max_history_size:
            self.width_history.pop(0)
        
        # 1. Validar largura básica
        is_valid, validation_msg = self._validate_gap_width(width)
        
        # 2. Verificar descontinuidades
        discontinuity_info = self._check_discontinuities(
            width, left_edge, right_edge, points
        )
        
        # 3. Calcular uniformidade do perfil
        profile_uniformity = self._calculate_profile_uniformity(points) if points is not None else 0.7
        
        # 4. Calcular confiança geral
        confidence = self._calculate_confidence(
            width, is_valid, discontinuity_info, profile_uniformity
        )
        
        return GapAnalysis(
            width=width,
            is_valid=is_valid,
            is_discontinuous=discontinuity_info['is_discontinuous'],
            discontinuity_type=discontinuity_info['type'],
            discontinuity_length=discontinuity_info['length'],
            discontinuity_position=discontinuity_info['position'],
            discontinuity_severity=discontinuity_info['severity'],
            confidence=confidence,
            profile_uniformity=profile_uniformity,
            validation_message=validation_msg
        )
    
    def _validate_gap_width(self, width: float) -> tuple:
        """Valida se a largura do gap está dentro dos limites aceitáveis"""
        # Limites aeronáuticos típicos
        min_acceptable = 0.001  # 1mm
        max_acceptable = 0.005  # 10mm
        
        if width < min_acceptable:
            return False, f"Gap muito estreito: {width*1000:.2f}mm < {min_acceptable*1000:.0f}mm"
        elif width > max_acceptable:
            return False, f"Gap muito largo: {width*1000:.2f}mm > {max_acceptable*1000:.0f}mm"
        elif abs(width - self.nominal_gap) > self.tolerance:
            return True, f"Gap fora da tolerância: {width*1000:.2f}mm (nominal: {self.nominal_gap*1000:.1f}mm)"
        else:
            return True, f"Gap dentro da especificação: {width*1000:.2f}mm"
    
    def _check_discontinuities(self, width: float, left_edge: Point, 
                              right_edge: Point, points: Optional[np.ndarray]) -> dict:
        """Verifica descontinuidades no gap"""
        result = {
            'is_discontinuous': False,
            'type': DiscontinuityType.NONE,
            'length': 0.0,
            'position': (left_edge.x + right_edge.x) / 2,
            'severity': 1.0
        }
        
        # 1. Verificar gap muito pequeno (interrupção física)
        if width < 0.0005:  # < 0.5mm
            result.update({
                'is_discontinuous': True,
                'type': DiscontinuityType.PHYSICAL_INTERRUPTION,
                'length': 0.050,  # Estimativa: 50mm
                'severity': 3.0
            })
            return result
        
        # 2. Verificar mudança brusca em relação ao histórico
        if len(self.width_history) >= 3:
            recent_history = self.width_history[-3:-1]
            if recent_history:
                avg_recent = np.mean(recent_history)
                change = abs(width - avg_recent)
                
                if change > self.discontinuity_threshold:
                    result.update({
                        'is_discontinuous': True,
                        'type': DiscontinuityType.ABRUPT_CHANGE,
                        'length': change * 10,  # Estimativa
                        'severity': 2.0
                    })
                    return result
        
        # 3. Verificar gap excessivamente largo
        if width > (self.nominal_gap + self.tolerance * 10):
            result.update({
                'is_discontinuous': True,
                'type': DiscontinuityType.EXCESSIVE_GAP,
                'length': width - self.nominal_gap,
                'severity': 1.5
            })
            return result
        
        # 4. Verificar lacunas nos dados (se points fornecido)
        if points is not None and len(points) > 10:
            x_coords = points[:, 0]
            dx = np.diff(x_coords)
            max_dx = np.max(dx)
            
            if max_dx > 0.001:  # Lacuna > 1mm nos dados
                gap_indices = np.where(dx > 0.001)[0]
                total_gap = np.sum(dx[gap_indices])
                
                result.update({
                    'is_discontinuous': True,
                    'type': DiscontinuityType.DATA_GAP,
                    'length': total_gap,
                    'severity': 1.0
                })
                return result
        
        return result
    
    def _calculate_profile_uniformity(self, points: np.ndarray) -> float:
        """Calcula a uniformidade do perfil do gap (0.0 a 1.0)"""
        if points is None or len(points) < 10:
            return 0.5
        
        # Usar coordenada Z para análise de uniformidade
        z_coords = points[:, 2]
        
        # Calcular desvio padrão normalizado
        z_mean = np.mean(z_coords)
        z_std = np.std(z_coords)
        
        if z_mean == 0:
            return 0.5
        
        # Uniformidade inversamente proporcional ao desvio padrão normalizado
        cv = z_std / abs(z_mean)  # Coeficiente de variação
        uniformity = 1.0 / (1.0 + cv * 10)  # Transformar para [0, 1]
        
        return max(0.0, min(1.0, uniformity))
    
    def _calculate_confidence(self, width: float, is_valid: bool,
                            discontinuity_info: dict, profile_uniformity: float) -> float:
        """Calcula confiança geral na medição do gap"""
        confidence = 1.0
        
        # Penalizar por gap inválido
        if not is_valid:
            confidence *= 0.6
        
        # Penalizar por descontinuidade
        if discontinuity_info['is_discontinuous']:
            severity = discontinuity_info['severity']
            confidence *= max(0.3, 1.0 - (severity * 0.2))
        
        # Incorporar uniformidade do perfil
        confidence *= (0.3 + profile_uniformity * 0.7)
        
        # Penalizar se está longe do nominal
        width_error = abs(width - self.nominal_gap) / self.nominal_gap
        if width_error > 0.3:  # > 30% de erro
            confidence *= 0.8
        
        return max(0.0, min(1.0, confidence))