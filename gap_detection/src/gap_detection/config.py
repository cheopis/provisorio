"""
Configurações e enums para o sistema de detecção de gaps
"""

from enum import Enum
from dataclasses import dataclass
from typing import Optional


class GapDetectionMode(Enum):
    """Modos de operação do detector"""
    STANDARD = "standard"          # Gap contínuo ~3mm
    DISCONTINUOUS = "discontinuous" # Gaps com interrupções
    ADAPTIVE = "adaptive"          # Ajuste automático de parâmetros


@dataclass
class GapDetectionConfig:
    """Configuração do sistema de detecção"""
    # Parâmetros do gap aeronáutico
    nominal_gap_width: float = 0.003  # 3mm em metros
    min_gap_width: float = 0.001      # 1mm mínimo
    max_gap_width: float = 0.005      # 5mm máximo
    gap_tolerance: float = 0.003      # ±1mm tolerância
    
    # Detecção de descontinuidades
    discontinuity_threshold: float = 0.01  # 2mm
    min_discontinuity_length: float = 0.020 # 20mm
    
    # Parâmetros de processamento
    smoothing_sigma: float = 2.0
    edge_detection_sensitivity: float = 0.0005  # 0.5mm
    min_points_for_detection: int = 600
    
    # Filtros
    apply_outlier_filter: bool = True
    outlier_std_dev: float = 3.0
    enable_smoothing: bool = True
    
    # Modo de operação
    detection_mode: GapDetectionMode = GapDetectionMode.STANDARD
    
    # Performance
    max_history_size: int = 100
    
    def __post_init__(self):
        """Validação pós-inicialização"""
        if self.nominal_gap_width <= 0:
            raise ValueError("nominal_gap_width deve ser positivo")
        if self.min_gap_width <= 0:
            raise ValueError("min_gap_width deve ser positivo")
        if self.max_gap_width <= self.min_gap_width:
            raise ValueError("max_gap_width deve ser maior que min_gap_width")