"""
Configurações e enums para o sistema de controle de velocidade
"""

from enum import Enum
from dataclasses import dataclass
from typing import Optional


@dataclass
class VelocityControlConfig:
    """Configuração do sistema de detecção"""
    # Parâmetro de velocidade
    nominal_velocity: float = 1       # em m/s
    nominal_distance: float = 0.025     # em m

    # Parâmetros de controle PI de velocidade
    P_x: float = 6.2
    I_x: float = 0.0

    # Parâmetro de vazão do selante
    vazao_nominal: float = 6.2
    
    def __post_init__(self):
        """Validação pós-inicialização"""
        if self.nominal_velocity <= 0:
            raise ValueError("nominal_velocity deve ser positivo")
        if self.vazao_nominal <=0:
            raise ValueError("vazao deve ser positivo")