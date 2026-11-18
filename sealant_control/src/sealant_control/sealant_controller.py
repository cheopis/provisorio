#!/usr/bin/env python3
"""
Módulo principal de detecção de gaps para selagem aeronáutica
"""

import rospy
import numpy as np

from time import time
from math import exp
from typing import Optional, Tuple, Dict, List
from dataclasses import dataclass
from enum import Enum

from std_msgs.msg import Float32, Header
from sealant_control.msg import SealingStatus
from .config import SealantControlConfig, SealantMode


class SealantController:
    """
    Classe principal para detecção de gaps em estruturas aeronáuticas
    Processa PointCloud2 do sensor folha de luz e detecta gaps de ~3mm
    """
    
    def __init__(self, config: Optional[SealantControlConfig] = None):
        """
        Inicializa o detector de gaps
        
        Args:
            config: Configuração do detector (opcional)
        """
        if config is None:
            # Use default configuration
            config = SealantControlConfig()

        print(config)
        
        # Init sealant parameters
        print(config.sealant_mode)
        self.K = config.K
        self.pressure = config.delta_P

        if config.sealant_mode == SealantMode.EXPONENTIAL:
            self.sealant_mode = 'exp'

            self.initial_viscosisty = config.n0
            self.viscosity_coef = config.beta

        elif config.sealant_mode == SealantMode.SIGMOIDAL:
            self.sealant_mode = 'sig'

            self.k = config.k
            self.n0 = config.n0
            self.n_max = config.n_max
            self.t_max = config.t_max

        # Start Timer
        self.start = time()
        
        # Histórico para análise temporal
        
        
        # Estatísticas
        
        
        rospy.loginfo(f"✅ SealantController inicializado")
        

    def calculate_sealant_flow(self, viscosity):
        # Calculate flow in cm³/s
        flow = self.K * self.pressure/viscosity

        #rospy.loginfo(f"   Vazão do selante: {flow :.1f} cm³/s")

        # Return flow in m³/s
        return flow*1e-6

    def update_viscosity(self):
        #Uptade time
        t = time() - self.start

        if self.sealant_mode == 'exp':
            # Update exponential viscosity
            viscosity = self.initial_viscosisty*exp(self.viscosity_coef*t)

        elif self.sealant_mode == 'sig':
            # Update sigmoidal viscosity
            viscosity = self.n0 + (self.n_max - self.n0) / (1 + exp(-self.k*(0 - self.t_max)))

        return viscosity, t

    def create_selaing_status_message(self, flow, viscosity, time) -> SealingStatus:
        msg = SealingStatus()
        msg.header = Header()

        msg.time = time
        msg.flow = flow
        msg.viscosity = viscosity

        return msg

    def process_sealing(self) -> SealingStatus:
        viscosity, time = self.update_viscosity()
        flow = self.calculate_sealant_flow(viscosity)

        msg = self.create_selaing_status_message(flow, viscosity, time)

        return msg