"""
Configurações e enums para o sistema de detecção de gaps
"""
import yaml
import warnings

from enum import Enum
from dataclasses import dataclass
from typing import Optional
import os

@dataclass
class SealantControlConfig:
    """Selant Parameters Configuration"""

    K: float = 0.001            #Pa.s
    delta_P: float = 0.4e6      #Pa
    n0: float = 150       
    beta: float =  0.03         #s⁻1

    def open_yaml_file_exp(self):
        # Build the path to the config file in another folder
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_file_path = os.path.join(script_dir, '../..', 'config', 'sealant_params.yaml')

        with open(config_file_path, 'r') as f:
            data = yaml.full_load(f)
            
        exp = data.get('exp')

        # Change the values for the yaml file
        self.K = float(exp.get('K'))
        self.delta_P = float(exp.get('delta_P'))
        self.n0 = float(exp.get('n0'))
        self.beta = float(exp.get('beta'))
    
    def __post_init__(self):
        """Validação pós-inicialização"""

        try:
            self.open_yaml_file_exp()

        except:
            warnings.warn("Error loading yaml file... loading default values")

        # if self.nominal_gap_width <= 0:
        #     raise ValueError("nominal_gap_width deve ser positivo")
        