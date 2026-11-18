"""
Configurações e enums para o sistema de detecção de gaps
"""
import yaml
import warnings

from enum import Enum
from dataclasses import dataclass
from typing import Optional
import os

class SealantMode(Enum):
    """Modos de operação do detector"""
    EXPONENTIAL = "exp"
    SIGMOIDAL = "sig"

@dataclass
class SealantControlConfig:
    """Selant Parameters Configuration"""

    # Sealant Mode
    sealant_mode: SealantMode = SealantMode.SIGMOIDAL

    K: float = 0.001            #Pa.s
    delta_P: float = 0.4e6      #Pa

    n0: float = 150       
    beta: float =  0.03         #s⁻1

    k: float = 0.01             #min-1
    n0: float = 150             #Pa
    n_max: float = 1e4       
    t_max: float =  360          #min


    def open_yaml_file(self):
        # Build the path to the config file in another folder
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_file_path = os.path.join(script_dir, '../..', 'config', 'sealant_params.yaml')

        with open(config_file_path, 'r') as f:
            data = yaml.full_load(f)

        self.K = float(data.get('K'))
        self.delta_P = float(data.get('delta_P'))

        return data     

    def open_exp(self, data):
        exp = data.get('exp')

        # Change the values for the yaml file
        self.n0 = float(exp.get('n0'))
        self.beta = float(exp.get('beta'))

    def open_sig(self, data):
        sig = data.get('sig')

        # Change the values for the yaml file
        self.n0 = float(sig.get('n0'))
        self.n_max = float(sig.get('n_max'))
        self.t_max = float(sig.get('t_max'))
        self.k = float(sig.get('k'))

    
    def __post_init__(self):
        """Validação pós-inicialização"""
        try:
            data = self.open_yaml_file()

            if data.get('type') == 'exp':
                sealant_mode: SealantMode = SealantMode.EXPONENTIAL
                self.open_exp(data)

            elif data.get('type') == 'sig':
                sealant_mode: SealantMode = SealantMode.SIGMOIDAL
                self.open_sig(data)
            else:
                raise ValueError("viscosity function type must be exponential(exp) or sigmoidal (sig)")

        except:
            warnings.warn("Error loading yaml file... loading default values")

        # if self.nominal_gap_width <= 0:
        #     raise ValueError("nominal_gap_width deve ser positivo")
        