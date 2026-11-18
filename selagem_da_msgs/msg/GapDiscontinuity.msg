# Detecção específica de uma descontinuidade
Header header

# Tipo e localização
string type                    # "PHYSICAL_INTERRUPTION", "ABRUPT_CHANGE", "EXCESSIVE_GAP"
geometry_msgs/Point location   # Posição central da descontinuidade
float32 length                 # Comprimento em metros
float32 width_before           # Largura antes da descontinuidade
float32 width_after            # Largura após a descontinuidade

# Severidade
uint8 severity                 # 1=baixa, 2=média, 3=alta, 4=crítica
string description             # Descrição textual

# Metadados
time detection_time
string sensor_id               # ID do sensor que detectou