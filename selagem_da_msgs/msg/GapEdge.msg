# Informações detalhadas sobre uma borda do gap
Header header

# Posição e orientação
geometry_msgs/Point position
geometry_msgs/Quaternion orientation  # Orientação da superfície

# Características geométricas
float32 sharpness               # Nitidez da borda (0-1)
float32 curvature               # Curvatura local
float32 edge_angle              # Ângulo da borda em radianos

# Propriedades de superfície
float32 roughness               # Rugosidade da superfície
float32 reflectivity            # Refletividade para o sensor

# Confiança
float32 detection_confidence    # Confiança na detecção
uint32 point_count              # Número de pontos usados