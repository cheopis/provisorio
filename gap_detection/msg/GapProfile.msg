# Perfil do gap detectado
Header header

# Largura atual do gap em metros
float32 current_width
float32 nominal_width  # 3mm = 0.003m padrão
uint16 point_step

# Informações de profundidade
float32 depth
float32 distance

# Estado do gap
# bool is_discontinuous
# float32 discontinuity_length

# Posições das bordas
geometry_msgs/Point left_edge
geometry_msgs/Point right_edge

# Confiança da medição (0.0 a 1.0)
# float32 confidence

# Métricas de qualidade
# float32 profile_uniformity
# float32 edge_sharpness