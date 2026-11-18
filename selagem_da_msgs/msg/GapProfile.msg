# Perfil completo de um gap detectado
Header header

# Larguras atuais e históricas
float32 current_width          # Largura atual em metros
float32 nominal_width          # Largura nominal esperada (ex: 0.003 = 3mm)
float32[] width_history        # Histórico das últimas N medições

# Características do gap
bool is_discontinuous          # Indica se há descontinuidade
string discontinuity_type      # Tipo: "PHYSICAL_INTERRUPTION", "ABRUPT_CHANGE", "DATA_GAP"
float32 discontinuity_length   # Comprimento da descontinuidade em metros

# Posições das bordas
geometry_msgs/Point left_edge  # Posição da borda esquerda
geometry_msgs/Point right_edge # Posição da borda direita

# Confiança da medição
float32 confidence             # 0.0 a 1.0
float32 measurement_std_dev    # Desvio padrão da medição

# Timestamps
time detection_time            # Quando foi detectado
time processing_time           # Tempo de processamento em segundos