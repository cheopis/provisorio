# Comando para sistema de selagem
Header header

# Controle de velocidade
float32 target_velocity        # Velocidade alvo em m/s
float32 sealant_flow_rate      # Taxa de fluxo do selante em m³/s
float32 acceleration_limit     # Limite de aceleração em m/s²

# Posição e trajetória
geometry_msgs/Pose target_pose
nav_msgs/Path planned_path

# Controle de aplicação
bool start_sealing
bool stop_sealing
bool pause_sealing
float32 sealant_pressure       # Pressão do selante em Pa

# Modo de operação
uint8 operation_mode          # 0=manual, 1=semi-auto, 2=auto
string sealing_pattern        # Padrão de aplicação