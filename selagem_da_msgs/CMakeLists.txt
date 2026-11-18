cmake_minimum_required(VERSION 3.0.2)
project(selagem_da_msgs)

# ===========================================================================
# CONFIGURAÇÃO DO COMPILADOR
# ===========================================================================

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ===========================================================================
# ENCONTRAR CATKIN
# ===========================================================================

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  nav_msgs
)

# ===========================================================================
# DEFINIÇÃO DE MENSAGENS
# ===========================================================================

## Mensagens principais do sistema
add_message_files(
  FILES
  # Detecção de gaps
  GapProfile.msg
  GapDiscontinuity.msg
  GapEdge.msg
  #LightSheetConfig.msg
  
  # Controle e comando
  SealingCommand.msg
  #RobotState.msg
  
  # Qualidade e monitoramento
  #SealingQuality.msg
  #SystemStatus.msg
)

# ===========================================================================
# DEFINIÇÃO DE SERVIÇOS
# ===========================================================================

add_service_files(
  FILES
  # Serviços de detecção
  #GetGapProfile.srv
  #CalibrateSensor.srv
  #SetDetectionParameters.srv
  
  # Serviços de controle
  #StartSealing.srv
  #StopSealing.srv
)

# ===========================================================================
# DEFINIÇÃO DE AÇÕES
# ===========================================================================

add_action_files(
  FILES
  # Ações de detecção
  #ScanGap.action
  
  # Ações de monitoramento
  #ContinuousMonitoring.action
  
  # Ações de execução
  #ExecuteSealing.action
)

# ===========================================================================
# GERAR MENSAGENS, SERVIÇOS E AÇÕES
# ===========================================================================

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  nav_msgs
)

# ===========================================================================
# CONFIGURAÇÃO DO CATKIN
# ===========================================================================

catkin_package(
  # Este pacote não gera bibliotecas, apenas mensagens
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
    actionlib_msgs
    nav_msgs
)

# ===========================================================================
# INCLUSÃO DE DIRETÓRIOS
# ===========================================================================

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

# ===========================================================================
# NÃO HÁ TARGETS PARA BUILD - apenas mensagens
# ===========================================================================

# ===========================================================================
# INSTALAÇÃO
# ===========================================================================

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
  FILES_MATCHING PATTERN "*.msg"
)

install(DIRECTORY srv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
  FILES_MATCHING PATTERN "*.srv"
)

install(DIRECTORY action/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/action
  FILES_MATCHING PATTERN "*.action"
)

install(FILES
  package.xml
  README.md
  
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# ===========================================================================
# MENSAGEM FINAL
# ===========================================================================

message(STATUS "")
message(STATUS "==========================================")
message(STATUS "Pacote de mensagens ${PROJECT_NAME} pronto")
message(STATUS "==========================================")
message(STATUS "")