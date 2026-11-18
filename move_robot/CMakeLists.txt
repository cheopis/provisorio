cmake_minimum_required(VERSION 3.0.2)
project(move_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

# Instalar módulos Python
catkin_install_python(PROGRAMS
  node/move_robot_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Instalar módulos Python (src/gap_detection/)
install(DIRECTORY src/move_robot/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Instalar outros arquivos
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)