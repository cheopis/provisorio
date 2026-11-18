cmake_minimum_required(VERSION 3.0.2)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# Instalar módulos Python
catkin_install_python(PROGRAMS
  nodes/line_follower_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Instalar módulos Python (src/gap_detection/)
install(DIRECTORY src/navigation/
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