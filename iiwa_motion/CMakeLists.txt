cmake_minimum_required(VERSION 3.0.2)
project(iiwa_motion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  std_srvs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    std_srvs
)

include_directories(${catkin_INCLUDE_DIRS})

# Install scripts
catkin_install_python(PROGRAMS
  nodes/iiwa_motion_node.py
  nodes/path_follower.py
  nodes/kuka_ros_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Installl Python modules (src/iiwa_motion/)
install(DIRECTORY src/iiwa_motion/
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