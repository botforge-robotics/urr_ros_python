cmake_minimum_required(VERSION 3.0.2)
project(urr_p_03_services)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

# ##################################
# # catkin specific configuration ##
# ##################################
catkin_package()

# ##########
# # Build ##
# ##########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ############
# # Install ##
# ############
catkin_install_python(PROGRAMS scripts/server.py scripts/client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
