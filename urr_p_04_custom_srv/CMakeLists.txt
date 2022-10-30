cmake_minimum_required(VERSION 3.0.2)
project(urr_p_04_custom_srv)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

# # Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTwoInts.srv
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

# ##################################
# # catkin specific configuration ##
# ##################################
catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(

  # include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/add_two_ints_server.py scripts/add_two_ints_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
