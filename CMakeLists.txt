cmake_minimum_required( VERSION 3.9 )
project( Signal-IO-OpenSim )

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

#SET(CMAKE_CXX_FLAGS "-O3")
#SET(CMAKE_C_FLAGS "-O3")

# Find the OpenSim libraries and header files.
set( OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME} CACHE PATH "Top-level directory of OpenSim install." )
# This command searches for the file OpenSimConfig.cmake
# in common system directories and in OPENSIM_INSTALL_DIR.
find_package( OpenSim 4.0 REQUIRED PATHS "${OPENSIM_INSTALL_DIR}" )

include( ${CMAKE_CURRENT_LIST_DIR}/signal_io/CMakeLists.txt )

include_directories( ${CMAKE_CURRENT_LIST_DIR} )

add_library( OpenSimRobotIO MODULE  ${CMAKE_CURRENT_LIST_DIR}/virtual_robot_osim.cpp )
set_target_properties( OpenSimRobotIO PROPERTIES PREFIX "" )
#set_target_properties( OpenSimRobotIO PROPERTIES INTERPROCEDURAL_OPTIMIZATION True)
target_link_libraries( OpenSimRobotIO ${OpenSim_LIBRARIES} )

add_library( OpenSimActuatorIO MODULE  ${CMAKE_CURRENT_LIST_DIR}/virtual_actuator_osim.cpp )
set_target_properties( OpenSimActuatorIO PROPERTIES PREFIX "" )
#set_target_properties( OpenSimActuatorIO PROPERTIES INTERPROCEDURAL_OPTIMIZATION True)
target_link_libraries( OpenSimActuatorIO ${OpenSim_LIBRARIES} )

add_executable( OpenSimTest ${CMAKE_CURRENT_LIST_DIR}/opensim_test.cpp )
target_link_libraries( OpenSimTest ${OpenSim_LIBRARIES} )
