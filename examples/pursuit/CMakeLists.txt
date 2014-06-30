cmake_minimum_required( VERSION 2.8 )

project( tcs_pursuit_experiment )

find_package( GSL REQUIRED )
find_package( OMPL REQUIRED )
include( FindPkgConfig )
if( PKG_CONFIG_FOUND )
  pkg_check_modules( GAZEBO gazebo )
endif()
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )

SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})

set( TCS_PURSUIT_EXPERIMENT_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/experiment.h
  ${CMAKE_CURRENT_SOURCE_DIR}/aabb.h
  ${CMAKE_CURRENT_SOURCE_DIR}/plane.h
  ${CMAKE_CURRENT_SOURCE_DIR}/moby_xml.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/control_space.h
  ${CMAKE_CURRENT_SOURCE_DIR}/directed_control_sampler.h
  ${CMAKE_CURRENT_SOURCE_DIR}/state_propagator.h
  ${CMAKE_CURRENT_SOURCE_DIR}/goal.h
  ${CMAKE_CURRENT_SOURCE_DIR}/utilities.h
  ${CMAKE_CURRENT_SOURCE_DIR}/constants.h
  ${CMAKE_CURRENT_SOURCE_DIR}/integrator.h
  ${CMAKE_CURRENT_SOURCE_DIR}/space.h
  ${CMAKE_CURRENT_SOURCE_DIR}/ship.h
  ${CMAKE_CURRENT_SOURCE_DIR}/moby_space.h
  ${CMAKE_CURRENT_SOURCE_DIR}/moby_ship.h
#  ${PURSUIT_HEADERS}
)

set( TCS_PURSUIT_EXPERIMENT_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/space.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ship.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/moby_space.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/moby_ship.cpp
#  ${PURSUIT_SOURCES}
)

set( TCS_PURSUIT_EXPERIMENT_LIBRARIES 
  ${OMPL_LIBRARIES}
  ${GSL_LIBRARIES}
)
 
# Copy data files
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/pursuit.xml DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/pursuit.xml DESTINATION ${EXECUTABLE_OUTPUT_PATH} )

add_subdirectory( test )
add_subdirectory( tcs )
add_subdirectory( gazebo )
add_subdirectory( moby )
