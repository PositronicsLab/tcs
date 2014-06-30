cmake_minimum_required( VERSION 2.8 )

project( gazebo_pursuit_experiment )

find_package( RT REQUIRED )
find_package( DL REQUIRED )
find_package( Pthread REQUIRED )
find_package( Moby REQUIRED )
find_package( OMPL REQUIRED )
find_package( GSL REQUIRED )
find_package( Boost REQUIRED)

include( FindPkgConfig )
if( PKG_CONFIG_FOUND )
  pkg_check_modules( GAZEBO gazebo )
endif()
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )

include( FindPkgConfig )
if( PKG_CONFIG_FOUND )
  pkg_check_modules( RAVELIN ravelin )
endif()
include_directories( ${RAVELIN_INCLUDE_DIRS} )
link_directories( ${RAVELIN_LIBRARY_DIRS} )

set( GAZEBO_PURSUIT_EXPERIMENT_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/gazebo_space.h
  ${CMAKE_CURRENT_SOURCE_DIR}/gazebo_ship.h
  ${TCS_PURSUIT_EXPERIMENT_HEADERS}
)

set( GAZEBO_PURSUIT_EXPERIMENT_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/gazebo_space.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/gazebo_ship.cpp
  ${TCS_PURSUIT_EXPERIMENT_SOURCES}
)

set( LIBS 
  ${TCS_LIBRARY}
  ${TCS_LIBRARIES}
  ${GAZEBO_LIBRARIES} 
  ${TCS_PURSUIT_EXPERIMENT_LIBRARIES} 
) 

include_directories( ${TCS_INCLUDE_DIR} )

add_library( pursuit-gazebo-prey-controller SHARED prey_controller.cpp prey_controller.h ${GAZEBO_PURSUIT_EXPERIMENT_HEADERS} ${GAZEBO_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( pursuit-gazebo-prey-controller ${LIBS} )

add_library( pursuit-gazebo-predator-controller SHARED predator_controller.cpp predator_controller.h ${GAZEBO_PURSUIT_EXPERIMENT_HEADERS} ${GAZEBO_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( pursuit-gazebo-predator-controller ${LIBS} )

# Copy data files
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${LIBRARY_OUTPUT_PATH} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/pursuit.world DESTINATION ${LIBRARY_OUTPUT_PATH} )

# Copy setup file and substitute correct path into file
SET( SEARCH_REGEX "%BUILD_DIR%" )
file( READ ${CMAKE_CURRENT_SOURCE_DIR}/setup.sh SETUP_FILE_TEXT )
STRING( REGEX REPLACE "${SEARCH_REGEX}" "${LIBRARY_OUTPUT_PATH}" MODIFIED_FILE_TEXT "${SETUP_FILE_TEXT}" )
FILE( WRITE ${LIBRARY_OUTPUT_PATH}/setup.sh "${MODIFIED_FILE_TEXT}" )

