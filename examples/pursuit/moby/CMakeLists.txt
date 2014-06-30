cmake_minimum_required( VERSION 2.8 )

project( moby_pursuit_experiment )

set( LIBS 
  ${TCS_LIBRARY}
  ${TCS_LIBRARIES}
  ${TCS_PURSUIT_EXPERIMENT_LIBRARIES}
  libboost_thread.so
)

include_directories( ${TCS_INCLUDE_DIR} )

# Moby controllers
add_library( pursuit-moby-prey-controller "MODULE" ${CMAKE_CURRENT_SOURCE_DIR}/moby_prey_controller.cpp ${TCS_PURSUIT_EXPERIMENT_HEADERS} ${TCS_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( pursuit-moby-prey-controller ${LIBS} )

add_library( pursuit-moby-predator-controller "MODULE" ${CMAKE_CURRENT_SOURCE_DIR}/moby_predator_controller.cpp ${TCS_PURSUIT_EXPERIMENT_HEADERS} ${TCS_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( pursuit-moby-predator-controller ${LIBS} )

# Copy data files
#file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/pursuit.xml DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )

