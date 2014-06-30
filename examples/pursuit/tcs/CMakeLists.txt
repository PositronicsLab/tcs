cmake_minimum_required( VERSION 2.8 )

project( tcs_pursuit_experiment )

#message( ${TCS_INCLUDE_DIR} )

set( LIBS 
  ${TCS_LIBRARY}
  ${TCS_LIBRARIES}
  ${TCS_PURSUIT_EXPERIMENT_LIBRARIES} 
  libboost_thread.so
) 

include_directories( ${TCS_INCLUDE_DIR} )

add_executable( pursuit-tcs-coordinator ${CMAKE_CURRENT_SOURCE_DIR}/coordinator.cpp ${TCS_PURSUIT_EXPERIMENT_HEADERS} ${TCS_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( pursuit-tcs-coordinator ${LIBS} )

add_library( pursuit-tcs-moby-plugin "MODULE" ${CMAKE_CURRENT_SOURCE_DIR}/moby_plugin.cpp ${TCS_PURSUIT_EXPERIMENT_HEADERS} ${TCS_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( pursuit-tcs-moby-plugin ${LIBS} )

add_executable( pursuit-tcs-prey-controller ${CMAKE_CURRENT_SOURCE_DIR}/prey_controller.cpp ${TCS_PURSUIT_EXPERIMENT_HEADERS} ${TCS_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( pursuit-tcs-prey-controller ${LIBS} )

add_executable( pursuit-tcs-pred-controller ${CMAKE_CURRENT_SOURCE_DIR}/pred_controller.cpp ${TCS_PURSUIT_EXPERIMENT_HEADERS} ${TCS_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( pursuit-tcs-pred-controller ${LIBS} )

add_executable( pursuit-tcs-pred-planner ${CMAKE_CURRENT_SOURCE_DIR}/pred_planner.cpp ${TCS_PURSUIT_EXPERIMENT_HEADERS} ${TCS_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( pursuit-tcs-pred-planner ${LIBS} )

