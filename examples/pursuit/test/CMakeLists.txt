cmake_minimum_required( VERSION 2.8 )

project( moby_pursuit_experiment_tests )

set( LIBS 
  ${TCS_LIBRARY}
  ${TCS_LIBRARIES}
  ${TCS_PURSUIT_EXPERIMENT_LIBRARIES}
  libboost_thread.so
)

include_directories( ${TCS_INCLUDE_DIR} )

add_executable( test-pursuit-moby-xml ${CMAKE_CURRENT_SOURCE_DIR}/test_moby_xml.cpp ${TCS_PURSUIT_EXPERIMENT_HEADERS} ${TCS_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( test-pursuit-moby-xml ${LIBS} )

add_executable( test-pursuit-pred-controller ${CMAKE_CURRENT_SOURCE_DIR}/test_pred_controller.cpp ${TCS_PURSUIT_EXPERIMENT_HEADERS} ${TCS_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( test-pursuit-pred-controller ${LIBS} )

#add_executable( test-pursuit-pred-planner ${CMAKE_CURRENT_SOURCE_DIR}/test_pred_planner.cpp ${TCS_PURSUIT_EXPERIMENT_HEADERS} ${TCS_PURSUIT_EXPERIMENT_SOURCES} )
#target_link_libraries( test-pursuit-pred-planner ${PURSUIT_LIBRARIES} libboost_thread.so )

add_executable( test-pursuit-prey-command-seed ${CMAKE_CURRENT_SOURCE_DIR}/test_prey_command_seed.cpp ${TCS_PURSUIT_EXPERIMENT_HEADERS} ${TCS_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( test-pursuit-prey-command-seed ${LIBS} )

add_executable( test-pursuit-pred-time-randval ${CMAKE_CURRENT_SOURCE_DIR}/test_pred_time_randval.cpp ${TCS_PURSUIT_EXPERIMENT_HEADERS} ${TCS_PURSUIT_EXPERIMENT_SOURCES} )
target_link_libraries( test-pursuit-pred-time-randval ${LIBS} )

