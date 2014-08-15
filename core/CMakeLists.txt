cmake_minimum_required(VERSION 2.8.7)

project( tcs_core )

set( TCS_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/tcs.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/types.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/os.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/log.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/cpu.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/time.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/timer.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/notification.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/message.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/thread.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/thread_heap.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/scheduler.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/timesink.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/processor.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/osthread.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/dynamics_data.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/dynamics_plugin.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tcs/dynamics.h
)

set( TCS_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/os.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/log.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cpu.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/time.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/timer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/message.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/thread_heap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/scheduler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/timesink.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/processor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/osthread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics.cpp
)

set( TCS_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${MOBY_INCLUDE_DIR}
  ${BOOST_INCLUDE_DIR}
)

set( TCS_LIBRARIES
  ${RT_LIBRARIES}
  ${DL_LIBRARIES}
  ${PTHREAD_LIBRARIES}
  ${BOOST_LIBRARIES}
)

include_directories( ${TCS_INCLUDE_DIRS} )

# build the library
add_library( tcs SHARED ${TCS_HEADERS} ${TCS_SOURCES} )
target_link_libraries( tcs LINK_PRIVATE ${TCS_LIBARIES})

# create the include and library variables for future linking
set( TCS_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
set( TCS_LIBRARY
  ${LIBRARY_OUTPUT_PATH}/libtcs.so
)

# build any additional subdirectories
add_subdirectory( test )

# set the variables in PARENT_SCOPE as well
set( TCS_INCLUDE_DIR ${TCS_INCLUDE_DIR} PARENT_SCOPE )
set( TCS_LIBRARY ${TCS_LIBRARY} PARENT_SCOPE )
set( TCS_LIBRARIES ${TCS_LIBRARIES} PARENT_SCOPE )

