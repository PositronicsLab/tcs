cmake_minimum_required(VERSION 2.8.7)

project( tcs )

option( BUILD_EXAMPLES
        "Build examples (Requires Gazebo and GSL)" ON )
 
#SET(CMAKE_CXX_COMPILER clang)
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -Wextra -g")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra -g ")
SET(CMAKE_CXX_COMPILER=g++)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g -std=c++0x")

# make available any custom .cmakes
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
#SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

find_package( RT REQUIRED )
find_package( DL REQUIRED )
find_package( Pthread REQUIRED )
find_package( Boost REQUIRED )

set( TCS_INCLUDE_DIR )
set( TCS_LIBRARY )
set( TCS_LIBRARIES )

add_subdirectory( core )

if( BUILD_EXAMPLES )
  add_subdirectory( examples )
endif( BUILD_EXAMPLES )
