project(panda_control_vrep)
cmake_minimum_required(VERSION 2.8)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" )
MESSAGE( STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )

find_package(PkgConfig QUIET)
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR} 
  ${RBDL_INCLUDE_DIR}
  vrep_src/include 
  vrep_src/remoteApi
  include
)

add_definitions (-DNON_MATLAB_PARSING)
add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
#add_definitions (-DDO_NOT_USE_SHARED_MEMORY)

set (SOURCES
	src/controller.cpp
	src/main.cpp
    src/vrep_bridge.cpp

	vrep_src/remoteApi/extApi.c
	vrep_src/remoteApi/extApiPlatform.c
	vrep_src/common/shared_memory.c
)

add_executable(panda_control_vrep ${SOURCES})

add_definitions(-DDOF=7)

target_link_libraries(panda_control_vrep
	${RBDL_LIBRARY}
	pthread
	rt
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(CMAKE_BUILD_TYPE RELEASE)

