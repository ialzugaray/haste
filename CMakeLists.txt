cmake_minimum_required(VERSION 3.0.0)
project(haste VERSION 1.0.0) # Hypotheses-based Asynchronous Tracker of Events

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -O3")

find_package(OpenCV 3.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(gflags REQUIRED)
find_package(glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen)
find_package(Eigen3 3.3.4 EXACT MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")
if(NOT EIGEN3_INCLUDE_DIR MATCHES "^${EIGEN_ROOT}")
  message(WARNING "Found Eigen headers are outside of specified EIGEN_ROOT '${EIGEN_ROOT}' \n Instead: '${EIGEN3_INCLUDE_DIR}'")
endif()

include_directories( include)

add_executable(tracking_app_file src/tracking_app_file.cpp)
target_link_libraries(tracking_app_file ${OpenCV_LIBS} gflags glog)

SET(GUI OFF CACHE BOOL "Builds HASTE GUI playground")

if (GUI)
  message("-DGUI=true (Default). GUI will be compiled.")

  add_subdirectory(thirdparty/quickgui)
  add_executable(tracking_app_gui ${CMAKE_CURRENT_LIST_DIR}/src/tracking_app_gui.cpp)
  target_link_libraries(tracking_app_gui ${OpenCV_LIBS} gflags glog QuickGui)

else ()
  message("-DGUI=false. No GUI will be compiled.")
endif ()




