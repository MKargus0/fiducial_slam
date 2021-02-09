cmake_minimum_required(VERSION 2.8)
project( vision_navigation )
find_package( OpenCV 3 REQUIRED )
find_package (Eigen3  REQUIRED)
#link_directories("/usr/lib/" "/usr/include")
include_directories(${EIGEN3_INCLUDE_DIR})
#INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )
include_directories(include ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )
#include_directories(/home/argus/opencv_tutorials/include)
#set(HEADERS include/vision_systems.hpp)
message("Building")
if(DVISUALIZATION)
  message("Building visual_navigation with visualization tools")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -VISUALIZATION")
  #add_compile_definitions(VISUALIZATION)
  #add_definitions(VISUALIZATION)
endif(DVISUALIZATION)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVISUALIZATION")
#set(SOURCE_LIB vision_systems.cpp)	
add_library(vis STATIC src/vision_systems.cpp src/aruco_custom_tools.cpp)

add_executable(video_cap src/video_cap.cpp)

target_link_libraries(video_cap vis ${OpenCV_LIBS} -lopencv_aruco yaml-cpp)



