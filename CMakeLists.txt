cmake_minimum_required(VERSION 2.8)
project( vision_navigation )
find_package( OpenCV 4 REQUIRED )
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
set(LIB_CPP_FILES 
	src/visualSystem.cpp
	src/visualizationOnImage.cpp
	src/fiducialDetector.cpp)	


add_library(vis STATIC ${LIB_CPP_FILES})

add_executable(fiducial_detector_example examples/fiducial_detector_example.cpp)

add_executable(get_image_from_source_example examples/get_image_from_source_example.cpp)


target_link_libraries(fiducial_detector_example vis ${OpenCV_LIBS} yaml-cpp -lopencv_aruco)
target_link_libraries(get_image_from_source_example vis ${OpenCV_LIBS} yaml-cpp)


