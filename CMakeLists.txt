cmake_minimum_required(VERSION 2.8)
project( vision_navigation )
set(CMAKE_CXX_STANDARD 17)
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
	src/fiducialDetector.cpp
	src/customBoard.cpp
	src/fuducialNavigationSystem.cpp
	src/cvMathOperations.cpp
	src/visualNavigationSystem.cpp
	src/mathOperations.cpp
	src/visualNavigationInterface.cpp
	src/navigationSystem.cpp
	src/opticalFlowNavigationSystem.cpp)
	# src/sendPoseDataUDP.c)	


add_library(vis STATIC ${LIB_CPP_FILES})

add_executable(fiducial_detector_example examples/fiducial_detector_example.cpp)

add_executable(get_image_from_source_example examples/get_image_from_source_example.cpp)

add_executable(fiducial_navigation_example examples/fiducial_navigation_example.cpp)

add_executable(fiducial_slam_example examples/fiducial_slam_example.cpp)

add_executable(visual_navigation_example examples/visual_navigation_example.cpp)

add_executable(optical_flow_nav_example examples/optical_flow_nav_example.cpp)

target_link_libraries(visual_navigation_example vis ${OpenCV_LIBS} yaml-cpp -lopencv_aruco)
target_link_libraries(fiducial_detector_example vis ${OpenCV_LIBS} yaml-cpp -lopencv_aruco)
target_link_libraries(get_image_from_source_example vis ${OpenCV_LIBS} yaml-cpp)
target_link_libraries(fiducial_navigation_example vis ${OpenCV_LIBS} yaml-cpp -lopencv_aruco)
target_link_libraries(fiducial_slam_example vis ${OpenCV_LIBS} yaml-cpp -lopencv_aruco)
target_link_libraries(optical_flow_nav_example vis ${OpenCV_LIBS} yaml-cpp -lopencv_aruco)

add_executable(test examples/test.cpp)
target_link_libraries(test vis)