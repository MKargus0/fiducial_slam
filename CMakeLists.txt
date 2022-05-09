cmake_minimum_required(VERSION 3.15)
project(fiducial_slam VERSION 0.2.1)

find_package(OpenCV 4 REQUIRED )
find_package(Eigen3  REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(inc)

add_library(fiducial_navigation STATIC "")

target_include_directories(fiducial_navigation
        PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        )

include(src/common_tools/CMakeLists.txt)
include(src/cv_tools/CMakeLists.txt)
include(src/visual_system/CMakeLists.txt)
include(src/fiducial_navigation/CMakeLists.txt)

set(BUILD_EXAMPLES ON)
if(BUILD_EXAMPLES)
    include(examples/CMakeLists.txt)
endif()


target_link_libraries(fiducial_navigation ${OpenCV_LIBS} yaml-cpp)