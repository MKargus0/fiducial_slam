
add_executable(detector_test  ${CMAKE_CURRENT_LIST_DIR}/fiducial_detector_example.cpp)
target_link_libraries(detector_test PUBLIC fiducial_navigation)

add_executable(fid_nav_test  ${CMAKE_CURRENT_LIST_DIR}/fiducial_navigation_example.cpp)
target_link_libraries(fid_nav_test PUBLIC fiducial_navigation)

add_executable(fid_slam_test  ${CMAKE_CURRENT_LIST_DIR}/fiducial_slam_example.cpp)
target_link_libraries(fid_slam_test PUBLIC fiducial_navigation)

add_executable(vis_nav  ${CMAKE_CURRENT_LIST_DIR}/visual_navigation_example.cpp)
target_link_libraries(vis_nav PUBLIC fiducial_navigation)

message(STATUS "build examples")