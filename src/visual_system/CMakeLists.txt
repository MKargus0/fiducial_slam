target_sources(fiducial_navigation
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/VisionSystem.cpp
        ${CMAKE_CURRENT_LIST_DIR}/VisualNavigation.cpp
        )

target_include_directories(fiducial_navigation
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )

message(STATUS "configure vision system module")