target_sources(fiducial_navigation
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/ArucoCustomTools.cpp
        ${CMAKE_CURRENT_LIST_DIR}/CVMathOperations.cpp
        ${CMAKE_CURRENT_LIST_DIR}/VisualizationOnImage.cpp
        )

target_include_directories(fiducial_navigation
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )

message(STATUS "configure cv tools")