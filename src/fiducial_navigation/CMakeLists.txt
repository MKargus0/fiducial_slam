target_sources(fiducial_navigation
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/CustomMultipleBoard.cpp
        ${CMAKE_CURRENT_LIST_DIR}/FiducialNavigationSystem.cpp
        ${CMAKE_CURRENT_LIST_DIR}/ANavigationSystem.cpp
        ${CMAKE_CURRENT_LIST_DIR}/FiducialDetector.cpp
        )

target_include_directories(fiducial_navigation
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )

message(STATUS "configure fiducial navigation module")