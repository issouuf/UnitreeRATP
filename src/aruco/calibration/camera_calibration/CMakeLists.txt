cmake_minimum_required(VERSION 3.16)

project(camera_calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(camera_calibration main.cpp)


find_package(OpenCV REQUIRED)


# Lier les bibliothèques
target_link_libraries(camera_calibration
    ${OpenCV_LIBS}
)

include(GNUInstallDirs)
install(TARGETS camera_calibration
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
