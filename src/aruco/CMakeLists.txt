cmake_minimum_required(VERSION 3.16)

project(unitree_opencv LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(PahoMqttCpp REQUIRED)


add_executable(unitree_opencv main.cpp)


# Lier les bibliothèques
target_link_libraries(unitree_opencv
    ${OpenCV_LIBS}
)
target_link_libraries(unitree_opencv PahoMqttCpp::paho-mqttpp3)



include(GNUInstallDirs)
install(TARGETS unitree_opencv
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
