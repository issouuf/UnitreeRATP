cmake_minimum_required(VERSION 3.16)

project(mqttLidar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PahoMqttCpp REQUIRED)


add_executable(mqttLidar main.cpp)


# Lier les bibliothèques
target_link_libraries(mqttLidar
    PahoMqttCpp::paho-mqttpp3
)


include(GNUInstallDirs)
install(TARGETS mqttLidar
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
