cmake_minimum_required(VERSION 3.16)

project(mqttLidar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PahoMqttCpp REQUIRED)
find_package(Curses REQUIRED)

file(GLOB_RECURSE SOURCES
    "libs/Movement/implementations/Unitree/**/*.cpp"
    "libs/Movement/abstract/*.cpp"
    "libs/Movement/implementations/Unitree/abstract/*.cpp"
)

add_executable(mqttLidar ${SOURCES} main.cpp)


# Lier les bibliothèques
target_link_libraries(mqttLidar
    ${CURSES_LIBRARIES}
    PahoMqttCpp::paho-mqttpp3
)


include(GNUInstallDirs)
install(TARGETS mqttLidar
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
