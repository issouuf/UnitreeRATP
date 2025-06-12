cmake_minimum_required(VERSION 3.10)
project(LidarMQTTApp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Inclure les headers RPLIDAR
include_directories(${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/include)
include_directories(${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src)
include_directories(${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src/arch/linux)
include_directories(${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src/dataunpacker)
include_directories(${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src/hal)

# Inclure les headers MQTT Paho
include_directories(/usr/include/paho-mqttpp3)

# Add the RPLIDAR SDK sources
file(GLOB RPLIDAR_SDK_SRC
    ${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src/*.cpp
    ${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src/dataunpacker/*.cpp
    ${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src/dataunpacker/unpacker/*.cpp   # <-- FIXED: correct directory
    ${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src/arch/linux/*.cpp
    ${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src/hal/*.cpp
)

add_executable(lidar_mqtt main.cpp ${RPLIDAR_SDK_SRC})

target_link_libraries(lidar_mqtt
    paho-mqttpp3
    paho-mqtt3as
    pthread
)