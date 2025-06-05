cmake_minimum_required(VERSION 3.10)
project(LidarMQTTApp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Inclure les headers RPLIDAR
include_directories(${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/include)

# Inclure les headers MQTT Paho
include_directories(/usr/include/paho-mqttpp3)

# Ajouter l'exécutable
add_executable(lidar_mqtt main.cpp)

# Lier les bibliothèques nécessaires
target_link_libraries(lidar_mqtt
    paho-mqttpp3
    paho-mqtt3as
    pthread
)