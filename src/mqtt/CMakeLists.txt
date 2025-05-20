cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(mqtt VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the paho submodule
add_subdirectory(paho/build/include)

# Add the executable
add_executable(mqtt main.cpp)

# Link the Paho MQTT C++ library
target_link_libraries(mqtt paho-mqttpp3 paho-mqtt3as)
