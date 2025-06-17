#!/bin/bash
# filepath: /home/pi/UnitreeRATP/run_all.sh

set -e

# Paths to each program
LIDAR_DIR="/home/pi/UnitreeRATP/src/lidar"
ARUCO_DIR="/home/pi/UnitreeRATP/src/aruco"
MOVEMENT_DIR="/home/pi/UnitreeRATP/src/movement"

# Executable names (from CMakeLists.txt)
LIDAR_EXE="lidar_mqtt"
ARUCO_EXE="unitree_opencv"
MOVEMENT_EXE="droneMovement"

# Kill existing processes if running
for PROC in "$LIDAR_EXE" "$ARUCO_EXE" "$MOVEMENT_EXE"; do
    pkill -f "$PROC" 2>/dev/null || true
done

# Clean, build, and launch LIDAR in new terminal
cd "$LIDAR_DIR"
rm -rf build
mkdir build && cd build
cmake ..
make
x-terminal-emulator -e "./$LIDAR_EXE --channel --serial /dev/ttyUSB0 115200" &
LIDAR_TERM_PID=$!
echo "Started $LIDAR_EXE in new terminal (PID $LIDAR_TERM_PID)"

# Clean, build, and launch ARUCO in new terminal
cd "$ARUCO_DIR"
rm -rf build
mkdir build && cd build
cmake ..
make
x-terminal-emulator -e "./$ARUCO_EXE" &
ARUCO_TERM_PID=$!
echo "Started $ARUCO_EXE in new terminal (PID $ARUCO_TERM_PID)"

# Clean, build, and launch MOVEMENT in new terminal
cd "$MOVEMENT_DIR"
rm -rf build
mkdir build && cd build
cmake ..
make
x-terminal-emulator -e "./$MOVEMENT_EXE" &
MOVEMENT_TERM_PID=$!
echo "Started $MOVEMENT_EXE in new terminal (PID $MOVEMENT_TERM_PID)"

echo ""
echo "All programs started in separate terminals."
echo "Press ENTER to close all terminals and stop the programs..."
read

kill $LIDAR_TERM_PID $ARUCO_TERM_PID $MOVEMENT_TERM_PID 2>/dev/null

# Kill this script's own process
echo "All terminals closed and programs killed."
kill $$