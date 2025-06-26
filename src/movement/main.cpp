#include <iostream>
#include <string>
#include <cmath>

#include <mqtt/async_client.h>

#include "libs/Movement/implementations/Unitree/dogs/Unitree Go 1/UnitreeGo1.hpp"

using namespace std::chrono;

void handleArucoMovementCB(mqtt::const_message_ptr msg, UnitreeGo1 &robot);
void handleLidarMovementCB(mqtt::const_message_ptr msg, UnitreeGo1 &robot);

int main(int argc, char *argv[])
{
    std::cout << "Entry !" << std::endl;
    std::cout << "[OK] Starting IPC MQTT Client initialization !" << std::endl;
    mqtt::async_client arucoIPCClient("mqtt://localhost:1883", "RATPArucoListener");
    mqtt::async_client lidarIPCClient("mqtt://localhost:1883", "RATPLidarListener");
    arucoIPCClient.start_consuming();
    lidarIPCClient.start_consuming();

    std::cout << "[OK] Connecting the IPC MQTT Client to the local broker !" << std::endl;
    mqtt::token_ptr waitTok = arucoIPCClient.connect();
    waitTok->wait();
    auto ipcClientConnectResponse = waitTok->get_connect_response();

    if (!ipcClientConnectResponse.is_session_present())
    {
        std::cout << "[OK] Subscribing to the ordre/commande topic" << std::endl;
        arucoIPCClient.subscribe("ordre/commande", 1)->wait();
    }
    else
        std::cout << "[OK] The aruco listener session was remembered, no need to resubscribe";

    waitTok = lidarIPCClient.connect();
    waitTok->wait();
    ipcClientConnectResponse = waitTok->get_connect_response();

    if (!ipcClientConnectResponse.is_session_present())
    {
        std::cout << "[OK] Subscribing to the lidar/commande topic" << std::endl;
        lidarIPCClient.subscribe("lidar/commande", 1)->wait();
    }
    else
        std::cout << "[OK] The lidar listener session was remembered, no need to resubscribe";

    std::cout << "[OK] Initialization is done the IPC MQTT client, starting robot initialization !" << std::endl;

    UnitreeGo1 robot = UnitreeGo1();
    try
    {
        if (robot.Connect())
        {
            robot.Start();
            arucoIPCClient.set_message_callback([&robot](mqtt::const_message_ptr msg)
                                                { handleArucoMovementCB(msg, robot); });
            lidarIPCClient.set_message_callback([&robot](mqtt::const_message_ptr msg)
                                                { handleLidarMovementCB(msg, robot); });
        }
        while (true);
    }
    catch (const mqtt::exception &exc)
    {
        robot.Stop();

        arucoIPCClient.disconnect();
        lidarIPCClient.disconnect();

        robot.Disconnect();
        std::cerr << exc.what() << std::endl;
        return 1;
    }

    return 0;
}

void handleArucoMovementCB(mqtt::const_message_ptr msg, UnitreeGo1 &robot)
{
    if (msg->get_topic() == "ordre/commande")
    {
        if (robot.IsConnected && robot.IsAllowedToMove)
        {
            std::string commande = msg->to_string();
            std::cout << "[OK] Received command: " << commande << std::endl;

            if (commande == "STOP")
            {
                if (!robot.IsMarkerReached) robot.IsMarkerReached = true;
                if (robot.GetZ() != 0.0f) robot.MoveZ(0.0f);
                if (robot.GetX() != 0.0f) robot.MoveX(0.0f);
                if (robot.GetY() != 0.0f) robot.MoveY(0.0f);
                std::cout << "[OK] Stopping the robot." << std::endl;
                return;
            }

            size_t pos = commande.find('$');
            if (pos != std::string::npos)
            {
                robot.IsMarkerReached = false;
                std::cout << "Substringing command : " << commande << std::endl;
                std::string order = commande.substr(0, pos - 1); // -1 to remove the space before $
                float value = std::stof(commande.substr(pos + 1));
                std::cout << "Command : " << order << " Value : " << value << std::endl;

                if (order == "avancer")
                {
                    robot.MoveY(value);
                    std::cout << "Moving Forward by " << value << " mm" << std::endl;
                }
                else if (order == "reculer")
                {
                    robot.MoveX(value);
                    std::cout << "Moving Backwards by " << value << " mm" << std::endl;
                }
                else if (order == "gauche" || order == "droite")
                {
                    robot.MoveZ(value);
                    std::cout << "Turning by " << value << " °" << std::endl;
                }
            }
        }
        else
            std::cout << "[ERROR] The robot is not allowed to move. Please call Robot.Start() to allow movement." << std::endl;
    }
}

void handleLidarMovementCB(mqtt::const_message_ptr msg, UnitreeGo1 &robot)
{
    if (msg->get_topic() == "lidar/commande")
    {
        if (robot.IsConnected && robot.IsAllowedToMove)
        {
            std::string commande = msg->to_string();
            std::cout << "[OK] Received command: " << commande << std::endl;

            // Parse distance and angle from the message
            size_t pos_dist = commande.find("distance $");
            size_t pos_angle = commande.find("angle $");

            if (pos_dist != std::string::npos && pos_angle != std::string::npos)
            {
                // Extract distance value
                size_t dist_start = pos_dist + std::string("distance $").length();
                size_t dist_end = commande.find(' ', dist_start);
                std::string dist_str = commande.substr(dist_start, dist_end - dist_start);
                float distance = std::stof(dist_str);

                // Extract angle value
                size_t angle_start = pos_angle + std::string("angle $").length();
                std::string angle_str = commande.substr(angle_start);
                float angle = std::stof(angle_str);

                std::cout << "Distance: " << distance << " mm, Angle: " << angle << "°" << std::endl;

                // Avoid obstacle: if obstacle is on the right, turn left; if on the left, turn right
                float turn_angle = 0.0f;
                if (angle >= 0 && angle <= 120) {
                    // Obstacle on the right, turn left (negative angle)
                    turn_angle = -45.0f;
                    std::cout << "Obstacle on the right, turning left by " << turn_angle << "°" << std::endl;
                } else if (angle >= 240 && angle <= 360) {
                    // Obstacle on the left, turn right (positive angle)
                    turn_angle = 45.0f;
                    std::cout << "Obstacle on the left, turning right by " << turn_angle << "°" << std::endl;
                } else {
                    // Obstacle in front or unknown, stop or move back
                    std::cout << "Obstacle in front or unknown, stopping." << std::endl;
                    robot.Stop();
                    return;
                }

                // Clamp turn_angle to [-180, 180]
                if (turn_angle > 180.0f) turn_angle = 180.0f;
                if (turn_angle < -180.0f) turn_angle = -180.0f;

                robot.MoveZ(turn_angle);
            }
            else
            {
                std::cout << "[ERROR] Could not parse distance and angle from message: " << commande << std::endl;
            }
        }
        else
            std::cout << "[ERROR] The robot is not allowed to move. Please call Robot.Start() to allow movement." << std::endl;
    }
}
