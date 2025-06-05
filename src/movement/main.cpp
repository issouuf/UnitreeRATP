#include <iostream>
#include <string>

#include <mqtt/async_client.h>

#include  "libs/Movement/implementations/Unitree/dogs/Unitree Go 1/UnitreeGo1.hpp"

using namespace std::chrono;

void handleMessageCallBack (mqtt::const_message_ptr msg, UnitreeGo1& robot);

int main(int argc, char *argv[])
{
    std::cout << "Entry !" << std::endl;
    std::cout << "[OK] Starting IPC MQTT Client initialization !" << std::endl;
    mqtt::async_client ipcClient("mqtt://localhost:1883", "RATPmovementClient");
    ipcClient.start_consuming();

    std::cout << "[OK] Connecting the IPC MQTT Client to the local broker !" << std::endl;
    mqtt::token_ptr waitTok = ipcClient.connect();
    waitTok->wait();
    auto ipcClientConnectResponse = waitTok->get_connect_response();

    if (!ipcClientConnectResponse.is_session_present()) {
        std::cout << "[OK] Subscribing to the ordre/commande topic" << std::endl;
        ipcClient.subscribe("ordre/commande", 0)->wait();
    }
    else std::cout << "[OK] The session was remembered, no need to resubscribe";

    std::cout << "[OK] Initialization is done the IPC MQTT client, starting robot initialization !" << std::endl;

    UnitreeGo1 robot = UnitreeGo1();
    try
    {
        if (robot.Connect()) {
            robot.Start();
            ipcClient.set_message_callback([&robot](mqtt::const_message_ptr msg) {handleMessageCallBack(msg, robot);});
        }
        while(true);
    }
    catch (const mqtt::exception &exc)
    {
        ipcClient.disconnect();
        robot.Stop();
        robot.Disconnect();
        std::cerr << exc.what() << std::endl;
        return 1;
    }

    return 0;
}

void handleMessageCallBack (mqtt::const_message_ptr msg, UnitreeGo1& robot) {
    msg->get_topic();
    if (msg->get_topic() == "ordre/commande") {
        std::string commande = msg->to_string();
        std::cout << "[OK] Received command: " << commande << std::endl;

        size_t pos = commande.find('$');
        if (pos != std::string::npos) {
            std::cout << "Substringing command : " << commande << std::endl;
            std::string order = commande.substr(0, pos - 1); // -1 to remove the space before $
            float value = std::stof(commande.substr(pos + 1));
            std::cout << "Command : " << order << " Value : " << value << std::endl;

            if (order == "avancer") {
                robot.MoveY(value);
                std::cout << "Moving Forward by " << value << " mm" << std::endl;
            }
            else if (order == "reculer") {
                robot.MoveX(value);
                std::cout << "Moving Backwards by " << value << " mm" << std::endl;
            }
            else if (order == "gauche" || order == "droite") {
                robot.MoveZ(value);
                std::cout << "Turning by " << value << " °" << std::endl;
            }
        }
    }
    return;
}
