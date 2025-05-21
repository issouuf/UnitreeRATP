#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <mqtt/async_client.h>

using namespace std;
using namespace std::chrono;

const string DFLT_SERVER_URI{"mqtt://192.168.12.1:1883"};
const string CLIENT_ID{"UnitreeRATP_stickEmulator"};

const auto CONNECT_TIMEOUT = std::chrono::seconds(15);
const auto PUBLISH_TIMEOUT = std::chrono::seconds(10);

const auto CONNECTION_OPTIONS = mqtt::connect_options_builder()
                                    .connect_timeout(CONNECT_TIMEOUT)
                                    .clean_session()
                                    .finalize();

struct Topics
{
    struct controller
    {
        struct Publishers
        {
            struct action
            {
                static inline const string Name = "action";
                static inline constexpr const unsigned char QOS = 1;

                struct Payloads
                {
                    static inline const char* Walk = "walk";
                    static inline const char* Run = "run";
                    static inline const char* Stand = "stand";
                };

                void cbPublishMessage(mqtt::async_client &mqttClient, const char* command = Payloads::Walk)
                {
                    auto pubmsg = mqtt::make_message(Name, command, 4);
                    pubmsg->set_qos(QOS);
                    mqttClient.publish(pubmsg)->wait_for(PUBLISH_TIMEOUT);
                }
            };

            struct stick
            {
                static inline const string Name = "stick";
                static inline constexpr const unsigned char QOS = 0;
                void cbPublishMessage(mqtt::async_client &mqttClient, const float deplacementVertical = 0.0f, const float rotation = 0.0f, const float ignore = 0.0f, const float deplacementHorizontal = 0.0f)
                {
                    float stick_values[4] = {deplacementVertical, rotation, ignore, deplacementHorizontal};
                    for (unsigned char index = 0; index < 4; index++) 
                    {
                        if (stick_values[index] > 1.0f)
                            stick_values[index] = 1.0f;
                        else if (stick_values[index] < -1.0f)
                            stick_values[index] = -1.0f;
                    }
                        
                    auto pubmsg = mqtt::make_message(Name, stick_values, 4);
                    pubmsg->set_qos(QOS);
                    mqttClient.publish(pubmsg)->wait_for(PUBLISH_TIMEOUT);
                }
            };
        };
    };
};

/* 1er param : Mouvement Gauche ou Droite
    -1 = Gauche à fond (~1m)
     1 = Droite à fond (~1m)

# 2eme param : Rotation Gauche ou Droite
# -1 = Rotation vers la gauche (~180°)
#  1 = Rotation vers la droite (~180°)

# 3eme param : ? Pitche/Yaw (Ne fais rien)

# 4eme param : Mouvement Arrière ou Avant
# -1 = Arrière à fond (~1m)
#  1 = Avant à fond (~1m) */


int main(int argc, char *argv[])
{

    const string serverURI = (argc > 1) ? string{argv[1]} : DFLT_SERVER_URI;
    const string clientID = (argc > 2) ? string{argv[2]} : CLIENT_ID;

    cout << "Initializing for server '" << serverURI << " with client id : " << clientID << endl;
    mqtt::async_client client(serverURI, clientID);

    try
    {
        cout << "\nConnecting..." << endl;
        mqtt::token_ptr conntok = client.connect(CONNECTION_OPTIONS);
        conntok->wait();
        cout << "Connected !" << endl;

        static const string TOPIC_ACTION = "controller/action";
        mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC_ACTION, "walk", sizeof("walk"));


        // Disconnect
        cout << "\nDisconnecting..." << endl;
        client.disconnect()->wait();
        cout << "Disconnected !" << endl;
    }
    catch (const mqtt::exception &exc)
    {
        client.disconnect()->wait();
        cerr << exc.what() << endl;
        return 1;
    }

    return 0;
}
