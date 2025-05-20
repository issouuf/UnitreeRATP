#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <mqtt/async_client.h>

using namespace std;
using namespace std::chrono;

const string DFLT_SERVER_URI{"mqtt://192.168.12.1:1883"};
const string CLIENT_ID{"UnitreeRATP_stickEmulator"};
const string TOPIC{"controller/stick"};

const auto CONNECT_TIMEOUT = std::chrono::seconds(15);
const auto PUBLISH_TIMEOUT = std::chrono::seconds(10);

const auto CONNECTION_OPTIONS = mqtt::connect_options_builder()
                        .connect_timeout(CONNECT_TIMEOUT)
                        .clean_session()
                        .finalize();

const int QOS = 1;

// Function to publish stick movement
void publish_stick(mqtt::async_client& client, float lx, float rx, float ry, float ly) 
{
    // Create a Float32Array (4 floats)
    float stick_values[4] = {lx, rx, ry, ly};
    mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC, stick_values, sizeof(stick_values));
    pubmsg->set_qos(QOS);
    client.publish(pubmsg)->wait_for(PUBLISH_TIMEOUT);
    cout << "Stick values published: [" << lx << ", " << rx << ", " << ry << ", " << ly << "]" << endl;
}

int main(int argc, char* argv[]) 
{
    const string serverURI = (argc > 1) ? string{argv[1]} : DFLT_SERVER_URI;
    const string clientID  = (argc > 2) ? string{argv[2]} : CLIENT_ID;

    cout << "Initializing for server '" << serverURI << " with client id : " << clientID << endl;
    mqtt::async_client client(serverURI, clientID);

    try 
    {
        cout << "\nConnecting..." << endl;
        mqtt::token_ptr conntok = client.connect(CONNECTION_OPTIONS);
        conntok->wait();
        cout << "Connected !" << endl;

        // Example stick movements
        publish_stick(client, 1.0f, 0.0f, 0.0f, 0.0f); // Move right
        std::this_thread::sleep_for(std::chrono::seconds(1));

        publish_stick(client, -1.0f, 0.0f, 0.0f, 0.0f); // Move left
        std::this_thread::sleep_for(std::chrono::seconds(1));

        publish_stick(client, 0.0f, 1.0f, 0.0f, 0.0f); // Move up
        std::this_thread::sleep_for(std::chrono::seconds(1));

        publish_stick(client, 0.0f, -1.0f, 0.0f, 0.0f); // Move down
        std::this_thread::sleep_for(std::chrono::seconds(1));

        publish_stick(client, 0.0f, 0.0f, 0.0f, 0.0f); // Center

        // Disconnect
        cout << "\nDisconnecting..." << endl;
        client.disconnect()->wait();
        cout << "Disconnected !" << endl;
    }
    catch (const mqtt::exception& exc) 
    {
        client.disconnect()->wait();
        cerr << exc.what() << endl;
        return 1;
    }

    return 0;
}
