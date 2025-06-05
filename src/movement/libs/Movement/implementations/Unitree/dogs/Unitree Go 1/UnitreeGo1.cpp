#include "UnitreeGo1.hpp"

UnitreeGo1::UnitreeGo1(std::string ipAdress, UnitreeProtocols protocol, u_int16_t port)
    : IUnitreeMovement(protocol) {
        IPAdress = ipAdress;
        Port = port;
        Model = "Unitree Go 1";

        client = new mqtt::async_client(std::format("mqtt:://{0}:{1}", IPAdress, Port), CLIENT_ID);
}

UnitreeGo1::~UnitreeGo1() {
    IUnitreeMovement::~IUnitreeMovement();
    IMovement::~IMovement();
    if (client != nullptr) {
        if (IsConnected) {
            if (IsMoving || !IsStopped) {
                Stop();
            }
            Disconnect();
        }

        delete client;
        client = nullptr;
    }

}

bool UnitreeGo1::Connect() {
    // Implement connection logic specific to Unitree Go 1
    try {
        mqtt::token_ptr waitToken = client->connect(CONNECTION_OPTIONS);
        waitToken->wait();
    } catch (const mqtt::exception& e) {
        std::cerr << "Connection failed: " << e.what() << std::endl;
        return false;
    }
    IsDisconnectRequested = false;
    IsConnected = true; // Simulate successful connection
    return IsConnected;
}

void UnitreeGo1::Start() {
    Topics::controller::Publishers::action::cbPublishMessage(*client, Topics::controller::Publishers::action::Payloads::Walk);
    IsStopped = false;
    IsAllowedToMove = true;

    headColorThread = std::thread(&UnitreeGo1::changeHeadColorThread, this, 1000);
    if (headColorThread.joinable()) {
        headColorThread.join();
    }
}


void UnitreeGo1::Stop() {
    Topics::controller::Publishers::stick::cbPublishMessage(*client, 0.0f, 0.0f, 0.0f, 0.0f);
    IsMoving = false;
    IsStopped = true;
    IsAllowedToMove = true;
}

bool UnitreeGo1::Disconnect() {
    // Implement disconnection logic specific to Unitree Go 1
    try {
        mqtt::token_ptr waitToken = client->disconnect();
        waitToken->wait();
    } catch (const mqtt::exception& e) {
        std::cerr << "Disconnection failed: " << e.what() << std::endl;
        return false;
    }
    IsDisconnectRequested = true;
    IsConnected = false; // Simulate successful disconnection
    return !IsConnected;
}

void UnitreeGo1::MoveXYZ(float xMillimeters, float yMillimeters, float angle) {
    Topics::controller::Publishers::stick::cbPublishMessage(*client, scaleMillimetersToNormals(yMillimeters), scaleAngleToNormals(angle), 0.0f, scaleMillimetersToNormals(yMillimeters));
}
void UnitreeGo1::MoveX(float millimeters) {
    // Implement X-axis movement logic for Unitree Go 1
    IsMoving = true; // Simulate movement
}
void UnitreeGo1::MoveY(float millimeters) {
    // Implement Y-axis movement logic for Unitree Go 1
    IsMoving = true; // Simulate movement
}
void UnitreeGo1::MoveZ(float millimeters) {
    // Implement Z-axis movement logic for Unitree Go 1
    IsMoving = true; // Simulate movement
}
void UnitreeGo1::MoveLeftBy(float millimeters) {
    // Implement left movement logic for Unitree Go 1
    IsMoving = true; // Simulate movement
}
void UnitreeGo1::MoveRightBy(float millimeters) {
    // Implement right movement logic for Unitree Go 1
    IsMoving = true; // Simulate movement
}
void UnitreeGo1::MoveForwardBy(float millimeters) {
    // Implement forward movement logic for Unitree Go 1
    IsMoving = true; // Simulate movement
}
void UnitreeGo1::MoveBackwardsBy(float millimeters) {
    // Implement backwards movement logic for Unitree Go 1
    IsMoving = true; // Simulate movement
}

float UnitreeGo1::GetX() {
    // Implement logic to get X position for Unitree Go 1
    return 0.0f; // Simulate X position
}
float UnitreeGo1::GetY() {
    // Implement logic to get Y position for Unitree Go 1
    return 0.0f; // Simulate Y position
}
float UnitreeGo1::GetZ() {
    // Implement logic to get Z position for Unitree Go 1
    return 0.0f; // Simulate Z position
}

// Implement IMovement.hpp

void UnitreeGo1::SetIPAdress(const std::string& ip) {
    IPAdress = ip;
    // if (client != nullptr) {
    //     client->set_server(ip, Port);
    // }
}
void UnitreeGo1::SetPort(u_int16_t port) {
    Port = port;
    // if (client != nullptr) {
    //     client->set_server(IPAdress, port);
    // }
}


// Threads

//@brief Changes the head color of the Unitree Go 1 robot in a loop.
void UnitreeGo1::changeHeadColorThread(int sleepTimeMillis) 
{
    while (IsConnected && !IsDisconnectRequested) 
    {
        if (client != nullptr && client->is_connected()) 
        {
            Topics::face_light::Publishers::color::cbPublishMessage(*client, Topics::face_light::Publishers::color::BlueColor);
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTimeMillis));
            Topics::face_light::Publishers::color::cbPublishMessage(*client, Topics::face_light::Publishers::color::RedColor);
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTimeMillis));
        }
    } 
}


// Helper Methodsp
float UnitreeGo1::scaleMillimetersToNormals(float millimeters) {
        return millimeters / 1000.0f; // Convert millimeters to meters
}

float UnitreeGo1::scaleAngleToNormals(float angle) {
    return angle / 180.0;
}


