#include "UnitreeGo1.hpp"

UnitreeGo1::UnitreeGo1(std::string ipAdress, UnitreeProtocols protocol, u_int16_t port)
    : IUnitreeMovement(protocol)
{
    IPAdress = ipAdress;
    Port = port;
    Model = "Unitree Go 1";

    client = new mqtt::async_client((std::ostringstream() << "mqtt://" << IPAdress << ":" << Port).str(), CLIENT_ID);
}

UnitreeGo1::~UnitreeGo1()
{
    IUnitreeMovement::~IUnitreeMovement();
    IMovement::~IMovement();

    if (headColorThread.joinable())
        headColorThread.join();

    if (client != nullptr)
    {
        if (IsConnected)
        {
            if (IsMoving || !IsStopped)
            {
                Stop();
            }
            Disconnect();
        }

        delete client;
        client = nullptr;
    }
}

bool UnitreeGo1::Connect()
{
    // Implement connection logic specific to Unitree Go 1
    try
    {
        std::cout << "[INFO] Trying to connect to " << IPAdress << " on port " << Port << "." << std::endl;
        mqtt::token_ptr waitToken = client->connect(CONNECTION_OPTIONS);
        waitToken->wait();
    }
    catch (const mqtt::exception &e)
    {
        std::cout << "[ERROR] Connection failed due to an exception: " << e.what() << std::endl;
        return false;
    }

    IsConnected = client->is_connected();
    if (IsConnected) {
        std::cout << "[OK] Connected to the drone !" << std::endl;
    }
    else {
        std::cout << "[INFO] COuld not connect to the drone !" << std::endl;
    }

    IsDisconnectRequested = false;
    return IsConnected;
}

void UnitreeGo1::Start()
{
    Topics::controller::Publishers::action::cbPublishMessage(*client, Topics::controller::Publishers::action::Payloads::Walk);
    IsStopped = false;

    xPosition = 0.0f;
    yPosition = 0.0f;
    zPosition = 0.0f;

    IsAllowedToMove = true;

    headColorThread = std::thread(&UnitreeGo1::changeHeadColorThread, this, 1000);
    headColorThread.detach();
}

void UnitreeGo1::Stop()
{
    xPosition = 0.0f;
    yPosition = 0.0f;
    zPosition = 0.0f;
    Topics::controller::Publishers::stick::cbPublishMessage(*client, 0.0f, 0.0f, 0.0f, 0.0f);
    IsMoving = false;
    IsStopped = true;
    IsAllowedToMove = true;
}

bool UnitreeGo1::Disconnect()
{
    // Implement disconnection logic specific to Unitree Go 1
    try
    {
        Stop();
        mqtt::token_ptr waitToken = client->disconnect();
        waitToken->wait();
    }
    catch (const mqtt::exception &e)
    {
        std::cerr << "Disconnection failed: " << e.what() << std::endl;
        return false;
    }
    IsDisconnectRequested = true;
    IsConnected = false; // Simulate successful disconnection
    return !IsConnected;
}

void UnitreeGo1::MoveXYZ(float xMillimeters, float yMillimeters, float angle)
{
    xPosition = scaleMillimetersToNormals(xMillimeters);
    yPosition = scaleMillimetersToNormals(yMillimeters);
    zPosition = scaleAngleToNormals(angle);

    if (!moveThread.joinable())
    {
        moveThread = std::thread(&UnitreeGo1::moveUntilMarkerThread, this);
        moveThread.detach();
    }
}
void UnitreeGo1::MoveX(float millimeters)
{
    xPosition = scaleMillimetersToNormals(millimeters);
    if (!moveThread.joinable())
    {
        moveThread = std::thread(&UnitreeGo1::moveUntilMarkerThread, this);
        moveThread.detach();
    }
}
void UnitreeGo1::MoveY(float millimeters)
{
    yPosition = scaleMillimetersToNormals(millimeters);
    if (!moveThread.joinable())
    {
        moveThread = std::thread(&UnitreeGo1::moveUntilMarkerThread, this);
        moveThread.detach();
    }
}
void UnitreeGo1::MoveZ(float millimeters)
{
    zPosition = scaleAngleToNormals(millimeters);
    if (!moveThread.joinable())
    {
        moveThread = std::thread(&UnitreeGo1::moveUntilMarkerThread, this);
        moveThread.detach();
    }
}
void UnitreeGo1::MoveLeftBy(float millimeters)
{
    // Implement left movement logic for Unitree Go 1
    IsMoving = true; // Simulate movement
}
void UnitreeGo1::MoveRightBy(float millimeters)
{
    // Implement right movement logic for Unitree Go 1
    IsMoving = true; // Simulate movement
}
void UnitreeGo1::MoveForwardBy(float millimeters)
{
    // Implement forward movement logic for Unitree Go 1
    IsMoving = true; // Simulate movement
}
void UnitreeGo1::MoveBackwardsBy(float millimeters)
{
    // Implement backwards movement logic for Unitree Go 1
    IsMoving = true; // Simulate movement
}

float UnitreeGo1::GetX()
{
    // Implement logic to get X position for Unitree Go 1
    return xPosition; // Simulate X position
}
float UnitreeGo1::GetY()
{
    // Implement logic to get Y position for Unitree Go 1
    return yPosition; // Simulate Y position
}
float UnitreeGo1::GetZ()
{
    // Implement logic to get Z position for Unitree Go 1
    return zPosition; // Simulate Z position
}

// Implement IMovement.hpp

void UnitreeGo1::SetIPAdress(const std::string &ip)
{
    IPAdress = ip;
    // if (client != nullptr) {
    //     client->set_server(ip, Port);
    // }
}
void UnitreeGo1::SetPort(u_int16_t port)
{
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
        try
        {
            if (client != nullptr && client->is_connected())
            {
                Topics::face_light::Publishers::color::cbPublishMessage(*client, Topics::face_light::Publishers::color::BlueColor);
                std::this_thread::sleep_for(std::chrono::milliseconds(sleepTimeMillis));
                Topics::face_light::Publishers::color::cbPublishMessage(*client, Topics::face_light::Publishers::color::RedColor);
                std::this_thread::sleep_for(std::chrono::milliseconds(sleepTimeMillis));
            }
        }
        catch (...)
        {
        }
    }
}

void UnitreeGo1::moveUntilMarkerThread()
{
    std::cout << "test" << std::endl;
    while (!IsMarkerReached && IsAllowedToMove && IsConnected && !IsDisconnectRequested)
    {
        IsMoving = true;

        std::cout << "Moving to position: X=" << xPosition << ", Y=" << yPosition << ", Z=" << zPosition << std::endl;
        mqtt::delivery_token_ptr rToken = Topics::controller::Publishers::stick::cbPublishMessage(*client, xPosition, zPosition, 0.0f, yPosition);
        std::cout << "Awaiting response";
        if (rToken != nullptr)
        {
            rToken->wait();
            if (rToken->get_return_code() == mqtt::SUCCESS)
            {
                std::cout << "Movement command sent successfully." << std::endl;
            }
            else
            {
                std::cerr << "Failed to send movement command: " << rToken->get_return_code() << std::endl;
            }
        }
    }

    yPosition = 0.0f;
    xPosition = 0.0f;
    zPosition = 0.0f;
    Topics::controller::Publishers::stick::cbPublishMessage(*client, xPosition, zPosition, 0.0f, yPosition);
    IsMoving = false;
}

// Helper Methodsp
float UnitreeGo1::scaleMillimetersToNormals(float millimeters)
{
    return millimeters / 1000.0f; // Convert millimeters to meters
}

float UnitreeGo1::scaleAngleToNormals(float angle)
{
    return angle / 180.0;
}


