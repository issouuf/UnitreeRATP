#include <format>
#include <thread>

#include "../../abstract/IUnitreeMovement.hpp"
#include "utilities/MQTT/Topics.hpp"

#include <mqtt/async_client.h>


class UnitreeGo1 : public IUnitreeMovement {
    public:
    UnitreeGo1(std::string ipAdress = FACTORY_UNITREE_GO_1_DEFAULT_IP_ADDRESS, UnitreeProtocols protocol = UnitreeProtocols::MQTT, u_int16_t port = FACTORY_UNITREE_GO_1_DEFAULT_MQTT_PORT);
    ~UnitreeGo1();

    // Override methods from IMovement
    bool Connect() override;
    bool Disconnect() override;
    void MoveXYZ(float xMillimeters, float yMillimeters, float angle) override;
    void MoveX(float millimeters) override;
    void MoveY(float millimeters) override;
    void MoveZ(float millimeters) override;
    void MoveLeftBy(float millimeters) override;
    void MoveRightBy(float millimeters) override;
    void MoveForwardBy(float millimeters) override;
    void MoveBackwardsBy(float millimeters) override;
    void Start() override;
    void Stop() override;
    float GetX() override;
    float GetY() override;
    float GetZ() override;

    void SetIPAdress(const std::string& ip) override;
    void SetPort(u_int16_t port) override;

    private:
    static constexpr const char* FACTORY_UNITREE_GO_1_DEFAULT_IP_ADDRESS = "192.168.12.1";
    static constexpr u_int16_t FACTORY_UNITREE_GO_1_DEFAULT_MQTT_PORT = 1883;

    std::string CLIENT_ID {"RATP_UnitreeGo1"};

    mqtt::connect_options CONNECTION_OPTIONS = mqtt::connect_options_builder()
        .mqtt_version(MQTTVERSION_DEFAULT)
        .automatic_reconnect()
        .keep_alive_interval(std::chrono::seconds(20))
        .clean_session()
        .finalize();

    mqtt::async_client* client = nullptr;

    float scaleMillimetersToNormals(float millimeters);
    float scaleAngleToNormals(float angle);

    void changeHeadColorThread(int sleepTimeMillis = 1000);
    std::thread headColorThread;
};