#include <mqtt/async_client.h>

const auto DEFAULT_PUBLISH_TIMEOUT = std::chrono::seconds(10);

struct Topics
{
    struct controller
    {
        struct Publishers
        {
            struct action
            {
                static inline const std::string Name = "controller/action";
                static inline constexpr const unsigned char QOS = 0;

                struct Payloads
                {
                    static inline const mqtt::message_ptr Walk = mqtt::make_message(Name, "walk", 4);
                    static inline const mqtt::message_ptr Run = mqtt::make_message(Name, "run", 3);
                    static inline const mqtt::message_ptr Stand = mqtt::make_message(Name, "stand", 5);
                };

                static void cbPublishMessage(mqtt::async_client &mqttClient, const mqtt::message_ptr payload = Payloads::Walk)
                {
                    try
                    {
                        payload->set_qos(QOS);
                        mqttClient.publish(payload);
                    }
                    catch (...)
                    {
                    }
                }
            };

            struct stick
            {
                static inline const std::string Name = "controller/stick";
                static inline constexpr const unsigned char QOS = 1;

                static mqtt::delivery_token_ptr cbPublishMessage(mqtt::async_client &mqttClient, const float deplacementVertical = 0.0f, const float rotation = 0.0f, const float ignore = 0.0f, const float deplacementHorizontal = 0.0f)
                {
                    float stick_values[4] = {deplacementVertical, rotation, ignore, deplacementHorizontal};
                    for (static unsigned char index = 0; index < 4; index++)
                    {
                        if (stick_values[index] > 1.0f)
                            stick_values[index] = 1.0f;
                        else if (stick_values[index] < -1.0f)
                            stick_values[index] = -1.0f;
                    }

                    try
                    {
                        auto pubmsg = mqtt::make_message(Name, stick_values, 4 * 4);
                        pubmsg->set_qos(QOS);
                        return mqttClient.publish(pubmsg);
                    }
                    catch (...)
                    {
                        return nullptr;
                    }
                }
            };
        };
    };
    struct face_light
    {
        struct Publishers
        {
            struct color
            {
                static inline const std::string Name = "face_light/color";
                static inline constexpr const unsigned char QOS = 0;

                static inline constexpr const unsigned char BlueColor[3] = {0x00, 0x00, 0xFF};
                static inline constexpr const unsigned char RedColor[3] = {0xFF, 0x00, 0x00};

                static void cbPublishMessage(mqtt::async_client &mqttClient, const unsigned char color[3] = BlueColor)
                {
                    auto pubmsg = mqtt::make_message(Name, color, 3);
                    pubmsg->set_qos(QOS);
                    mqttClient.publish(pubmsg);
                }
            };
        };
    };
};