#include <string>

class IMovement {
    public :
    virtual ~IMovement();

    bool IsConnected = false;
    bool IsAllowedToMove = false;
    bool IsMoving = false;
    bool IsStopped = true;
    bool IsDisconnectRequested = false;

    std::string Model;
    std::string Manufacturer;

    virtual bool Connect() = 0;
    virtual bool Disconnect() = 0;

    virtual void MoveXYZ(float xMillimeters, float yMillimeters, float angle) = 0;
    
    virtual void MoveX(float millimeters) = 0;
    virtual void MoveY(float millimeters) = 0;
    virtual void MoveZ(float millimeters) = 0;

    virtual void MoveLeftBy(float millimeters) = 0;
    virtual void MoveRightBy(float millimeters) = 0;
    virtual void MoveForwardBy(float millimeters) = 0;
    virtual void MoveBackwardsBy(float millimeters) = 0;

    virtual void Start() = 0;
    virtual void Stop() = 0;

    virtual float GetX() = 0;
    virtual float GetY() = 0;
    virtual float GetZ() = 0;
};