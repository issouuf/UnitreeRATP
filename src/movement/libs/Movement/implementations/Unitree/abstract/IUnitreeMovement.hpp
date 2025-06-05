#include "../../../abstract/IMovement.hpp"
#include "../utilities/enums/UnitreeProtocols.hpp"

class IUnitreeMovement : public IMovement {
    public:
    explicit IUnitreeMovement(UnitreeProtocols protocol);
    virtual ~IUnitreeMovement() = default;    

    virtual void SetIPAdress(const std::string& ip) = 0;
    virtual void SetPort(u_int16_t port) = 0;

    protected:
    UnitreeProtocols Protocol;

    //@brief IPAdress of the device
    //@note This can be changed during instantiation or via a setter method
    std::string IPAdress;

    //@brief Port of the device
    //@note This can be changed during instantiation or via a setter method
    u_int16_t Port;
};