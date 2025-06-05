#include "IUnitreeMovement.hpp"
IUnitreeMovement::IUnitreeMovement(UnitreeProtocols protocol)
    : Protocol(protocol) {
        Manufacturer = "Unitree Robotics";
        Model = "Unknown";
    }