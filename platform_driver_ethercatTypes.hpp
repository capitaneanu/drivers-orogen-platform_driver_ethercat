#pragma once

#include <string>
#include <base/NamedVector.hpp>
#include <base/Time.hpp>
#include <platform_driver_ethercat/CanEnumsAndStructs.h>

namespace platform_driver_ethercat
{
    struct SlaveMap
    {
        int slave;
        std::string name;
    };

    struct MotorMap : public SlaveMap
    {
        GearMotorParamType config;
        bool active;
    };

    struct Temperatures : public base::NamedVector<double>
    {
        base::Time time;
    };
}
