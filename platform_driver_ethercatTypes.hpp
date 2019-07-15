#pragma once

#include <string>
#include <base/NamedVector.hpp>
#include <base/Time.hpp>

namespace platform_driver_ethercat
{
    struct AnalogId
    {
        int id;
        std::string name;
    };

    struct Temperatures : public base::NamedVector<double>
    {
        base::Time time;
    };
}
