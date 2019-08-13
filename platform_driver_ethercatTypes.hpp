#pragma once

#include <string>
#include <base/NamedVector.hpp>
#include <base/Time.hpp>

namespace platform_driver_ethercat
{
    struct PassiveJointParams
    {
        unsigned int drive_id;
        std::string name;
    };

    typedef std::vector<PassiveJointParams> PassiveJointMapping;

    struct Temperatures : public base::NamedVector<double>
    {
        base::Time time;
    };
}
