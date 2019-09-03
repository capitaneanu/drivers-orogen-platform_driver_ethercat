#pragma once

#include <platform_driver_ethercat/PlatformDriverEthercatTypes.h>
#include <base/NamedVector.hpp>
#include <base/Time.hpp>
#include <string>
#include <vector>

namespace platform_driver_ethercat
{
struct SlaveParams
{
    unsigned int slave_id;
    std::string name;
};

struct DriveSlaveParams : public SlaveParams
{
    DriveConfig config;
    bool enabled;
};

struct FtsSlaveParams : public SlaveParams
{
};

struct JointParams
{
    std::string name;
    std::string drive;
    bool enabled;
};

struct JointConfig
{
    bool flip_sign;
    double min_position_rad;
    double max_position_rad;
    double max_velocity_rad_sec;
    double max_torque_nm;
};

struct ActiveJointParams : public JointParams
{
    JointConfig config;
};

struct PassiveJointParams : public JointParams
{
};

typedef std::vector<DriveSlaveParams> DriveSlaveMapping;
typedef std::vector<FtsSlaveParams> FtsSlaveMapping;
typedef std::vector<ActiveJointParams> ActiveJointMapping;
typedef std::vector<PassiveJointParams> PassiveJointMapping;

struct Temperatures : public base::NamedVector<double>
{
    base::Time time;
};
}
