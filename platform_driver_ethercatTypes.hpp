#pragma once

#include <platform_driver_ethercat/PlatformDriverEthercatTypes.h>
#include <base/NamedVector.hpp>
#include <base/Time.hpp>
#include <string>
#include <vector>

namespace platform_driver_ethercat
{
struct GenericSlaveParams
{
    unsigned int slave_id;
    std::string name;
};

struct DriveSlaveParams : public GenericSlaveParams
{
    DriveConfig config;
    bool enabled;
};

struct FtsSlaveParams : public GenericSlaveParams
{
};

enum JointType
{
    ACTIVE,
    PASSIVE
};

struct JointParams
{
    std::string name;
    std::string drive;
    JointType type;
    bool enabled;
};

typedef std::vector<GenericSlaveParams> GenericSlaveMapping;
typedef std::vector<DriveSlaveParams> DriveSlaveMapping;
typedef std::vector<FtsSlaveParams> FtsSlaveMapping;
typedef std::vector<JointParams> JointMapping;

struct Temperatures : public base::NamedVector<double>
{
    base::Time time;
};
}
