#include <platform_driver_ethercat/PlatformDriverEthercat.h>
#include <sys/stat.h>
#include <base-logging/Logging.hpp>
#include <base/Time.hpp>
#include <base/commands/Joints.hpp>
#include <set>

#include "Task.hpp"

using namespace platform_driver_ethercat;

Task::Task(std::string const& name) : TaskBase(name) {}
Task::Task(std::string const& name, RTT::ExecutionEngine* engine) : TaskBase(name, engine) {}
Task::~Task() {}

bool Task::configureHook()
{
    if (!TaskBase::configureHook())
    {
        return false;
    }

    // Read configuration
    dev_address_ = _dev_address.get();
    num_slaves_ = _num_slaves.get();
    drive_mapping_ = _drive_mapping.get();
    fts_mapping_ = _fts_mapping.get();
    joint_mapping_ = _joint_mapping.get();

    if (!validateConfig())
    {
        return false;
    }

    fts_readings_.resize(fts_mapping_.size());
    joint_readings_.resize(joint_mapping_.size());
    temp_readings_.resize(joint_mapping_.size());

    platform_driver_.reset(new PlatformDriverEthercat(dev_address_, num_slaves_));

    // Add the drives to the platform driver
    for (const auto& drive : drive_mapping_)
    {
        platform_driver_->addDriveTwitter(
            drive.slave_id, drive.name, drive.config);
    }

    // Fill the fts output names with the fts mapping names and add the fts to platform driver
    size_t i = 0;
    for (const auto& fts : fts_mapping_)
    {
        fts_readings_.names[i] = fts.name;
        platform_driver_->addAtiFts(fts.slave_id, fts.name);
        ++i;
    }

    // Fill the joint and temp output names with the joint mapping names and add the joints to
    // platform driver
    i = 0;
    for (const auto& joint : joint_mapping_)
    {
        joint_readings_.names[i] = joint.name;
        temp_readings_.names[i] = joint.name;

        if (joint.type == ACTIVE)
        {
            platform_driver_->addActiveJoint(joint.name, joint.drive, joint.enabled);
        }
        else if (joint.type == PASSIVE)
        {
            platform_driver_->addPassiveJoint(joint.name, joint.drive, joint.enabled);
        }

        ++i;
    }

    return true;
}

bool Task::startHook()
{
    if (!TaskBase::startHook()) return false;

    if (platform_driver_->initPlatform())
    {
        return true;
    }

    return false;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    evalJointCommands();

    updateJointReadings();
    updateFtsReadings();
    updateTempReadings();

    joint_readings_.time = base::Time::now();
    _joints_readings.write(joint_readings_);

    fts_readings_.time = base::Time::now();
    _fts_readings.write(fts_readings_);

    temp_readings_.time = base::Time::now();
    _temp_readings.write(temp_readings_);
}

void Task::errorHook()
{
    TaskBase::errorHook();
    platform_driver_->shutdownPlatform();
}

void Task::stopHook()
{
    TaskBase::stopHook();
    platform_driver_->shutdownPlatform();
}

void Task::cleanupHook() { TaskBase::cleanupHook(); }

bool Task::validateConfig()
{
    // Check if interface exists
    struct stat buffer;
    if (stat(("/sys/class/net/" + dev_address_).c_str(), &buffer) != 0)
    {
        LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Interface " << dev_address_ << " does not exist";
        return false;
    }

    // Check if num slaves is valid
    if (num_slaves_ <= 0)
    {
        LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Invalid number of slaves " << num_slaves_;
        return false;
    }

    std::set<unsigned int> id_set;
    std::set<std::string> name_set;

    auto validateDevice = [&id_set, &name_set](GenericSlaveParams device) {
        const auto& slave_id = device.slave_id;
        const auto& name = device.name;

        // Check if slave id is valid
        if (slave_id <= 0)
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Invalid slave id " << slave_id;
            return false;
        }

        // Check if slave id already exists
        if (id_set.find(slave_id) != id_set.end())
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Slave id " << slave_id << " already exists";
            return false;
        }

        id_set.insert(slave_id);

        // Check if device name already exists
        if (name_set.find(name) != name_set.end())
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Device name " << name << " already exists";
            return false;
        }

        name_set.insert(name);

        return true;
    };

    std::set<std::string> drive_set;
    for (const auto& drive : drive_mapping_)
    {
        if (!validateDevice(drive)) return false;
        drive_set.insert(drive.name);
    }

    for (const auto& fts : fts_mapping_)
    {
        if (!validateDevice(fts)) return false;
    }

    std::set<std::string> joint_set, active_set, passive_set;
    for (const auto& joint : joint_mapping_)
    {
        const auto& name = joint.name;
        const auto& drive = joint.drive;

        // Check if joint name already exists
        if (joint_set.find(name) != joint_set.end())
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Joint name " << name << " already exists";
            return false;
        }
        joint_set.insert(name);

        // Check if drive name does not exist
        if (drive_set.find(drive) == drive_set.end())
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Drive " << drive << " for joint " << name
                        << " does not exist";
            return false;
        }

        if (joint.type == ACTIVE)
        {
            // Check if the same drive is already in use for another active joint
            if (active_set.find(drive) != active_set.end())
            {
                LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Drive " << drive
                            << " already in use with another active joint";
                return false;
            }

            active_set.insert(drive);
        }
        else if (joint.type == PASSIVE)
        {
            // Check if the same drive is already in use for another passive joint
            if (passive_set.find(drive) != passive_set.end())
            {
                LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Drive " << drive
                            << " already in use with another passive joint";
                return false;
            }

            passive_set.insert(drive);
        }
    }

    return true;
}

void Task::evalJointCommands()
{
    base::commands::Joints joint_commands;

    if (_joints_commands.read(joint_commands, false) == RTT::NewData)
    {
        for (size_t i = 0; i < joint_commands.size(); ++i)
        {
            base::JointState& joint(joint_commands[i]);

            if (joint.isPosition())
            {
                platform_driver_->commandJointPositionRad(joint_commands.names[i], joint.position);
            }
            else if (joint.isSpeed())
            {
                platform_driver_->commandJointVelocityRadSec(joint_commands.names[i], joint.speed);
            }
        }
    }
}

void Task::updateJointReadings()
{
    size_t i = 0;
    for (const auto& joint : joint_mapping_)
    {
        double position, velocity, torque;

        platform_driver_->readJointPositionRad(joint.name, position);
        platform_driver_->readJointVelocityRadSec(joint.name, velocity);
        platform_driver_->readJointTorqueNm(joint.name, torque);

        base::JointState& joint_state(joint_readings_[i]);
        joint_state.position = position;
        joint_state.speed = velocity;
        joint_state.effort = torque;

        ++i;
    }
}

void Task::updateFtsReadings()
{
    size_t i = 0;
    for (const auto& fts_params : fts_mapping_)
    {
        double fx, fy, fz;
        double tx, ty, tz;

        platform_driver_->readFtsForceN(fts_params.name, fx, fy, fz);
        platform_driver_->readFtsTorqueNm(fts_params.name, tx, ty, tz);

        base::Wrench& wrench(fts_readings_[i]);
        wrench.force = base::Vector3d(fx, fy, fz);
        wrench.torque = base::Vector3d(tx, ty, tz);

        ++i;
    }
}

void Task::updateTempReadings()
{
    size_t i = 0;
    for (const auto& joint : joint_mapping_)
    {
        double& joint_temp(temp_readings_[i]);
        platform_driver_->readJointTempDegC(joint.name, joint_temp);
        ++i;
    }
}
