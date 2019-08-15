#include "Task.hpp"
#include <platform_driver_ethercat/PlatformDriverEthercat.h>

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
    const auto dev_address = _dev_address.value();
    const auto num_slaves = _num_slaves.value();
    drive_mapping_ = _drive_mapping.get();
    fts_mapping_ = _fts_mapping.get();
    passive_mapping_ = _passive_mapping.get();

    joints_readings_.resize(drive_mapping_.size() + passive_mapping_.size());
    fts_readings_.resize(fts_mapping_.size());
    temp_readings_.resize(drive_mapping_.size());

    // Fill the joint names with the drive mapping names
    size_t i = 0;
    for (const auto& joint : drive_mapping_)
    {
        joints_readings_.names[i] = joint.name;
        ++i;
    }

    // Fill the joint names with the passive mapping names
    for (const auto& joint : passive_mapping_)
    {
        joints_readings_.names[i] = joint.name;
        ++i;
    }

    // Fill the fts output names with the fts mapping names
    i = 0;
    for (const auto& fts : fts_mapping_)
    {
        fts_readings_.names[i] = fts.name;
        ++i;
    }

    // Fill the temp output names with the drive mapping names
    i = 0;
    for (const auto& joint : drive_mapping_)
    {
        temp_readings_.names[i] = joint.name;
        ++i;
    }

    platform_driver_.reset(
        new PlatformDriverEthercat(dev_address, num_slaves, drive_mapping_, fts_mapping_));

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

    evalJointsCommands();

    updateJointsReadings();
    updateFtsReadings();
    updateTempReadings();

    joints_readings_.time = base::Time::now();
    _joints_readings.write(joints_readings_);

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

void Task::evalJointsCommands()
{
    base::commands::Joints joints_commands;

    if (_joints_commands.read(joints_commands, false) == RTT::NewData)
    {
        for (size_t i = 0; i < joints_commands.size(); ++i)
        {
            base::JointState& joint(joints_commands[i]);

            if (joint.isPosition())
            {
                platform_driver_->commandDrivePositionRad(joints_commands.names[i], joint.position);
            }
            else if (joint.isSpeed())
            {
                platform_driver_->commandDriveVelocityRadSec(joints_commands.names[i], joint.speed);
            }
        }
    }
}

void Task::updateJointsReadings()
{
    // get joints readings
    size_t i = 0;
    for (const auto& drive_params : drive_mapping_)
    {
        double position, velocity, current, torque;

        bool is_error =
            platform_driver_->readDriveData(drive_params.name, position, velocity, current, torque);

        base::JointState& joint(joints_readings_[i]);
        joint.position = position;
        joint.speed = velocity;
        joint.raw = current;
        joint.effort = torque;

        ++i;
    }

    // TODO: get passive joints information
    for (const auto& passive_params : passive_mapping_)
    {
        base::JointState& joint(joints_readings_[i]);
        joint.position = 0;  // set to zero until encoders are working

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
    for (const auto& drive_params : drive_mapping_)
    {
        double& motor_temp(temp_readings_[i]);
        platform_driver_->readDriveAnalogInputV(
            drive_params.name, motor_temp);  // TODO: Implement conversion to degrees

        ++i;
    }
}
