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
    joint_mapping_ = _joint_mapping.get();

    fts_readings_.resize(fts_mapping_.size());
    joints_readings_.resize(joint_mapping_.size());
    temp_readings_.resize(drive_mapping_.size());

    platform_driver_.reset(new PlatformDriverEthercat(dev_address, num_slaves));

    // Add the drives to the platform driver
    for (const auto& drive : drive_mapping_)
    {
        std::cout << "Drive name: " << drive.name << std::endl;
        platform_driver_->addDriveTwitter(
            drive.slave_id, drive.name, drive.config, drive.temp_sensor);
    }

    // Fill the fts output names with the fts mapping names and add the fts to platform driver
    size_t i = 0;
    for (const auto& fts : fts_mapping_)
    {
        fts_readings_.names[i] = fts.name;
        platform_driver_->addAtiFts(fts.slave_id, fts.name);
        ++i;
    }

    // Fill the joint names with the joint mapping names and add the joints to platform driver
    i = 0;
    for (const auto& joint : joint_mapping_)
    {
        joints_readings_.names[i] = joint.name;

        std::cout << "Joint name: " << joint.name << " Drive name: " << joint.drive << std::endl;

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

    // Fill the temp output names with the drive mapping names
    i = 0;
    for (const auto& drive : drive_mapping_)
    {
        temp_readings_.names[i] = drive.name;
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
                platform_driver_->commandJointPositionRad(joints_commands.names[i], joint.position);
            }
            else if (joint.isSpeed())
            {
                platform_driver_->commandJointVelocityRadSec(joints_commands.names[i], joint.speed);
            }
        }
    }
}

void Task::updateJointsReadings()
{
    // get joints readings
    size_t i = 0;
    for (const auto& joint : joint_mapping_)
    {
        double position, velocity, torque;

        platform_driver_->readJointPositionRad(joint.name, position);
        platform_driver_->readJointVelocityRadSec(joint.name, velocity);
        platform_driver_->readJointTorqueNm(joint.name, torque);

        base::JointState& joint_state(joints_readings_[i]);
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
   // size_t i = 0;
   // for (const auto& drive_params : drive_mapping_)
   // {
   //     double& motor_temp(temp_readings_[i]);
   //     platform_driver_->readDriveAnalogInputV(
   //         drive_params.name, motor_temp);  // TODO: Implement conversion to degrees

   //     ++i;
   // }
}
