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

    num_nodes_ = _num_nodes.value();
    system_current_factor_ = _current_factor.value();
    system_voltage_factor_ = _voltage_factor.value();
    motor_mapping_ = _motor_mapping.get();
    passive_joint_mapping_ = _passive_joint_mapping.get();
    temp_mapping_ = _temp_mapping.get();

    joints_readings_.resize(motor_mapping_.size() + passive_joint_mapping_.size());

    // Fill the Joints names with the can_parameters names
    size_t i = 0;
    for (int j = 0; j < num_motors_; ++j)
    {
        joints_readings_.names[i] = can_parameters_.Name[j];
        ++i;
    }

    // Fill the Joints names with the passive_config names
    for (const auto& joint : passive_config_)
    {
        joints_readings_.names[i] = joint.name;
        ++i;
    }

    // Fill the Joints names with the system info
    for (const auto& joint : analog_config_)
    {
        joints_readings_.names[i] = joint.name;
        ++i;
    }

    return true;
}

bool Task::startHook()
{
    if (!TaskBase::startHook()) return false;

    if (platform_driver_->initPltf(_param_gear_motor_wheel,
                                   _param_gear_motor_steer,
                                   _param_gear_motor_walk,
                                   _param_gear_motor_pan,
                                   _param_gear_motor_tilt,
                                   _param_gear_motor_arm,
                                   _can_parameters))
    {
        return true;
    }

    return false;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    setJointCommands();
    getJointInformation();

    joints_readings_.time = base::Time::now();
    _joints_readings.write(joints_readings_);
}

void Task::errorHook()
{
    TaskBase::errorHook();
    platform_driver_->shutdownPltf();
}

void Task::stopHook()
{
    TaskBase::stopHook();
    platform_driver_->shutdownPltf();
}

void Task::cleanupHook() { TaskBase::cleanupHook(); }

double Task::degToRad(const double deg) const { return deg * M_PI / 180.0; }
double Task::radToDeg(const double rad) const { return rad * 180.0 / M_PI; }
