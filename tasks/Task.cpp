#include "Task.hpp"
#include <platform_driver/PlatformDriver.h>

using namespace platform_driver;

Task::Task(std::string const& name) : TaskBase(name) {}
Task::Task(std::string const& name, RTT::ExecutionEngine* engine) : TaskBase(name, engine) {}
Task::~Task() {}

bool Task::configureHook()
{
    if (!TaskBase::configureHook())
    {
        return false;
    }

    can_parameters_ = _can_parameters.get();
    analog_config_ = _analog_readings_config.get();
    passive_config_ = _passive_readings_config.get();
    num_motors_ = _num_motors.value();

    if (static_cast<unsigned int>(_num_nodes) != _can_parameters.get().CanId.size()
        || _can_parameters.get().CanId.size() != _can_parameters.get().Name.size()
        || _can_parameters.get().Name.size() != _can_parameters.get().Type.size())
    {
        LOG_ERROR_S << "wrong config " << _num_nodes << " " << _can_parameters.get().CanId.size()
                    << " " << _can_parameters.get().Name.size() << " "
                    << _can_parameters.get().Type.size();
        return false;
    }

    //platform_driver_ =
    //    new PlatformDriver(_num_motors, _num_nodes, _can_dev_type, _can_dev_address, _watchdog);

    joints_status_.resize(num_motors_);
    joints_readings_.resize(num_motors_ + passive_config_.size() + analog_config_.size());
    system_current_factor_ = _current_factor.value();
    system_voltage_factor_ = _voltage_factor.value();
    bogie_pitch_factor_ = _bogie_factor.value();
    joints_resurrection_.resize(num_motors_);
    sample_index_ = 0;
    stop_motor_.resize(num_motors_);
    start_motor_.resize(num_motors_);

    // Initialize values of joints_resurrection_
    for (int j = 0; j < num_motors_; ++j)
    {
        joints_resurrection_[j] = 0;
        stop_motor_[j] = false;
        start_motor_[j] = true;
    }

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

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

double Task::degToRad(const double deg) const { return deg * M_PI / 180.0; }
double Task::radToDeg(const double rad) const { return rad * 180.0 / M_PI; }
