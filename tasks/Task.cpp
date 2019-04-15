#include "Task.hpp"
#include <platform_driver/Platform_Driver.h>

using namespace platform_driver;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    canParameters = _can_parameters.get();
    analogConfig = _analog_readings_config.get();
    passiveConfig = _passive_readings_config.get();
    numMotors=_num_motors.value();

    if (static_cast<unsigned int>(_num_nodes) != _can_parameters.get().CanId.size() ||
        _can_parameters.get().CanId.size() != _can_parameters.get().Name.size() ||
        _can_parameters.get().Name.size() != _can_parameters.get().Type.size())
    {
        LOG_ERROR_S << "wrong config "<< _num_nodes << " " << _can_parameters.get().CanId.size() << " " << _can_parameters.get().Name.size() << " " << _can_parameters.get().Type.size();
        return false;
    }

    m_pPlatform_Driver = new Platform_Driver(_num_motors,_num_nodes,_can_dev_type,_can_dev_address,_watchdog);

    joints_status.resize(numMotors);
    joints_readings.resize(numMotors + passiveConfig.size() + analogConfig.size());
    system_current_factor=_current_factor.value();
    system_voltage_factor=_voltage_factor.value();
    bogie_pitch_factor=_bogie_factor.value();
    joints_resurrection.resize(numMotors);
    sample_index=0;
    stop_motor.resize(numMotors);
    start_motor.resize(numMotors);

    // Initialize values of joints_resurrection
    for(int j = 0; j < numMotors; ++j)
    {
        joints_resurrection[j] = 0;
        stop_motor[j]=false;
        start_motor[j]=true;
    }

    // Fill the Joints names with the can_parameters names
    size_t i = 0;
    for(int j = 0; j < numMotors; ++j)
    {
        joints_readings.names[i] = canParameters.Name[j];
        ++i;
    }

    // Fill the Joints names with the passive_config names
    for(const auto& joint : passiveConfig)
    {
        joints_readings.names[i] = joint.name;
        ++i;
    }

    // Fill the Joints names with the system info
    for(const auto& joint : analogConfig)
    {
        joints_readings.names[i] = joint.name;
        ++i;
    }

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    if (m_pPlatform_Driver->initPltf(_param_gear_motor_wheel,
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

    joints_readings.time = base::Time::now();
    _joints_readings.write(joints_readings);
}

void Task::errorHook()
{
    TaskBase::errorHook();

    m_pPlatform_Driver->shutdownPltf();
}

void Task::stopHook()
{
    TaskBase::stopHook();

    m_pPlatform_Driver->shutdownPltf();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    delete m_pPlatform_Driver;
}
