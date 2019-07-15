#include "Marta.hpp"
#include <platform_driver_ethercat/PlatformDriverEthercat.h>

using namespace platform_driver_ethercat;

Marta::Marta(std::string const& name) : MartaBase(name) {}
Marta::Marta(std::string const& name, RTT::ExecutionEngine* engine) : MartaBase(name, engine) {}
Marta::~Marta() {}

bool Marta::configureHook()
{
    if (!MartaBase::configureHook() || !Task::configureHook())
    {
        return false;
    }

    num_fts_ = _num_fts.value();
    fts_readings_.resize(num_fts_);
    motor_temps_.resize(num_motors_);

    for (int i = 0; i < num_fts_; ++i)
    {
        fts_readings_.names[i] = can_parameters_.Name[num_motors_ + i];
    }

    for (int i = 0; i < num_motors_; ++i)
    {
        motor_temps_.names[i] = can_parameters_.Name[i];
    }

    platform_driver_.reset(new PlatformDriverEthercat(
        _num_motors, _num_nodes, _can_dev_type, _can_dev_address, _watchdog));

    return true;
}

void Marta::updateHook()
{
    MartaBase::updateHook();
    Task::updateHook();

    getFtsInformation();

    fts_readings_.time = base::Time::now();
    _fts_readings.write(fts_readings_);

    getMotorTemps();

    motor_temps_.time = base::Time::now();
    _motor_temps.write(motor_temps_);
}

void Marta::setJointCommands()
{
    if (_joints_commands.read(joints_commands_, false) == RTT::NewData)
    {
        for (size_t i = 0; i < joints_commands_.size(); ++i)
        {
            if (can_parameters_.Active[i])
            {
                base::JointState& joint(joints_commands_[i]);
                if (joint.isPosition())
                {
                    platform_driver_->nodePositionCommandRad(i, joint.position);
                }
                else if (joint.isSpeed())
                {
                    platform_driver_->nodeVelocityCommandRadS(i, joint.speed);
                }
            }
        }
    }
}

void Marta::getJointInformation()
{
    // get joints readings
    for (int i = 0; i < num_motors_; i++)
    {
        double position, velocity, current, torque;

        bool is_error = platform_driver_->getNodeData(i, &position, &velocity, &current, &torque);

        base::JointState& joint(joints_readings_[i]);
        joint.position = position;
        joint.speed = velocity;
        joint.raw = current;
        joint.effort = torque;
    }

    // TODO: get passive joints information
    for (const auto& passive_joint : passive_config_)
    {
        base::JointState& joint(joints_readings_[passive_joint.name]);
        joint.position = 0;  // set to zero until encoders are working
    }

    // TODO: get system volt / amp information
}

void Marta::getFtsInformation()
{
    for (int i = 0; i < num_fts_; i++)
    {
        double fx, fy, fz;
        double tx, ty, tz;

        platform_driver_->getNodeFtsForceN(i, &fx, &fy, &fz);
        platform_driver_->getNodeFtsForceN(i, &fx, &fy, &fz);

        auto& wrench(fts_readings_[i]);
        wrench.force = base::Vector3d(fx, fy, fz);
        wrench.torque = base::Vector3d(tx, ty, tz);
    }
}

void Marta::getMotorTemps()
{
    for (int i = 0; i < num_motors_; i++)
    {
        double& motor_temp = motor_temps_[i];
        platform_driver_->getNodeAnalogInputV(i, &motor_temp);  // TODO: Implement conversion to degrees
    }
}
