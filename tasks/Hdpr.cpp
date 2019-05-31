#include "Hdpr.hpp"
#include <platform_driver/PlatformDriverPcan.h>

using namespace platform_driver;

Hdpr::Hdpr(std::string const& name) : HdprBase(name) {}
Hdpr::Hdpr(std::string const& name, RTT::ExecutionEngine* engine) : HdprBase(name, engine) {}
Hdpr::~Hdpr() {}

void Hdpr::setJointCommands()
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
                    if (start_motor_[i] && i < 6)
                    {
                        platform_driver_->startNode(i);
                        start_motor_[i] = false;
                    }
                    platform_driver_->nodeVelocityCommandRadS(i, joint.speed);

                    if (i == 10 && joint.speed == 0)
                    {
                        for (int j = 0; j < 6; j++)
                        {
                            stop_motor_[j] = true;
                        }
                    }
                    else if (i < 6)
                    {
                        stop_motor_[i] = false;
                    }
                }
            }
        }
    }
}

void Hdpr::getJointInformation()
{
    bool error_in_motor = false;
    for (int i = 0; i < num_motors_; i++)
    {
        bool status = platform_driver_->getNodeData(i, &position_, &velocity_, &current_, &torque_);

        // Joints readings & status information
        base::JointState& joint(joints_readings_[i]);
        joint.position = position_;
        joint.speed = velocity_;
        if (stop_motor_[i] && i < 6 && std::abs(joint.speed) < 0.01)
        {
            platform_driver_->nodeTorqueCommandNm(i, 0.0);
            platform_driver_->shutdownNode(i);
            stop_motor_[i] = false;
            start_motor_[i] = true;
        }
        joint.raw = current_;
        joint.effort = torque_;

        if (can_parameters_.Active[i] && status)
            joints_status_[i] = true;
        else
            joints_status_[i] = false;

        if (!status)
        {
            if (joints_resurrection_[i] < 3)
            {
                // In case of reseting a node stop the motion of the rest of the motor
                for (size_t j = 0; j < static_cast<size_t>(num_motors_); ++j)
                {
                    if (can_parameters_.Active[j])
                    {
                        platform_driver_->nodeVelocityCommandRadS(j, 0.0);
                    }
                }
                LOG_INFO_S << "Resetting motor " << i;
                platform_driver_->resetNode(i);
                joints_resurrection_[i]++;
            }
            else
            {
                if (can_parameters_.Active[i] == ACTIVE)
                {
                    // In case of inactivating a node stop the motion of the rest of the motor
                    for (size_t j = 0; j < static_cast<size_t>(num_motors_); ++j)
                    {
                        if (can_parameters_.Active[j])
                        {
                            platform_driver_->nodeVelocityCommandRadS(j, 0.0);
                        }
                    }
                    platform_driver_->shutdownNode(i);
                    LOG_INFO_S << "Motor " << i << " INACTIVE";
                    can_parameters_.Active[i] = INACTIVE;
                }
            }
            _error_in_motor.write(i + 1);
            error_in_motor = true;
        }
    }
    if (!error_in_motor) _error_in_motor.write(0);

    // Get the Passive joints information
    for (const auto& passive_joint : passive_config_)
    {
        platform_driver_->getNodeAnalogInput(passive_joint.id, &analog_input_);

        base::JointState& joint(joints_readings_[passive_joint.name]);
        if (passive_joint.id == 3 || passive_joint.id == 5)
            joint.position = (-analog_input_) * bogie_pitch_factor_;  //*D2R;
        else
            joint.position = (analog_input_)*bogie_pitch_factor_;  //*D2R;
    }
}

bool Hdpr::configureHook()
{
    if (!HdprBase::configureHook() || !Task::configureHook())
    {
        return false;
    }

    platform_driver_.reset(new PlatformDriverPcan(
        _num_motors, _num_nodes, _can_dev_type, _can_dev_address, _watchdog));

    return true;
}
