#pragma once

#include <base/Time.hpp>
#include <base/commands/Joints.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/Wrenches.hpp>

#include "platform_driver/TaskBase.hpp"
#include "platform_driver/platform_driverTypes.hpp"

#include <base-logging/Logging.hpp>

class Platform_Driver;

namespace platform_driver
{

class Task : public TaskBase
{
    friend class TaskBase;

  protected:
    virtual void setJointCommands() = 0;
    virtual void getJointInformation() = 0;

    Platform_Driver* platform_driver_;

    // position in rad
    double position_;

    // velocity in rad/s
    double velocity_;

    // current in ampers
    double current_;

    // torque in newton meters
    double torque_;

    double analog_input_;

    // Configuration variables
    int num_motors_;
    double system_current_factor_;
    double system_voltage_factor_;
    double bogie_pitch_factor_;

    PltfCanParams can_parameters_;

    // Analog configuration
    std::vector<platform_driver::AnalogId> passive_config_;
    std::vector<platform_driver::AnalogId> analog_config_;

    base::commands::Joints joints_commands_;

    uint64_t sample_index_;

    std::vector<bool> joints_status_;
    std::vector<bool> stop_motor_;
    std::vector<bool> start_motor_;

    // Counter to keep track of anumber of attends to (re)start a joint/motor
    std::vector<unsigned int> joints_resurrection_;

    base::samples::Joints joints_readings_;

    double degToRad(const double deg) const;
    double radToDeg(const double rad) const;

  public:
    Task(std::string const& name = "platform_driver::Task");
    Task(std::string const& name, RTT::ExecutionEngine* engine);
    ~Task();
    virtual bool configureHook();
    bool startHook();
    virtual void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};

}  // namespace platform_driver
