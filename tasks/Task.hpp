#pragma once

#include <base/Time.hpp>
#include <base/commands/Joints.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/Wrenches.hpp>

#include "platform_driver_ethercat/TaskBase.hpp"
#include "platform_driver_ethercat/platform_driver_ethercatTypes.hpp"

#include <base-logging/Logging.hpp>

namespace platform_driver_ethercat
{

class PlatformDriverEthercat;

class Task : public TaskBase
{
    friend class TaskBase;

  protected:
    virtual void setJointCommands() = 0;
    virtual void getJointInformation() = 0;

    std::unique_ptr<PlatformDriverEthercat> platform_driver_;

    // Configuration variables
    int num_motors_;
    double system_current_factor_;
    double system_voltage_factor_;
    PltfCanParams can_parameters_;

    // Analog configuration
    std::vector<platform_driver_ethercat::AnalogId> passive_config_;
    std::vector<platform_driver_ethercat::AnalogId> analog_config_;

    base::commands::Joints joints_commands_;
    base::samples::Joints joints_readings_;

    double degToRad(const double deg) const;
    double radToDeg(const double rad) const;

  public:
    Task(std::string const& name = "platform_driver_ethercat::Task");
    Task(std::string const& name, RTT::ExecutionEngine* engine);
    ~Task();
    virtual bool configureHook();
    bool startHook();
    virtual void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};

}  // namespace platform_driver_ethercat
