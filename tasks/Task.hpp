#pragma once

#include <base/Time.hpp>
#include <base/commands/Joints.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/Wrenches.hpp>
#include <memory>

#include "platform_driver_ethercat/PlatformDriverEthercatTypes.h"
#include "platform_driver_ethercat/TaskBase.hpp"
#include "platform_driver_ethercatTypes.hpp"

#include <base-logging/Logging.hpp>

namespace platform_driver_ethercat
{

class PlatformDriverEthercat;

class Task : public TaskBase
{
    friend class TaskBase;

  public:
    Task(std::string const& name = "platform_driver_ethercat::Task");
    Task(std::string const& name, RTT::ExecutionEngine* engine);
    ~Task();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();

  protected:
    void evalJointsCommands();
    void updateJointsReadings();
    void updateFtsReadings();
    void updateTempReadings();

    std::unique_ptr<PlatformDriverEthercat> platform_driver_;

    DriveSlaveMapping drive_mapping_;
    FtsSlaveMapping fts_mapping_;
    PassiveJointMapping passive_mapping_;

    base::samples::Joints joints_readings_;
    base::samples::Wrenches fts_readings_;
    Temperatures temp_readings_;
};

}
