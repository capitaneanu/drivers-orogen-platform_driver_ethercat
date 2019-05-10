#pragma once

#include "platform_driver/TaskBase.hpp"
#include "platform_driver/platform_driverTypes.hpp"

#include <base/Time.hpp>
#include <base/commands/Joints.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/Wrenches.hpp>

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

    Platform_Driver* m_pPlatform_Driver;
    base::Time t;
    double dPositionRad;
    double dVelocityRadS;
    double dCurrentAmp;
    double dTorqueNm;
    double dAnalogInput;

    /****************************/
    /*  Configuration Variables */
    /****************************/

    int numMotors;
    double system_current_factor;
    double system_voltage_factor;
    double bogie_pitch_factor;

    /** Can Parameters **/
    PltfCanParams canParameters;

    /** Analog configuration **/
    std::vector<platform_driver::AnalogId> passiveConfig, analogConfig;

    /**********************/
    /*  Input  Variables */
    /*********************/

    /** Commands of the Joints **/
    base::commands::Joints joints_commands;

    /************************/
    /*  Internal  Variables */
    /************************/

    uint64_t sample_index;

    std::vector<bool> joints_status;
    std::vector<bool> stop_motor;
    std::vector<bool> start_motor;
    std::vector<unsigned int> joints_resurrection;  // Counter to keep track of anumber of attends
                                                    // to (re)start a joint/motor

    /**********************/
    /*  Output  Variables */
    /**********************/

    /** Reads the status of the joints */
    base::samples::Joints joints_readings;

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

}

