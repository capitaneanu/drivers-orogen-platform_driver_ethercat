/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <platform_driver/Platform_Driver.h>


#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

using namespace platform_driver;
using namespace base::actuators;

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

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /** Get the configurations **/
    canParameters = _can_parameters.get();
    analogConfig = _analog_readings_config.get();
    passiveConfig = _passive_readings_config.get();
    numMotors=_num_motors.value();


    if (static_cast<unsigned int>(_num_nodes) != _can_parameters.get().CanId.size() || _can_parameters.get().CanId.size()!=_can_parameters.get().Name.size() || _can_parameters.get().Name.size()!=_can_parameters.get().Type.size())
    {
    	std::cout<<"wrong config "<< _num_nodes << " " << _can_parameters.get().CanId.size() << " " << _can_parameters.get().Name.size() << " " << _can_parameters.get().Type.size() <<std::endl;
    	return false;
    }


    m_pPlatform_Driver = new Platform_Driver(_num_motors,_num_nodes,_can_dev_type,_can_dev_address,_watchdog);

    joints_status.resize(numMotors);
    joints_readings.resize(numMotors+passiveConfig.size()+analogConfig.size());
    system_current_factor=_current_factor.value();
    system_voltage_factor=_voltage_factor.value();
    bogie_pitch_factor=_bogie_factor.value();
    joints_resurrection.resize(numMotors);
    sample_index=0;
    
    // Initialize values of joints_resurrection
    for(register int j = 0; j < numMotors; ++j)
    {
        joints_resurrection[j] = 0;
    }

    // Fill the Joints names with the can_parameters names
    size_t i = 0;
    for(register int j = 0; j < numMotors; ++j)
    {
        joints_readings.names[i] = canParameters.Name[j];
        ++i;
    }

    // Fill the Joints names with the passive_config names
    for(std::vector<platform_driver::AnalogId>::iterator it = passiveConfig.begin(); it != passiveConfig.end(); ++it)
    {
        joints_readings.names[i] = it->name;
        ++i;
    }

    // Fill the Joints names with the system info
    for(std::vector<platform_driver::AnalogId>::iterator it = analogConfig.begin(); it != analogConfig.end(); ++it)
    {
        joints_readings.names[i] = it->name;
        ++i;
    }

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    if (m_pPlatform_Driver->initPltf(_param_gear_motor_wheel,_param_gear_motor_steer,_param_gear_motor_walk,_param_gear_motor_pan,_param_gear_motor_tilt,_param_gear_motor_arm,_can_parameters))
    	return true;

    return false; 
}
void Task::updateHook()
{
    TaskBase::updateHook();

    /****************/
    /** COMMANDING **/
    /****************/

    /** Set New Joints commands **/
    if (_joints_commands.read(joints_commands, false) == RTT::NewData)
    {
        //std::cout<<"platform_driver::Task::updateHook : new joints command received"<<std::endl;
        for (size_t i=0; i<joints_commands.size(); ++i)
        {
		if (canParameters.Active[i])
		{
                	base::JointState &joint(joints_commands[i]);
	                if (joint.isPosition())
        	        {
                	        m_pPlatform_Driver->nodePositionCommandRad(i, joint.position);
	                }
        	        else if (joint.isSpeed())
                	{
	                        m_pPlatform_Driver->nodeVelocityCommandRadS(i, joint.speed);
        	        }
		}
       }
    }

    /*************/
    /** READING **/
    /*************/

    /** Time stamp for the joints port**/
    joints_readings.time = base::Time::now();

    /** Get the Joints information **/
    for (int i=0;i<numMotors;i++)
    {
    	bool status=m_pPlatform_Driver->getNodeData(i, &dPositionRad, &dVelocityRadS, &dCurrentAmp, &dTorqueNm);
	
        /** Joints readings & status information **/
	base::JointState &joint(joints_readings[i]);
        joint.position = dPositionRad;
        joint.speed = dVelocityRadS;
        joint.raw = dCurrentAmp;
        joint.effort = dTorqueNm;

	if (canParameters.Active[i] && status)
		joints_status[i]=true;
	else
		joints_status[i]=false;

	if (!status)
	{
		if (joints_resurrection[i]<3)
                {
			//! In case of reseting a node stop the motion of the rest of the motor
                        for (register size_t j=0;j<static_cast<size_t>(numMotors);j++)
                        {
                                if (canParameters.Active[j])
                                {
                                        m_pPlatform_Driver->nodeVelocityCommandRadS(j, 0.0);
                                }
                        }
			std::cout << "Resetting motor "<< i << std::endl;
			m_pPlatform_Driver->resetNode(i);
			joints_resurrection[i]++;
		}
                else
		{
			if (canParameters.Active[i]==ACTIVE)
                        {	
			    //! In case of inactivating a node stop the motion of the rest of the motor
                            for (register size_t j=0;j<static_cast<size_t>(numMotors);j++)
                            {
                                if (canParameters.Active[j])
                                {
                                        m_pPlatform_Driver->nodeVelocityCommandRadS(j, 0.0);
                                }
                            }
                            m_pPlatform_Driver->shutdownNode(i);
		    	    std::cout << "Motor " << i << " INACTIVE" << std::endl;
			    canParameters.Active[i]=INACTIVE;
                        }
		}
	}
    }

    /** Get the Passive joints information **/
    for(std::vector<platform_driver::AnalogId>::iterator it = passiveConfig.begin(); it != passiveConfig.end(); ++it)
    {
        m_pPlatform_Driver->getNodeAnalogInput(it->id, &dAnalogInput);

        base::JointState &joint(joints_readings[it->name]);
        if (it->id == 0)
            joint.position = (-(dAnalogInput-2.5)*bogie_pitch_factor)*D2R;
        else
            joint.position = (dAnalogInput-2.5)*bogie_pitch_factor*D2R;
    }

    /** Get the Analog System information (Voltage) **/
    m_pPlatform_Driver->getNodeAnalogInput(analogConfig[0].id, &dAnalogInput);
    base::JointState &joint_voltage(joints_readings[analogConfig[0].name]);
    joint_voltage.raw = (dAnalogInput-2.5)*system_voltage_factor;

    /** Get the Analog System information (Current) **/
    m_pPlatform_Driver->getNodeAnalogInput(analogConfig[1].id, &dAnalogInput);
    base::JointState &joint_current(joints_readings[analogConfig[1].name]);
    joint_current.raw = (dAnalogInput-2.5)*system_current_factor;

    _joints_readings.write(joints_readings);

}

void Task::errorHook()
{
    TaskBase::errorHook();
    RTT::log(RTT::Info)<<"platform_driver::Task: Entering the error hook!"<<RTT::endlog();
    m_pPlatform_Driver->shutdownPltf();
}

void Task::stopHook()
{
    TaskBase::stopHook();
    RTT::log(RTT::Info)<<"platform_driver::Task: Entering the stop hook!"<<RTT::endlog();
    /** Emergency stop of the motors **/
    m_pPlatform_Driver->shutdownPltf();

}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    RTT::log(RTT::Info)<<"platform_driver::Task: Entering the cleanup hook!"<<RTT::endlog();
    m_pPlatform_Driver->shutdownPltf();

    delete m_pPlatform_Driver;
}

