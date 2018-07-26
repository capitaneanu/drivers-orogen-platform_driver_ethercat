#include "Hdpr.hpp"

using namespace platform_driver;

Hdpr::Hdpr(std::string const& name)
    : HdprBase(name)
{
}

Hdpr::Hdpr(std::string const& name, RTT::ExecutionEngine* engine)
    : HdprBase(name, engine)
{
}

Hdpr::~Hdpr()
{
}

void Task::setJointCommands()
{
    if (_joints_commands.read(joints_commands, false) == RTT::NewData)
    {
        for (size_t i = 0; i < joints_commands.size(); ++i)
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
                    if (start_motor[i] && i<6)
                    {
                        m_pPlatform_Driver->startNode(i);
                        start_motor[i]=false;
                    }
                    m_pPlatform_Driver->nodeVelocityCommandRadS(i, joint.speed);

                    if (i==10 && joint.speed==0)
                    {
                        for (int j=0;j<6;j++)
                        {
                            stop_motor[j]=true;
                        }
                    }
                    else if (i<6)
                    {
                        stop_motor[i]=false;
                    }
                }
            }
        }
    }
}

void Task::getJointInformation()
{
    bool error_in_motor=false;
    for (int i=0; i < numMotors; i++)
    {
        bool status=m_pPlatform_Driver->getNodeData(i, &dPositionRad, &dVelocityRadS, &dCurrentAmp, &dTorqueNm);

        /** Joints readings & status information **/
        base::JointState& joint(joints_readings[i]);
        joint.position = dPositionRad;
        joint.speed = dVelocityRadS;
        if (stop_motor[i] && i<6 && std::abs(joint.speed)<0.01){
            //std::cout<< "stop motor: " << i << std::endl;
            m_pPlatform_Driver->nodeTorqueCommandNm(i,0.0);
            m_pPlatform_Driver->shutdownNode(i);
            stop_motor[i]=false;
            start_motor[i]=true;
        }
        joint.raw = dCurrentAmp;
        joint.effort = dTorqueNm;

        if (canParameters.Active[i] && status)
            joints_status[i]=true;
        else
            joints_status[i]=false;

        if (!status)
        {
            if (joints_resurrection[i] < 3)
            {
                //! In case of reseting a node stop the motion of the rest of the motor
                for (size_t j = 0; j < static_cast<size_t>(numMotors); ++j)
                {
                    if (canParameters.Active[j])
                    {
                        m_pPlatform_Driver->nodeVelocityCommandRadS(j, 0.0);
                    }
                }
                //std::cout << "Resetting motor "<< i << std::endl;
                m_pPlatform_Driver->resetNode(i);
                joints_resurrection[i]++;
            }
            else
            {
                if (canParameters.Active[i] == ACTIVE)
                {
                    //! In case of inactivating a node stop the motion of the rest of the motor
                    for (size_t j = 0; j < static_cast<size_t>(numMotors); ++j)
                    {
                        if (canParameters.Active[j])
                        {
                            m_pPlatform_Driver->nodeVelocityCommandRadS(j, 0.0);
                        }
                    }
                    m_pPlatform_Driver->shutdownNode(i);
                    //std::cout << "Motor " << i << " INACTIVE" << std::endl;
                    canParameters.Active[i]=INACTIVE;
                }
            }
            _error_in_motor.write(i+1);
            error_in_motor=true;
        }
    }
    if (!error_in_motor)
        _error_in_motor.write(0);

    /** Get the Passive joints information **/
    for(const auto& passive_joint : passiveConfig)
    {
        m_pPlatform_Driver->getNodeAnalogInput(passive_joint.id, &dAnalogInput);

        base::JointState& joint(joints_readings[passive_joint.name]);
        if (passive_joint.id == 3 || passive_joint.id == 5)
            joint.position = (-dAnalogInput)*bogie_pitch_factor;//*D2R;
        else
            joint.position = (dAnalogInput)*bogie_pitch_factor;//*D2R;
    }
}