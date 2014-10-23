#include "MyRobot_vrepHW.h"

#include "../v_repLib.h"

#include <string>
#include <iostream>


namespace MR
{


// Joint names in V-REP
std::string MyRobot_vrepHW::sm_jointsName[MR_JOINTS_NUM] = {
    "j1",
    "j2",
    "j3",
    "j4",
    "j5",
    "j6"
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MyRobot_vrepHW::MyRobot_vrepHW() :
    hardware_interface::RobotHW()
{
    // Init arrays m_cmd[], m_pos[], m_vel[], m_eff[].
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        m_cmd[i] = 0.0;
        m_pos[i] = 0.0;
        m_vel[i] = 0.0;
        m_eff[i] = 0.0;
    }

    // Init and get handles of the joints to control.
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
        m_vrepJointsHandle[i] = -1;

    // Register joint interfaces.
    registerHardwareInterfaces();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MyRobot_vrepHW::init()
{
    // Get joint handles.
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        int vrepJointsHandle = simGetObjectHandle(sm_jointsName[i].c_str());

        if (vrepJointsHandle == -1)
        {
            ROS_ERROR_STREAM("MR robot interface not able to get handle for '" << sm_jointsName[i] << "'." << std::endl);

            return false;
        }

        ROS_INFO_STREAM("Joint handle for " << sm_jointsName[i] << ": " << vrepJointsHandle);
        m_vrepJointsHandle[i] = vrepJointsHandle;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MyRobot_vrepHW::registerHardwareInterfaces()
{
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        // Joint state interface.
        hardware_interface::JointStateHandle jointStateHandle(sm_jointsName[i], &m_pos[i], &m_vel[i], &m_eff[i]);
        m_jointState_interface.registerHandle(jointStateHandle);

        // Joint command interface (in MyRobot's case this is a velocity interface).
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &m_cmd[i]);
        m_jointPosition_interface.registerHandle(jointPositionHandle);
    }

    registerInterface(&m_jointState_interface);
    registerInterface(&m_jointPosition_interface);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MyRobot_vrepHW::read()
{
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        float pos;

        if (simGetJointPosition(m_vrepJointsHandle[i], &pos) == -1)
        {
            ROS_ERROR_STREAM("MR robot interface not able to get state for '" << sm_jointsName[i] << "'." << std::endl);

            return false;
        }

        m_pos[i] = pos;
        m_vel[i] = 0.0;
        m_eff[i] = 0.0;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MyRobot_vrepHW::write()
{
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        if (simSetJointPosition(m_vrepJointsHandle[i], m_cmd[i]) == -1)
        {
            ROS_ERROR_STREAM("MR robot interface not able to write state for '" << sm_jointsName[i] << "'." << std::endl);

            return false;
        }
    }

    return true;
}

} // namespace MR.
