#ifndef MITSUBISHIARM_VREPHW_H_AIHUXBVT
#define MITSUBISHIARM_VREPHW_H_AIHUXBVT

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


namespace MR
{


enum RobotJointsEnum
{
    J1 = 0,
    J2,
    J3,
    J4,
    J5,
    J6,

    ROBOT_JOINTS_NUM
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief This is the hardware interface for a 6DOF Mitsubishi arm  simulated in vrep.
class MitsubishiArm_vrepHW : public hardware_interface::RobotHW
{
public:
    MitsubishiArm_vrepHW();

    bool init();

    bool read();
    bool write();

protected:
    static std::string sm_jointsName[ROBOT_JOINTS_NUM];

    // Vrep handles.
    int m_vrepJointsHandle[ROBOT_JOINTS_NUM];

    // Interfaces.
    double m_cmd[ROBOT_JOINTS_NUM];
    double m_pos[ROBOT_JOINTS_NUM];
    double m_vel[ROBOT_JOINTS_NUM];
    double m_eff[ROBOT_JOINTS_NUM];

    hardware_interface::JointStateInterface m_jointState_interface;
    hardware_interface::PositionJointInterface m_jointPosition_interface;

    void registerHardwareInterfaces();
};


} // namespace MR.
#endif /* end of include guard: MITSUBISHIARM_VREPHW_H_AIHUXBVT */
