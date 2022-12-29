#ifndef SR80_HW_HPP
#define SR80_HW_HPP

#include <ros/ros.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>

#include <iostream>

namespace SR80
{
    
    class Hardware : public hardware_interface::RobotHW
    {
        protected:
        
        std::vector<std::string> m_JointNames;

        int m_NumJoints;

        std::vector<double> m_JointPositions;
        std::vector<double> m_JointVelocities;
        std::vector<double> m_JointEfforts;

        std::vector<double> m_PositionCommand;
        std::vector<double> m_VelocityCommand;

        hardware_interface::JointStateInterface m_JointStateInterface;
        hardware_interface::PosVelJointInterface m_PosVelJointInterface;

    };

} // end of namespace SR80


#endif