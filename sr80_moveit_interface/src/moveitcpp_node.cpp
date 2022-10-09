#include "../include/sr80_moveit_interface/moveitcpp_node.hpp"

MoveitcppNode::MoveitcppNode(const rclcpp::Node::SharedPtr& node, const std::string& planning_group_name)
    : m_Node(node), m_PlanningGroupName(planning_group_name)
{

    m_MoveitcppPtr = std::make_shared<moveit_cpp::MoveItCpp>(m_Node);
    m_MoveitcppPtr->getPlanningSceneMonitor()->providePlanningSceneService();

    m_PlanningComponent = std::make_shared<moveit_cpp::PlanningComponent>(m_PlanningGroupName, m_MoveitcppPtr);

    m_RobotModel = m_MoveitcppPtr->getRobotModel();
    m_RobotState = m_PlanningComponent->getStartState();
    m_JointModelGroup = m_RobotModel->getJointModelGroup(m_PlanningGroupName);

}
