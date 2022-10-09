#pragma once

#include <memory>
#include <functional>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

class MoveitcppNode
{
    public:

    MoveitcppNode(const rclcpp::Node::SharedPtr& node, const std::string& joint_group_name);

    void executePoseGoal();

    private:

    std::shared_ptr<moveit_cpp::MoveItCpp> m_MoveitcppPtr;
    std::shared_ptr<moveit_cpp::PlanningComponent> m_PlanningComponent;
    std::shared_ptr<const moveit::core::RobotModel> m_RobotModel;
    std::shared_ptr<moveit::core::RobotState> m_RobotState;
    const moveit::core::JointModelGroup* m_JointModelGroup;

    rclcpp::Node::SharedPtr m_Node;

    std::string m_PlanningGroupName;

};
