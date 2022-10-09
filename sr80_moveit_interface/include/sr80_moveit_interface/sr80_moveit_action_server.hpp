// C++ standart library includes
#include <functional>
#include <memory>
#include <thread>

// ROS2 includes 
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <sr80_custom_interfaces/action/pose_goal.hpp>
#include <sr80_custom_interfaces/action/joint_space_goal.hpp>

class SR80MoveitActionServer : public rclcpp::Node
{
    public:

    explicit SR80MoveitActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:

    rclcpp_action::Server<sr80_custom_interfaces::action::PoseGoal>::SharedPtr m_PoseGoalServer;
    rclcpp_action::Server<sr80_custom_interfaces::action::JointSpaceGoal>::SharedPtr m_JointGoalServer;

    moveit::planning_interface::MoveGroupInterfacePtr m_MoveGroupInterface;
    moveit::planning_interface::MoveGroupInterface::Plan m_MoveGroupPlan;

    void plan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goalHandle);

    // Handler callback functions:

    // Goal-Handling function for 
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const sr80_custom_interfaces::action::PoseGoal::Goal> poseGoalRequest);

    rclcpp_action::GoalResponse handle_joint_space_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const sr80_custom_interfaces::action::JointSpaceGoal::Goal> jointSpaceGoalRequest);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goalHandle);

    rclcpp_action::CancelResponse handle_joint_space_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>>& jointSpaceGoalHandle);    

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goalHandle);

    void handle_joint_space_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>>& jointSpaceGoalHandle);

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goalHandle);

    void execute_joint_space_goal(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>>& jointSpaceGoalHandle);

    //void CheckForCancelReq(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goalHandle);

};