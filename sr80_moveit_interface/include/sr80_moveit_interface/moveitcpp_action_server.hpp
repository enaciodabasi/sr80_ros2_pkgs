#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>


#include <sr80_custom_interfaces/action/pose_goal.hpp>
#include <sr80_custom_interfaces/action/joint_space_goal.hpp>
#include <sr80_custom_interfaces/action/servo_goal.hpp>

#include <memory>
#include <vector>

class MoveitServoNode;

class MoveitcppNode
{
    public:

    MoveitcppNode(const rclcpp::Node::SharedPtr& node, const std::string& joint_group_name);

    moveit_cpp::PlanningComponent::PlanSolution generate_plan(const geometry_msgs::msg::PoseStamped& pose_goal, const std::string& end_effector_name);
    moveit_cpp::PlanningComponent::PlanSolution generate_plan(const std::vector<double>& joint_goal);
    void executePoseGoal();
    void executeJointGoal();

    std::vector<double> degToRad(const std::vector<double>& deg_vals);

    moveit_cpp::MoveItCppPtr getMoveitCppPtr();

    private:

    std::shared_ptr<moveit_cpp::MoveItCpp> m_MoveitcppPtr;
    std::shared_ptr<moveit_cpp::PlanningComponent> m_PlanningComponent;
    std::shared_ptr<const moveit::core::RobotModel> m_RobotModel;
    std::shared_ptr<moveit::core::RobotState> m_RobotState;
    const moveit::core::JointModelGroup* m_JointModelGroup;

    rclcpp::Node::SharedPtr m_Node;
    std::shared_ptr<MoveitServoNode> m_MoveitServoNodePtr;

    std::string m_PlanningGroupName;

    moveit_cpp::PlanningComponent::PlanSolution m_PlanSolution;

};

class MoveitServoNode
{
    public:

    MoveitServoNode(const rclcpp::Node::SharedPtr& node, const moveit_cpp::MoveItCppPtr& moveitcpp_ptr);

    void startServo();
    void pauseServo();
    void unpauseServo();
    bool isServoActive();
    void setServoActiveFlag(bool servo_active_flag);

    void send_joint_cmd(const control_msgs::msg::JointJog& joint_jog_msg);
    void send_twist_cmd(const geometry_msgs::msg::TwistStamped& twist_stamped_msg);

    private:
    
    rclcpp::Node::SharedPtr m_Node;
    std::shared_ptr<moveit_cpp::MoveItCpp> m_MoveitCppPtr;

    std::unique_ptr<moveit_servo::Servo> m_Servo;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> m_PlanningSceneMonitor;

    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr m_JointCmdPublisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_TwistCmdPublisher;

    bool m_ServoActiveFlag = false;
};

class MoveitcppActionServer : public rclcpp::Node
{
    public:

    MoveitcppActionServer(rclcpp::NodeOptions options);

    void initMoveitcpp(const rclcpp::Node::SharedPtr& node, const std::string& group_name);
    void initMoveitServo(const rclcpp::Node::SharedPtr& node);

    private:

    MoveitcppNode* m_MoveitcppNode;
    MoveitServoNode* m_MoveitServoNode;
    
    rclcpp_action::Server<sr80_custom_interfaces::action::PoseGoal>::SharedPtr m_PoseActionServer;
    rclcpp_action::GoalResponse handle_goal_pose(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const sr80_custom_interfaces::action::PoseGoal::Goal> goal_request);
    rclcpp_action::CancelResponse handle_cancel_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goal_handle);
    void handle_accepted_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goal_handle);
    void execute_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goal_handle);

    rclcpp_action::Server<sr80_custom_interfaces::action::JointSpaceGoal>::SharedPtr m_JointActionServer;
    rclcpp_action::GoalResponse handle_goal_joint(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const sr80_custom_interfaces::action::JointSpaceGoal::Goal> goal_request);
    rclcpp_action::CancelResponse handle_cancel_joint(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>>& goal_handle);
    void handle_accepted_joint(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>>& goal_handle);
    // Executes Joint Space Goal using the MoveitcppNode class
    void execute_joint(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>>& goal_handle);    
    
    rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr m_JointJogSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_TwistStampedSubscriber;

    void callback_joint_jog(const control_msgs::msg::JointJog& joint_jog_msg) const;
    void callback_twist_stamped(const geometry_msgs::msg::TwistStamped& stamped_twist_msg) const;

};