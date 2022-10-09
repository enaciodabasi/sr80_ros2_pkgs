#include "../include/sr80_moveit_interface/sr80_moveit_action_server.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>

#include <vector>

SR80MoveitActionServer::SR80MoveitActionServer(const rclcpp::NodeOptions& options)
    : Node("sr80_moveit_interface_server", options)
{
    using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(), "Creating SR80 Moveit Action Server Node");
    this->m_PoseGoalServer = rclcpp_action::create_server<sr80_custom_interfaces::action::PoseGoal>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "sr80_pose_goal",
        std::bind(&SR80MoveitActionServer::handle_goal, this, _1, _2),
        std::bind(&SR80MoveitActionServer::handle_cancel, this, _1),
        std::bind(&SR80MoveitActionServer::handle_accepted, this, _1)
    );

    this->m_JointGoalServer = rclcpp_action::create_server<sr80_custom_interfaces::action::JointSpaceGoal>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "sr80_joint_space_goal",
        std::bind(&SR80MoveitActionServer::handle_joint_space_goal, this, _1, _2),
        std::bind(&SR80MoveitActionServer::handle_joint_space_cancel, this, _1),
        std::bind(&SR80MoveitActionServer::handle_joint_space_accepted, this, _1)
    );
}

// POSE GOAL

rclcpp_action::GoalResponse SR80MoveitActionServer::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const sr80_custom_interfaces::action::PoseGoal::Goal> poseGoalRequest)
{
    
    RCLCPP_INFO(this->get_logger(), "Received goal request of type Pose");
    const auto pos_x = poseGoalRequest->request.requested_pose.position.x;
    const auto pos_y = poseGoalRequest->request.requested_pose.position.y;
    const auto pos_z = poseGoalRequest->request.requested_pose.position.z;
    const auto orient_x = poseGoalRequest->request.requested_pose.orientation.x;
    const auto orient_y = poseGoalRequest->request.requested_pose.orientation.y;
    const auto orient_z = poseGoalRequest->request.requested_pose.orientation.z;
    const auto orient_w = poseGoalRequest->request.requested_pose.orientation.w;

    RCLCPP_INFO(this->get_logger(), "Requested position: x: %.f y: %.f z: %.f", pos_x, pos_y, pos_z);
    RCLCPP_INFO(this->get_logger(), "Requested orientation: x: %.f y: %.f z: %.f w: %.f", orient_x, orient_y, orient_z, orient_w);

    // moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "sr80_arm");
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // moveit::planning_interface::MoveGroupInterface::Plan plan;

    // move_group.setPoseTarget(poseGoalRequest->request.requested_pose);
    
    // bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(!success){
    //     return rclcpp_action::GoalResponse::REJECT;
    // }

    // move_group.execute(plan);

    //robot_model_loader::RobotModelLoaderPtr robot_model_loader_(new robot_model_loader::RobotModelLoader(this->shared_from_this(), "robot_description"));
    //planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_(new planning_scene_monitor::PlanningSceneMonitor(this->shared_from_this(), robot_model_loader_));
//
    //planning_scene_monitor_->startSceneMonitor();
    //planning_scene_monitor_->startWorldGeometryMonitor();
    //planning_scene_monitor_->startStateMonitor();
//
    //moveit::core::RobotModelPtr robot_model = robot_model_loader_->getModel();
//
    //moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState()));

   // moveit::planning_interface::MoveGroupInterface move_group_arm_(this->shared_from_this(), "sr80_arm");
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    
    //moveit::planning_interface::MoveGroupInterface::Plan posePlan_;
    //const moveit::core::JointModelGroup* joint_model_group_arm_(robot_state->getJointModelGroup("sr80_arm"));
    //std::vector<double> joint_group_positions_arm;

    //std::copy(move_group_arm_.getJointModelGroupNames().begin(), move_group_arm_.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    //current_state_arm_ = move_group_arm_.getCurrentState(100);
    //current_state_arm_->copyJointGroupPositions(joint_model_group_arm_, joint_group_positions_arm);                

    //auto goalPose = poseGoalRequest->request.requested_pose;
    //move_group_arm_.setPoseTarget(goalPose);
    //bool success = (move_group_arm_.plan(posePlan_) == moveit::core::MoveItErrorCode::SUCCESS);

    //if(!success)
    //   return rclcpp_action::GoalResponse::REJECT;

    //move_group_arm_.execute(posePlan_);

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SR80MoveitActionServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goalHandle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
    (void)goalHandle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SR80MoveitActionServer::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goalHandle)
{
    using namespace std::placeholders;

    std::thread{std::bind(&SR80MoveitActionServer::execute, this, _1), goalHandle}.detach();
}

void SR80MoveitActionServer::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goalHandle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal...");

    rclcpp::Time time_before_execute = this->get_clock()->now();
    const auto goal = goalHandle->get_goal();
    
    moveit::planning_interface::MoveGroupInterface move_group_arm_(this->shared_from_this(), "sr80_arm");
    moveit::planning_interface::MoveGroupInterface::Plan posePlan_;

    auto goalPose = goal->request.requested_pose;
    move_group_arm_.setPoseTarget(goalPose);
    bool success = (move_group_arm_.plan(posePlan_) == moveit::core::MoveItErrorCode::SUCCESS);

    move_group_arm_.execute(posePlan_);

    rclcpp::Time time_after_execute = this->get_clock()->now();
    auto stamp = time_after_execute - time_before_execute;
    builtin_interfaces::msg::Time et;
    
    auto result = std::make_shared<sr80_custom_interfaces::action::PoseGoal::Result>();
    result->result.final_pose = move_group_arm_.getCurrentPose("tool0").pose;
       
    if(rclcpp::ok())
    {
       goalHandle->succeed(result);
       RCLCPP_INFO(this->get_logger(), "Goal succeeded");   
    }

}

// JOINT SPACE GOAL

rclcpp_action::GoalResponse SR80MoveitActionServer::handle_joint_space_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const sr80_custom_interfaces::action::JointSpaceGoal::Goal> jointSpaceGoalRequest
)
{   

    RCLCPP_INFO(this->get_logger(), "Received joint space goal from client, planning...");

    std::vector<float> joint_goals;
    std::copy(
        jointSpaceGoalRequest->request.joint_positions.begin(),
        jointSpaceGoalRequest->request.joint_positions.end(),
        std::back_inserter(joint_goals)
    );

    m_MoveGroupInterface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "sr80_arm");
    
    //const moveit::core::JointModelGroup* joint_model_group = m_MoveGroupInterface->getCurrentState()->getJointModelGroup("sr80_arm");
    //moveit::core::RobotStatePtr current_robot_state = m_MoveGroupInterface->getCurrentState(50);

    robot_model_loader::RobotModelLoaderPtr rml(new robot_model_loader::RobotModelLoader(this->shared_from_this(), "robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(this->shared_from_this(), rml));
    
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();

    moveit::core::RobotStatePtr current_robot_state(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
    const moveit::core::JointModelGroup* joint_model_group(current_robot_state->getJointModelGroup("sr80_arm"));
    
    std::vector<double> joint_group_positions;
    current_robot_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    std::cout << joint_group_positions.size() << std::endl;
    std::cout << joint_goals.size() << std::endl;

    for(int i = 0; i < joint_group_positions.size(); i++)
    {
        joint_group_positions[i] = joint_goals[i];
    }

    m_MoveGroupInterface->setJointValueTarget(joint_group_positions);

    bool plan_successful = (m_MoveGroupInterface->plan(m_MoveGroupPlan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(!plan_successful)
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Joint state goal is valid, executing...");

    (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse SR80MoveitActionServer::handle_joint_space_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>>& jointSpaceGoalHandle)
{  
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
    (void)jointSpaceGoalHandle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SR80MoveitActionServer::handle_joint_space_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>>& jointSpaceGoalHandle)
{
    using namespace std::placeholders;

    std::thread{std::bind(&SR80MoveitActionServer::execute_joint_space_goal, this, _1), jointSpaceGoalHandle}.detach();

}

void SR80MoveitActionServer::execute_joint_space_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>>& jointSpaceGoalHandle)
{   
    
    m_MoveGroupInterface->execute(m_MoveGroupPlan);

    auto result = std::make_shared<sr80_custom_interfaces::action::JointSpaceGoal::Result>();
    
    //result->result.final_joint_positions = m_MoveGroupInterface->getCurrentJointValues();

    if(rclcpp::ok())
    {
        jointSpaceGoalHandle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Joint State Goal is successful.");
        RCLCPP_INFO(this->get_logger(), "Waiting for the next goal to arrive.");
    }
    
}


RCLCPP_COMPONENTS_REGISTER_NODE(SR80MoveitActionServer)

//int main(int argc, char** argv)
//{
//
//    rclcpp::init(argc, argv);
//
//    auto node = std::make_shared<SR80MoveitActionServer>();
//    rclcpp::spin(node);
//
//    rclcpp::shutdown();
//
//    return 0;
//}

