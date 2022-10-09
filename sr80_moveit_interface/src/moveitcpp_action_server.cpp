#include "../include/sr80_moveit_interface/moveitcpp_action_server.hpp"

#include <math.h>

MoveitcppNode::MoveitcppNode(const rclcpp::Node::SharedPtr& node, const std::string& planning_group_name)
    : m_Node(node), m_PlanningGroupName(planning_group_name)
{

    m_MoveitcppPtr = std::make_shared<moveit_cpp::MoveItCpp>(m_Node);
    m_MoveitcppPtr->getPlanningSceneMonitor()->providePlanningSceneService();

    m_PlanningComponent = std::make_shared<moveit_cpp::PlanningComponent>(m_PlanningGroupName, m_MoveitcppPtr);

    m_RobotModel = m_MoveitcppPtr->getRobotModel();
    m_RobotState = m_PlanningComponent->getStartState();
    m_JointModelGroup = m_RobotModel->getJointModelGroup(m_PlanningGroupName);

    //ps.header.frame_id = "base_link";
    //ps.pose.orientation.w = 0;
    //ps.pose.orientation.x = 0.7071;
    //ps.pose.orientation.y = 0.7071;
    //ps.pose.orientation.z = 0;
    //ps.pose.position.x = 1.682;
    //ps.pose.position.y = 0;
    //ps.pose.position.z = 0.49;


}

moveit_cpp::MoveItCppPtr MoveitcppNode::getMoveitCppPtr()
{
    return this->m_MoveitcppPtr;
}

void MoveitcppNode::executePoseGoal()
{
    if(m_PlanSolution)
        m_PlanningComponent->execute();
}

moveit_cpp::PlanningComponent::PlanSolution MoveitcppNode::generate_plan(const geometry_msgs::msg::PoseStamped& pose_goal, const std::string& end_effector_name)
{   

    m_PlanningComponent->setStartStateToCurrentState();

    m_PlanningComponent->setGoal(pose_goal, end_effector_name);

    m_PlanSolution = m_PlanningComponent->plan();

    return m_PlanSolution;
}

moveit_cpp::PlanningComponent::PlanSolution MoveitcppNode::generate_plan(const std::vector<double>& joint_goal)
{
    std::vector<double> jointGroupPositions;
    m_RobotState->copyJointGroupPositions(m_JointModelGroup, jointGroupPositions);

    std::vector<double> joint_goal_rad = degToRad(joint_goal);

    for(std::size_t i = 0; i < jointGroupPositions.size(); ++i)
    {
        jointGroupPositions.at(i) = joint_goal_rad.at(i);
    }

    auto goalState = *(m_MoveitcppPtr->getCurrentState());
    goalState.setJointGroupPositions(m_JointModelGroup, jointGroupPositions);

    m_PlanningComponent->setGoal(goalState);

    m_PlanSolution = m_PlanningComponent->plan();

    return m_PlanSolution;
}

std::vector<double> MoveitcppNode::degToRad(const std::vector<double>& deg_vals)
{   
    std::vector<double> rads;

    for(auto d : deg_vals)
    {
        rads.emplace_back(d*(M_PI/180));
    }

    return rads;
}

void MoveitcppNode::executeJointGoal()
{
    if(m_PlanSolution)
        m_PlanningComponent->execute();
}

// -------------------------------------------------------------

MoveitServoNode::MoveitServoNode(const rclcpp::Node::SharedPtr& node, const moveit_cpp::MoveItCppPtr& moveitcpp_ptr)
    : m_Node{node}, m_MoveitCppPtr{moveitcpp_ptr}
{

    m_PlanningSceneMonitor = m_MoveitCppPtr->getPlanningSceneMonitor();
    m_PlanningSceneMonitor->startStateMonitor("/joint_states");
    m_PlanningSceneMonitor->setPlanningScenePublishingFrequency(25);
    m_PlanningSceneMonitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "/moveit_servo/publish_planning_scene");
    m_PlanningSceneMonitor->startSceneMonitor();
    m_PlanningSceneMonitor->providePlanningSceneService();

    this->m_JointCmdPublisher = m_Node->create_publisher<control_msgs::msg::JointJog>(
        "/action_server/delta_joint_cmds",
        10
    );

    this->m_TwistCmdPublisher = m_Node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/action_server/delta_twist_cmds",
        10
    );

    auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(m_Node);
    if(!servo_parameters)
    {
        RCLCPP_FATAL(m_Node->get_logger(), "Failed to load the servo parameters");
        rclcpp::shutdown();
    }

    RCLCPP_INFO(m_Node->get_logger(), "Creating servo node");
    m_Servo = std::make_unique<moveit_servo::Servo>(m_Node, servo_parameters, m_PlanningSceneMonitor);
    m_Servo->start();
    m_ServoActiveFlag = true;
    RCLCPP_INFO(m_Node->get_logger(), "Starting the servo");
}

void MoveitServoNode::startServo()
{
    if(m_Servo != nullptr)
    {
        m_Servo->start();
        m_ServoActiveFlag = true;
    }
}

void MoveitServoNode::pauseServo()
{
    if(m_Servo != nullptr)
    {

        m_Servo->setPaused(true);
        m_ServoActiveFlag = false;
    }
}

void MoveitServoNode::unpauseServo()
{
    if(m_Servo != nullptr)
    {
        m_Servo->setPaused(false);
        m_ServoActiveFlag = true;
    }
}

bool MoveitServoNode::isServoActive()
{
    return m_ServoActiveFlag;
}

void MoveitServoNode::setServoActiveFlag(bool servo_active_flag)
{
    this->m_ServoActiveFlag = servo_active_flag;
}

void MoveitServoNode::send_joint_cmd(const control_msgs::msg::JointJog& joint_jog_msg)
{   
    if(m_ServoActiveFlag)
        m_JointCmdPublisher->publish(joint_jog_msg);
    else
    {
        unpauseServo();
        m_JointCmdPublisher->publish(joint_jog_msg);
    }

}

void MoveitServoNode::send_twist_cmd(const geometry_msgs::msg::TwistStamped& twist_stamped_msg)
{
    if(m_ServoActiveFlag)
        m_TwistCmdPublisher->publish(twist_stamped_msg);
    else
    {
        unpauseServo();
        m_TwistCmdPublisher->publish(twist_stamped_msg);
    }

}

// -------------------------------------------------------------

MoveitcppActionServer::MoveitcppActionServer(rclcpp::NodeOptions options)
    : Node("moveitcpp_action_server_node", options)
{
    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(), "Creating Action Server for pose goals.");
    this->m_PoseActionServer = rclcpp_action::create_server<sr80_custom_interfaces::action::PoseGoal>(
        this,
        "sr80_pose_action",
        std::bind(&MoveitcppActionServer::handle_goal_pose, this, _1, _2),
        std::bind(&MoveitcppActionServer::handle_cancel_pose, this, _1),
        std::bind(&MoveitcppActionServer::handle_accepted_pose, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "Action Server for pose goals is created successfully.");

    RCLCPP_INFO(this->get_logger(), "Creating Action Server for joint space goals.");
    this->m_JointActionServer = rclcpp_action::create_server<sr80_custom_interfaces::action::JointSpaceGoal>(
        this,
        "sr80_joint_action",
        std::bind(&MoveitcppActionServer::handle_goal_joint, this, _1, _2),
        std::bind(&MoveitcppActionServer::handle_cancel_joint, this, _1),
        std::bind(&MoveitcppActionServer::handle_accepted_joint, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "Action Server for joint space goals is created successfully.");

    RCLCPP_INFO(this->get_logger(), "Creating subscriptions for the Servo Node");

    this->m_JointJogSubscriber = this->create_subscription<control_msgs::msg::JointJog>(
        "joint_jog_data",
        10,
        std::bind(&MoveitcppActionServer::callback_joint_jog, this, _1)
    );

    this->m_TwistStampedSubscriber = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "stamped_twist_data",
        10,
        std::bind(&MoveitcppActionServer::callback_twist_stamped, this, _1)
    );

}

// Pose Goal Callback Functions

rclcpp_action::GoalResponse MoveitcppActionServer::handle_goal_pose(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const sr80_custom_interfaces::action::PoseGoal::Goal> goal_request)
{
    
    // Plan the path for the given pose goal
    // If plan is false reject the goal request
    if(!m_MoveitcppNode->generate_plan(goal_request->request.requested_pose, "tool0"))   
    {
        return rclcpp_action::GoalResponse::REJECT;
    }
    // Else Accept and Execute the goal request

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveitcppActionServer::handle_cancel_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveitcppActionServer::handle_accepted_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goal_handle)
{
    using namespace std::placeholders;

    std::thread{std::bind(&MoveitcppActionServer::execute_pose, this, _1), goal_handle}.detach();
}

void MoveitcppActionServer::execute_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::PoseGoal>>& goal_handle)
{
    this->m_MoveitcppNode->executePoseGoal();
    auto result = std::make_shared<sr80_custom_interfaces::action::PoseGoal::Result>();
    if(rclcpp::ok())
    {
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Pose goal succeeded.");
    }
}

// Joint Goal Callback Functions

rclcpp_action::GoalResponse MoveitcppActionServer::handle_goal_joint(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const sr80_custom_interfaces::action::JointSpaceGoal::Goal> goal_request)
{
    if(!m_MoveitcppNode->generate_plan(std::vector<double>(goal_request->request.joint_positions.double_array.begin(), goal_request->request.joint_positions.double_array.end())))
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveitcppActionServer::handle_cancel_joint(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>>& goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveitcppActionServer::handle_accepted_joint(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>>& goal_handle)
{
    using namespace std::placeholders;

    std::thread{std::bind(&MoveitcppActionServer::execute_joint, this, _1), goal_handle}.detach();
}

void MoveitcppActionServer::execute_joint(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>>& goal_handle)
{
    this->m_MoveitcppNode->executeJointGoal();
    auto result = std::make_shared<sr80_custom_interfaces::action::JointSpaceGoal::Result>();
    if(rclcpp::ok())
    {
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Pose goal succeeded.");
    }
}

// Servo Functions

void MoveitcppActionServer::callback_joint_jog(const control_msgs::msg::JointJog& joint_jog_msg) const
{
    this->m_MoveitServoNode->send_joint_cmd(joint_jog_msg);
}

void MoveitcppActionServer::callback_twist_stamped(const geometry_msgs::msg::TwistStamped& stamped_twist_msg) const
{
    this->m_MoveitServoNode->send_twist_cmd(stamped_twist_msg);
}

void MoveitcppActionServer::initMoveitcpp(const rclcpp::Node::SharedPtr& node, const std::string& group_name)
{
    m_MoveitcppNode = new MoveitcppNode(node, group_name);
}

void MoveitcppActionServer::initMoveitServo(const rclcpp::Node::SharedPtr& node)
{
    m_MoveitServoNode = new MoveitServoNode(node, m_MoveitcppNode->getMoveitCppPtr());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    //node_options.use_intra_process_comms(false);

    auto moveitcpp_action_server_node = std::make_shared<MoveitcppActionServer>(node_options);
    moveitcpp_action_server_node->initMoveitcpp(moveitcpp_action_server_node, "sr80_arm");
    

    rclcpp::NodeOptions servo_node_options;
    servo_node_options.automatically_declare_parameters_from_overrides(false);
    servo_node_options.use_intra_process_comms(false);
    auto moveit_servo_node = std::make_shared<rclcpp::Node>("servo_node", servo_node_options);
    moveitcpp_action_server_node->initMoveitServo(moveit_servo_node);

    rclcpp::executors::MultiThreadedExecutor mt_executor1;
    //rclcpp::executors::MultiThreadedExecutor mt_executor2;
    mt_executor1.add_node(moveitcpp_action_server_node);
    mt_executor1.add_node(moveit_servo_node);
    while(rclcpp::ok())
    {
        mt_executor1.spin();
        //mt_executor2.spin();
    }

    mt_executor1.remove_node(moveitcpp_action_server_node);
    mt_executor1.remove_node(moveit_servo_node);
    rclcpp::shutdown();

}